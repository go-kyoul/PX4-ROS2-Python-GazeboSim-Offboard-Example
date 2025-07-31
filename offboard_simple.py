#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# PX4 Offboard - 초간단 학습용 (ACK 로그 포함, executor 없음)
# 절차:
#   1) 지면 근처에서 세트포인트 스트림 시작
#   2) ARM
#   3) OFFBOARD 모드 전환
#   4) 지정 고도까지 이륙
#   5) 웨이포인트들을 차례로 유지
#   6) 종료 (착륙/안전처리 없음, 의도적으로 단순화)
#
# 포인트:
#   - executor를 쓰지 않고, 루프마다 rclpy.spin_once(node, 0.0)로
#     ACK 구독 콜백만 처리합니다.
#   - 퍼블리셔 QoS는 기본값(정수 depth)로 충분하고,
#     ACK 구독은 PX4 out과 맞추기 위해 BEST_EFFORT만 지정합니다.

import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleCommandAck

# ---------- 설정 ----------
drones      = ['px4_1', 'px4_2', 'px4_3']   # 예: ['px4_1','px4_2']
takeoff_alt = 5.0                  # 이륙 고도 [m]
dt          = 0.1                  # 주기 [s] (오프보드 ≥2Hz 필요, 여기선 10Hz)

# 웨이포인트: (x, y, z, yaw_deg)
waypoints = [
    (  0.0,  0.0, takeoff_alt,   0.0),
    ( 10.0,  0.0, takeoff_alt,   0.0),
    ( 10.0, 10.0, takeoff_alt,  90.0),
    (  0.0, 10.0, takeoff_alt, 180.0),
]
stay_sec = 3.0                     # 각 웨이포인트에서 머무를 시간 [s]

# -------------------------------------------------


# -------------- 보조 함수들 --------------

def ns_to_sysid(ns: str) -> int:
    """네임스페이스를 MAV_SYS_ID로 매핑.
    규칙: ''→1, 'px4_1'→ 2, 'px4_2'→ 3
    """
    if not ns:
        return 1
    if ns.startswith('px4_'):
        try:
            return int(ns.split('_')[1]) + 1
        except Exception:
            return 1
    return 1


def print_ack(node: Node, ns: str, ack: VehicleCommandAck):
    """VehicleCommandAck를 간단히 출력(수락/거절 여부)."""
    # result: 0=ACCEPTED, 그 외는 거절/실패 코드
    result = "ACCEPTED" if ack.result == 0 else f"REJECTED({ack.result})"
    node.get_logger().info(f"[{ns or '(none)'}] ACK cmd={ack.command}: {result}")


def make_publishers(node: Node, names):
    """기체별 퍼블리셔(오프보드 플래그/세트포인트/명령)와 SYS_ID를 준비."""
    pub_off = {}
    pub_sp  = {}
    pub_cmd = {}
    sys_id  = {}
    for ns in names:
        prefix = f'/{ns}' if ns else ''
        # 퍼블리셔는 기본 QoS(여기선 depth=10만 전달)
        pub_off[ns] = node.create_publisher(OffboardControlMode, f'{prefix}/fmu/in/offboard_control_mode', 10)
        pub_sp[ns]  = node.create_publisher(TrajectorySetpoint,  f'{prefix}/fmu/in/trajectory_setpoint',   10)
        pub_cmd[ns] = node.create_publisher(VehicleCommand,      f'{prefix}/fmu/in/vehicle_command',       10)
        sys_id[ns]  = ns_to_sysid(ns)
        node.get_logger().info(f"[{ns or '(none)'}] SYS_ID = {sys_id[ns]}")
    return pub_off, pub_sp, pub_cmd, sys_id


def make_ack_subs(node: Node, names):
    """ACK 토픽 구독(로그만 출력). PX4 out과 맞추기 위해 BEST_EFFORT만 지정."""
    qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
    for ns in names:
        prefix = f'/{ns}' if ns else ''
        node.create_subscription(
            VehicleCommandAck,
            f'{prefix}/fmu/out/vehicle_command_ack',
            lambda ack, ns=ns: print_ack(node, ns, ack),
            qos
        )


def send_offboard_flag(node: Node, pub_off, ns: str):
    """오프보드 플래그 전송(포지션 제어 사용)."""
    msg = OffboardControlMode()
    msg.timestamp = node.get_clock().now().nanoseconds // 1000
    msg.position = True
    pub_off[ns].publish(msg)


def send_setpoint(node: Node, pub_sp, ns: str, x: float, y: float, z_up: float, yaw_deg: float):
    """위치 세트포인트 전송. 주의: PX4는 NED → z_up(위로 +)는 음수로 변환."""
    msg = TrajectorySetpoint()
    msg.timestamp = node.get_clock().now().nanoseconds // 1000
    msg.position = [float(x), float(y), float(-z_up)]  # z_up → NED z = -z_up
    msg.yaw = math.radians(yaw_deg)                    # yaw는 라디안
    pub_sp[ns].publish(msg)


def send_command(node: Node, pub_cmd, sys_id_map, ns: str, command: int, p1=0.0, p2=0.0):
    """VehicleCommand 전송(ARM/모드 전환 등)."""
    msg = VehicleCommand()
    msg.timestamp = node.get_clock().now().nanoseconds // 1000
    msg.command = command
    msg.param1 = float(p1)
    msg.param2 = float(p2)
    msg.target_system = sys_id_map[ns]
    msg.target_component = 1
    msg.source_system = 200 + sys_id_map[ns]  # 임의(식별용)
    msg.source_component = 1
    msg.from_external = True
    pub_cmd[ns].publish(msg)


def arm(node: Node, pub_cmd, sys_id_map, ns: str):
    """ARM 명령."""
    send_command(node, pub_cmd, sys_id_map, ns, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
    node.get_logger().info(f"[{ns or '(none)'}] Sent ARM")


def set_offboard(node: Node, pub_cmd, sys_id_map, ns: str):
    """OFFBOARD 모드 전환(main_mode=1, sub_mode=6)."""
    send_command(node, pub_cmd, sys_id_map, ns, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
    node.get_logger().info(f"[{ns or '(none)'}] Sent DO_SET_MODE(OFFBOARD)")


def send_loop(node: Node, seconds: float, poses: dict, pub_off, pub_sp):
    """지정 시간 동안 모든 기체에 대해 오프보드 플래그 + 세트포인트를 주기적으로 전송.
    poses: { ns: (x, y, z_up, yaw_deg) }
    - executor 없이 rclpy.spin_once(node, 0.0)로 ACK 콜백만 처리합니다.
    """
    loops = max(1, int(seconds / dt))
    for _ in range(loops):
        for ns, (x, y, z, yaw) in poses.items():
            send_offboard_flag(node, pub_off, ns)
            send_setpoint(node, pub_sp, ns, x, y, z, yaw)
        rclpy.spin_once(node, timeout_sec=0.0)  # ACK 콜백 처리
        time.sleep(dt)


# -------------- main(흐름만 한눈에) --------------

def main():
    rclpy.init()
    node = Node('offboard_super_basic_ack_spinonce_kr')

    # 퍼블리셔/시스템ID 준비 + ACK 구독
    pub_off, pub_sp, pub_cmd, sys_id_map = make_publishers(node, drones)
    make_ack_subs(node, drones)

    # 시작 위치(간단 간격 적용)
    start_pose = {}
    for i, ns in enumerate(drones):
        start_pose[ns] = (0.0, 0.0, 0.0, 0.0)

    # 1) 지면 근처에서 세트포인트 스트림 (전환 직전에도 계속 세트포인트 필요)
    send_loop(node, 1.0, start_pose, pub_off, pub_sp)

    # 2) ARM
    for ns in drones:
        arm(node, pub_cmd, sys_id_map, ns)

    # 3) OFFBOARD
    for ns in drones:
        set_offboard(node, pub_cmd, sys_id_map, ns)

    # 4) 이륙(정해둔 고도까지 유지)
    takeoff_pose = {ns: (p[0], p[1], takeoff_alt, p[3]) for ns, p in start_pose.items()}
    send_loop(node, 10.0, takeoff_pose, pub_off, pub_sp)

    # 5) 웨이포인트 순회(각 웨이포인트에서 stay_sec 동안 유지)
    for (wx, wy, wz, wyaw) in waypoints:
        pose = {}
        for i, ns in enumerate(drones):
            pose[ns] = (wx, wy, wz, wyaw)
        send_loop(node, stay_sec, pose, pub_off, pub_sp)

    # 6) 종료(착륙/안전처리 없음)
    print("[INFO] Done. Exit now (no landing).")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
