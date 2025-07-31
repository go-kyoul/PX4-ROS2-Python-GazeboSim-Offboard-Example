#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleStatus,
    VehicleCommand, VehicleCommandAck, TimesyncStatus
)

MICROS = 1_000_000

def now_us(node: Node) -> int:
    return node.get_clock().now().nanoseconds // 1000


# ---------------------------
# 디스커버리: 끝까지 모으고 timesync까지 확인
# ---------------------------
def discover_px4_namespaces(temp_node: Node, timeout_sec: float = 5.0, settle_sec: float = 0.3):
    """
    /px4_#/fmu/out/vehicle_status(_v1) & /timesync_status 를 기준으로
    '완전히 올라온' PX4 인스턴스들을 수집하여 리스트로 반환.
    - timeout_sec 동안 반복 스캔 (중간 발견시 break 금지)
    - 마지막 변화 이후 settle_sec 동안 변화 없으면 조기 종료
    - 네임스페이스 없는 경우 '' 로 표기
    """
    deadline = time.time() + timeout_sec
    status_pat = re.compile(r'^(/px4_\d+)?/fmu/out/vehicle_status(_v1)?$')
    ts_pat     = re.compile(r'^(/px4_\d+)?/fmu/out/timesync_status$')

    status_ns, ts_ns = set(), set()
    last_change = time.time()

    def add_ns(name, pat, bucket: set):
        m = pat.match(name)
        if not m:
            return False
        ns = (m.group(1).lstrip('/') if m.group(1) else '')
        before = len(bucket)
        bucket.add(ns)
        return len(bucket) != before

    while time.time() < deadline:
        changed = False
        for name, types in temp_node.get_topic_names_and_types():
            if any('px4_msgs/msg/VehicleStatus' in t for t in types):
                changed |= add_ns(name, status_pat, status_ns)
            if any('px4_msgs/msg/TimesyncStatus' in t for t in types):
                changed |= add_ns(name, ts_pat, ts_ns)
        if changed:
            last_change = time.time()

        ready = sorted((status_ns & ts_ns), key=lambda s: (s == '', s))  # ''는 뒤로
        if ready and (time.time() - last_change) >= settle_sec:
            return ready

        time.sleep(0.3)

    return sorted((status_ns & ts_ns), key=lambda s: (s == '', s))


# ---------------------------
# 각 인스턴스 제어 노드
# ---------------------------
class OffboardForInstance(Node):
    """
    특정 PX4 인스턴스(ns='px4_1' 등 또는 ''(none))를 오프보드 제어
    - timesync 수신 후 일정 시간 세트포인트 스트리밍
    - OFFBOARD → ARM 자동 시도(ACK 확인/재시도 + 순서 폴백)
    - 원운동(반지름 10m, 고도 5m, 각속도 0.5rad/s)
    """
    def __init__(self, ns: str, radius=10.0, omega=0.5, altitude=5.0, phase=0.0):
        super().__init__(f'offboard_{ns or "px4"}')
        self.ns = ns
        self.ns_prefix = f'/{ns}' if ns else ''
        self.radius = float(radius)
        self.omega  = float(omega)     # rad/s
        self.alt    = float(altitude)  # NED: z = -alt
        self.theta  = float(phase)

        # SYS_ID: ''(none)->1, 'px4_1'->2, 'px4_2'->3 ...
        self.sys_id = self._infer_sys_id_from_ns()
        self.get_logger().info(f"SYS_ID for {ns or '(none)'} = {self.sys_id}")

        # 상태/동기화
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.timesync_ok = False
        self.timesync_last_us = 0
        self.stream_started_us = None
        self.last_offboard_try_us = 0
        self.last_arm_try_us = 0

        # QoS: in(명령)=RELIABLE/VOLATILE, out(상태)=BEST_EFFORT/TRANSIENT_LOCAL
        qos_pub_in = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_sub_out = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 상태 구독: vehicle_status_v1 우선, 없으면 vehicle_status
        topics = dict(self.get_topic_names_and_types())
        cand1 = f'{self.ns_prefix}/fmu/out/vehicle_status_v1'
        cand2 = f'{self.ns_prefix}/fmu/out/vehicle_status'
        status_topic = cand1 if cand1 in topics else cand2

        self.create_subscription(VehicleStatus, status_topic, self._on_status, qos_sub_out)
        self.get_logger().info(f"Subscribing status from: {status_topic}")

        self.create_subscription(TimesyncStatus, f'{self.ns_prefix}/fmu/out/timesync_status',
                                 self._on_timesync, qos_sub_out)

        self.create_subscription(VehicleCommandAck, f'{self.ns_prefix}/fmu/out/vehicle_command_ack',
                                 self._on_cmd_ack, qos_sub_out)

        # 퍼블리셔
        self.pub_offboard = self.create_publisher(
            OffboardControlMode, f'{self.ns_prefix}/fmu/in/offboard_control_mode', qos_pub_in)
        self.pub_traj = self.create_publisher(
            TrajectorySetpoint, f'{self.ns_prefix}/fmu/in/trajectory_setpoint', qos_pub_in)
        self.pub_cmd = self.create_publisher(
            VehicleCommand, f'{self.ns_prefix}/fmu/in/vehicle_command', qos_pub_in)

        # 20 Hz 타이머
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self._loop)

        self.get_logger().info(f"Target namespace: '{self.ns_prefix or '/(none)'}'")

    # ----- Callbacks -----
    def _on_status(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def _on_timesync(self, msg: TimesyncStatus):
        self.timesync_ok = True
        self.timesync_last_us = now_us(self)

    def _on_cmd_ack(self, ack: VehicleCommandAck):
        # 0=ACCEPTED, 2=DENIED, 1=TEMP_REJ, 3=UNSUP, 4=FAILED, ...
        if ack.result == 0:
            if ack.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE:
                self.get_logger().info("ACK: OFFBOARD accepted")
            elif ack.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
                self.get_logger().info("ACK: ARM accepted")
        else:
            self.get_logger().warn(f"ACK: command={ack.command} rejected (result={ack.result})")

    # ----- Helpers -----
    def _infer_sys_id_from_ns(self) -> int:
        # ''(none) -> 1, 'px4_1' -> 2, 'px4_2' -> 3 ...
        try:
            if self.ns and self.ns.startswith('px4_'):
                return int(self.ns.split('_')[1]) + 1
        except Exception:
            pass
        return 1

    def _publish_cmd(self, command: int, p1=0.0, p2=0.0):
        cmd = VehicleCommand()
        cmd.timestamp = now_us(self)
        cmd.command = command
        cmd.param1 = float(p1)
        cmd.param2 = float(p2)
        cmd.target_system = self.sys_id
        cmd.target_component = 1
        cmd.source_system = 200 + self.sys_id  # 구분용 임의값
        cmd.source_component = 1
        cmd.from_external = True
        self.pub_cmd.publish(cmd)

    def _try_offboard(self):
        t = now_us(self)
        if t - self.last_offboard_try_us > 0.5 * MICROS:
            self._publish_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
            self.last_offboard_try_us = t
            self.get_logger().info("Sent: DO_SET_MODE(OFFBOARD)")

    def _try_arm(self):
        t = now_us(self)
        if t - self.last_arm_try_us > 0.5 * MICROS:
            self._publish_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
            self.last_arm_try_us = t
            self.get_logger().info("Sent: ARM")

    # ----- Main loop -----
    def _loop(self):
        t_now = now_us(self)

        # OffboardControlMode + TrajectorySetpoint 스트림 (≥2 Hz)
        oc = OffboardControlMode()
        oc.timestamp = t_now
        oc.position = True
        self.pub_offboard.publish(oc)

        x = self.radius * math.cos(self.theta)
        y = self.radius * math.sin(self.theta)
        z = -self.alt  # NED
        self.theta += self.omega * self.dt

        sp = TrajectorySetpoint()
        sp.timestamp = t_now
        sp.position = [float(x), float(y), float(z)]
        sp.yaw = 0.0
        self.pub_traj.publish(sp)

        if self.stream_started_us is None:
            self.stream_started_us = t_now

        # timesync 최근 수신 + 스트림 3초 이상 유지 → 전환/아밍 시도
        stream_age = (t_now - self.stream_started_us) / MICROS
        timesync_recent = self.timesync_ok and ((t_now - self.timesync_last_us) < 2 * MICROS)

        if timesync_recent and stream_age > 3.0:
            # 기본: OFFBOARD → ARM
            if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self._try_offboard()
            elif self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self._try_arm()



# ---------------------------
# Supervisor: 주기 재스캔 → 신규 인스턴스 자동 등록
# ---------------------------
class Supervisor(Node):
    def __init__(self, exec_):
        super().__init__('offboard_supervisor')
        self.exec = exec_
        self.known = set()
        self.nodes = []  # GC 방지
        # 1초마다 재스캔
        self.timer = self.create_timer(1.0, self._scan)
        self.get_logger().info("Supervisor started. Watching for PX4 instances...")

    def _scan(self):
        # 1초 제한으로 빠르게 스캔, settle 0.3초
        ns_list = discover_px4_namespaces(self, timeout_sec=1.0, settle_sec=0.3)
        # phase를 분산시키기 위해 현재 known 수를 기반으로 위상 부여
        for idx, ns in enumerate(ns_list):
            if ns in self.known:
                continue
            phase = (2.0 * math.pi / max(1, len(ns_list))) * idx
            node = OffboardForInstance(ns=ns, radius=10.0, omega=0.5, altitude=5.0, phase=phase)
            self.nodes.append(node)
            self.exec.add_node(node)
            self.known.add(ns)
            self.get_logger().info(f"New PX4 instance detected & attached: {ns or '(none)'}")


# ---------------------------
# main
# ---------------------------
def main():
    rclpy.init()

    # 스레드 수는 코어 수에 맞춰 여유 있게
    num_threads = max(4, (os.cpu_count() or 4))
    exec_ = MultiThreadedExecutor(num_threads=num_threads)

    supervisor = Supervisor(exec_)
    exec_.add_node(supervisor)

    try:
        exec_.spin()
    finally:
        supervisor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
