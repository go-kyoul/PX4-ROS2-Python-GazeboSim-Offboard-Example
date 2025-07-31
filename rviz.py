#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy, re, time, math
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

MICROS = 1_000_000

R_ENU_NED = np.array([[0, 1, 0],
                      [1, 0, 0],
                      [0, 0,-1]], dtype=float)   # NED -> ENU
R_FLU_FRD = np.diag([1,-1,-1]).astype(float)     # FRD -> FLU

def q_wxyz_to_R(q):
    # q = [w, x, y, z]
    w, x, y, z = q
    xx, yy, zz = x*x, y*y, z*z
    wx, wy, wz = w*x, w*y, w*z
    xy, xz, yz = x*y, x*z, y*z
    return np.array([
        [1-2*(yy+zz),   2*(xy-wz),     2*(xz+wy)],
        [2*(xy+wz),     1-2*(xx+zz),   2*(yz-wx)],
        [2*(xz-wy),     2*(yz+wx),     1-2*(xx+yy)]
    ], dtype=float)

def R_to_q_wxyz(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t+1.0)*2
        w = 0.25*s
        x = (R[2,1]-R[1,2])/s
        y = (R[0,2]-R[2,0])/s
        z = (R[1,0]-R[0,1])/s
    else:
        i = np.argmax([R[0,0], R[1,1], R[2,2]])
        if i == 0:
            s = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            w = (R[2,1] - R[1,2]) / s
            x = 0.25 * s
            y = (R[0,1] + R[1,0]) / s
            z = (R[0,2] + R[2,0]) / s
        elif i == 1:
            s = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            w = (R[0,2] - R[2,0]) / s
            x = (R[0,1] + R[1,0]) / s
            y = 0.25 * s
            z = (R[1,2] + R[2,1]) / s
        else:
            s = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            w = (R[1,0] - R[0,1]) / s
            x = (R[0,2] + R[2,0]) / s
            y = (R[1,2] + R[2,1]) / s
            z = 0.25 * s
    # normalize
    n = math.sqrt(w*w + x*x + y*y + z*z)
    return (w/n, x/n, y/n, z/n)

def discover_namespaces(node, timeout_sec=10.0):
    deadline = time.time() + timeout_sec
    pat = re.compile(r'^(/px4_\d+)?/fmu/out/vehicle_odometry$')
    ns = set()
    while time.time() < deadline:
        for name, types in node.get_topic_names_and_types():
            if not any('px4_msgs/msg/VehicleOdometry' in t for t in types):
                continue
            m = pat.match(name)
            if m:
                grp = m.group(1)  # '/px4_N' or None
                ns.add(grp.lstrip('/') if grp else '')
        if ns:
            break
        time.sleep(0.3)
    return sorted(ns, key=lambda s: (s == '', s))

class OdomBridge(Node):
    def __init__(self, ns: str):
        super().__init__(f'px4_odom_bridge_{ns or "none"}')
        self.ns = ns
        self.ns_prefix = f'/{ns}' if ns else ''
        self.frame_map = 'map'
        self.child = (f'{ns}/base_link' if ns else 'base_link')

        qos_sub = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        qos_pub = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.VOLATILE,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.sub = self.create_subscription(
            VehicleOdometry, f'{self.ns_prefix}/fmu/out/vehicle_odometry',
            self.cb, qos_sub)

        self.pub = self.create_publisher(
            Odometry, f'{self.ns_prefix}/odom', qos_pub)

        self.tfb = TransformBroadcaster(self)
        self.get_logger().info(f'Bridging {self.ns_prefix or "/(none)"}: '
                               f'vehicle_odometry -> {self.ns_prefix}/odom and TF {self.frame_map}->{self.child}')

    def cb(self, msg: VehicleOdometry):
        # 위치 NED -> ENU
        p_ned = np.array([msg.position[0], msg.position[1], msg.position[2]], dtype=float)
        p_enu = R_ENU_NED @ p_ned

        # 자세 FRD@NED -> FLU@ENU
        q_ned_wxyz = (msg.q[0], msg.q[1], msg.q[2], msg.q[3])  # PX4는 [w,x,y,z]
        R_ned = q_wxyz_to_R(q_ned_wxyz)
        R_enu = R_ENU_NED @ R_ned @ R_FLU_FRD
        q_enu_wxyz = R_to_q_wxyz(R_enu)

        # nav_msgs/Odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.frame_map
        odom.child_frame_id = self.child
        odom.pose.pose.position.x = float(p_enu[0])
        odom.pose.pose.position.y = float(p_enu[1])
        odom.pose.pose.position.z = float(p_enu[2])
        odom.pose.pose.orientation.w = float(q_enu_wxyz[0])
        odom.pose.pose.orientation.x = float(q_enu_wxyz[1])
        odom.pose.pose.orientation.y = float(q_enu_wxyz[2])
        odom.pose.pose.orientation.z = float(q_enu_wxyz[3])

        # (선택) 속도 변환도 가능: v_enu = R_ENU_NED @ v_ned
        self.pub.publish(odom)

        # TF (map -> <ns>/base_link)
        tf = TransformStamped()
        tf.header.stamp = odom.header.stamp
        tf.header.frame_id = self.frame_map
        tf.child_frame_id = self.child
        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.translation.z = odom.pose.pose.position.z
        tf.transform.rotation = odom.pose.pose.orientation
        self.tfb.sendTransform(tf)

def main():
    rclpy.init()
    probe = Node('px4_odom_probe')
    ns_list = discover_namespaces(probe, timeout_sec=10.0)
    probe.destroy_node()
    if not ns_list:
        print('[WARN] vehicle_odometry 토픽을 못 찾음. 기본 (none)로 시도.')
        ns_list = ['']  # /fmu/out/... 케이스

    exec_ = MultiThreadedExecutor()
    nodes = []
    for ns in ns_list:
        n = OdomBridge(ns)
        nodes.append(n)
        exec_.add_node(n)
    try:
        exec_.spin()
    finally:
        for n in nodes:
            n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
