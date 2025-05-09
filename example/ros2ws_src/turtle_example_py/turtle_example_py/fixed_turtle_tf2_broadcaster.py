#!/usr/bin/env python3

import sys
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Compute quaternion [x, y, z, w] from roll-pitch-yaw angles."""
    half = np.array([roll, pitch, yaw]) / 2.0
    c = np.cos(half)
    s = np.sin(half)

    cx, cy, cz = c
    sx, sy, sz = s

    qx = sx * cy * cz - cx * sy * sz
    qy = cx * sy * cz + sx * cy * sz
    qz = cx * cy * sz - sx * sy * cz
    qw = cx * cy * cz + sx * sy * sz

    return np.array([qx, qy, qz, qw])


class FixedTransformPublisher(Node):
    """ROS2 node that emits a constant transform once at startup."""

    def __init__(self, frame_id: str, child_id: str, translation: tuple, rotation_rpy: tuple):
        super().__init__('fixed_transform_emitter')
        self.broadcaster = StaticTransformBroadcaster(self)
        self.publish_transform(frame_id, child_id, translation, rotation_rpy)

    def publish_transform(self, parent: str, child: str, pos: tuple, rpy: tuple):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent
        msg.child_frame_id = child

        msg.transform.translation.x = pos[0]
        msg.transform.translation.y = pos[1]
        msg.transform.translation.z = pos[2]

        quat = euler_to_quaternion(*rpy)
        msg.transform.rotation.x = quat[0]
        msg.transform.rotation.y = quat[1]
        msg.transform.rotation.z = quat[2]
        msg.transform.rotation.w = quat[3]

        self.broadcaster.sendTransform(msg)


def parse_args(args):
    if len(args) != 8:
        raise ValueError("Expected 7 arguments: <child_frame> x y z roll pitch yaw")
    if args[1] == "world":
        raise ValueError("Child frame must not be 'world'")
    child = args[1]
    xyz = tuple(map(float, args[2:5]))
    rpy = tuple(map(float, args[5:8]))
    return child, xyz, rpy


def main():
    rclpy.init()
    logger = rclpy.logging.get_logger('transform_setup')

    try:
        child_frame, pos, angles = parse_args(sys.argv)
    except ValueError as e:
        logger.error(str(e))
        sys.exit(1)

    node = FixedTransformPublisher('world', child_frame, pos, angles)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
