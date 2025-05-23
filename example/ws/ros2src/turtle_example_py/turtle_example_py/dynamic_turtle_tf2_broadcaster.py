#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from turtlesim.msg import Pose


def euler_z_to_quaternion(theta: float) -> np.ndarray:
    """Return a unit quaternion representing yaw-only (2D) rotation."""
    half_theta = theta / 2.0
    return np.array([0.0, 0.0, math.sin(half_theta), math.cos(half_theta)])


class TransformRelay(Node):
    """Publishes a transform from a tracked 2D pose to a static world frame."""

    def __init__(self):
        super().__init__('pose_to_tf_broadcaster')

        self.entity_name = self.declare_parameter(
            'entity_name', 'entity').get_parameter_value().string_value

        self.broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Pose,
            f'/{self.entity_name}/pose',
            self.broadcast_pose_as_tf,
            qos_profile=1
        )

    def broadcast_pose_as_tf(self, pose: Pose):
        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'world'
        transform.child_frame_id = self.entity_name

        transform.transform.translation.x = pose.x
        transform.transform.translation.y = pose.y
        transform.transform.translation.z = 0.0

        quat = euler_z_to_quaternion(pose.theta)
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]

        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = TransformRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
