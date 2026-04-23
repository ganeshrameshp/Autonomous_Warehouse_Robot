#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("initial_pose_publisher")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("topic", "/initialpose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("publish_count", 10)
        self.declare_parameter("startup_delay_sec", 0.0)

        topic = self.get_parameter("topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.yaw = float(self.get_parameter("yaw").value)
        self.publish_count = int(self.get_parameter("publish_count").value)
        self.startup_delay_sec = float(self.get_parameter("startup_delay_sec").value)
        self.start_time = time.monotonic()

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, topic, 10)
        self.sent_count = 0
        self._waiting_for_clock = True
        self.timer = self.create_timer(0.2, self._publish_initial_pose)

    def _publish_initial_pose(self) -> None:
        now = self.get_clock().now()
        if (time.monotonic() - self.start_time) < self.startup_delay_sec:
            return

        if now.nanoseconds == 0:
            if self._waiting_for_clock:
                self.get_logger().info(
                    "Waiting for a valid /clock message before publishing the initial pose"
                )
                self._waiting_for_clock = False
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685

        self.publisher.publish(msg)
        self.sent_count += 1

        if self.sent_count >= self.publish_count:
            self.timer.cancel()
            self.get_logger().info(
                f"Published initial pose to {self.publisher.topic} "
                f"at ({self.x}, {self.y}, yaw={self.yaw})"
            )
            self.destroy_node()


def main() -> None:
    rclpy.init()
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
