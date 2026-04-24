#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node


class GoalPoseRelay(Node):
    def __init__(self) -> None:
        super().__init__("goal_pose_relay")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("input_topic", "/raw_goal_pose")
        self.declare_parameter("output_topic", "/goal_pose")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("use_current_orientation", True)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        amcl_pose_topic = str(self.get_parameter("amcl_pose_topic").value)
        self.use_current_orientation = bool(
            self.get_parameter("use_current_orientation").value
        )

        self._last_orientation = None
        self._warned_no_pose = False

        self._goal_pub = self.create_publisher(PoseStamped, output_topic, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            amcl_pose_topic,
            self._handle_amcl_pose,
            10,
        )
        self.create_subscription(PoseStamped, input_topic, self._handle_goal, 10)

        self.get_logger().info(
            f"Relaying goals from '{input_topic}' to '{output_topic}'"
        )

    def _handle_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._last_orientation = msg.pose.pose.orientation

    def _handle_goal(self, msg: PoseStamped) -> None:
        relayed = PoseStamped()
        relayed.header = msg.header
        relayed.pose = msg.pose

        if self.use_current_orientation and self._last_orientation is not None:
            relayed.pose.orientation = self._last_orientation
        elif self.use_current_orientation and not self._warned_no_pose:
            self.get_logger().warn(
                "No AMCL pose received yet; forwarding goal orientation unchanged."
            )
            self._warned_no_pose = True

        self._goal_pub.publish(relayed)


def main() -> None:
    rclpy.init()
    node = GoalPoseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
