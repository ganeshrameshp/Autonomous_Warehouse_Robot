#!/usr/bin/env python3

import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class GoalPoseCapture(Node):
    def __init__(self) -> None:
        super().__init__("capture_goal_poses")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("input_topic", "/raw_goal_pose")
        self.declare_parameter(
            "output_file",
            "/home/ganesh/robot_ws/src/donar_robot_description/config/final_demo_goals.json",
        )

        input_topic = str(self.get_parameter("input_topic").value)
        self.output_file = Path(str(self.get_parameter("output_file").value))
        self.goals: list[dict] = []

        self.create_subscription(PoseStamped, input_topic, self._handle_goal, 10)
        self.get_logger().info(
            "Listening for goal poses on '%s'. Use RViz 2D Goal Pose to add goals."
            % input_topic
        )

    def _handle_goal(self, msg: PoseStamped) -> None:
        yaw = self._yaw_from_quaternion(msg.pose.orientation.z, msg.pose.orientation.w)
        goal = {
            "name": f"goal_{len(self.goals) + 1}",
            "x": round(float(msg.pose.position.x), 3),
            "y": round(float(msg.pose.position.y), 3),
            "yaw": round(yaw, 3),
        }
        self.goals.append(goal)
        self._write_goals_file()
        self.get_logger().info(
            "Saved %s at x=%.3f y=%.3f yaw=%.3f"
            % (goal["name"], goal["x"], goal["y"], goal["yaw"])
        )

    @staticmethod
    def _yaw_from_quaternion(z: float, w: float) -> float:
        return 2.0 * math.atan2(z, w)

    def _write_goals_file(self) -> None:
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        with self.output_file.open("w", encoding="utf-8") as outfile:
            json.dump({"goals": self.goals}, outfile, indent=2)
            outfile.write("\n")


def main() -> None:
    rclpy.init()
    node = GoalPoseCapture()
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
