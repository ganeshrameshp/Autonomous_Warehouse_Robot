#!/usr/bin/env python3

import json
import math
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
from std_msgs.msg import String


class MissionPlanner(Node):
    def __init__(self) -> None:
        super().__init__("mission_planner")

        self.declare_parameter("goals_file", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("start_delay_sec", 5.0)
        self.declare_parameter("max_retries", 2)
        self.declare_parameter("stop_on_failure", False)
        self.declare_parameter("status_topic", "/mission_status")

        self.goals_file = Path(self.get_parameter("goals_file").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.start_delay_sec = float(self.get_parameter("start_delay_sec").value)
        self.max_retries = int(self.get_parameter("max_retries").value)
        self.stop_on_failure = bool(self.get_parameter("stop_on_failure").value)
        status_topic = str(self.get_parameter("status_topic").value)

        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.navigator = BasicNavigator()
        self._started = False
        self._worker = None
        self._timer = self.create_timer(1.0, self._start_once)

    def _start_once(self) -> None:
        if self._started:
            return
        self._started = True
        self._timer.cancel()
        self._worker = threading.Thread(target=self._run_mission, daemon=True)
        self._worker.start()

    def _publish_status(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)
        self.get_logger().info(msg.data)

    def _load_goals(self) -> list[dict]:
        if not self.goals_file.exists():
            raise FileNotFoundError(f"Goals file not found: {self.goals_file}")

        with self.goals_file.open("r", encoding="utf-8") as infile:
            data = json.load(infile)

        goals = data.get("goals", [])
        if not goals:
            raise ValueError(f"No goals found in {self.goals_file}")
        return goals

    def _make_pose(self, goal: dict) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = float(goal["x"])
        pose.pose.position.y = float(goal["y"])
        pose.pose.position.z = 0.0
        yaw = float(goal.get("yaw", 0.0))
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _run_mission(self) -> None:
        try:
            goals = self._load_goals()
        except Exception as exc:
            self._publish_status({"state": "FAILED", "reason": str(exc)})
            return

        self._publish_status({"state": "STARTING", "goal_count": len(goals)})
        time.sleep(self.start_delay_sec)

        self.navigator.waitUntilNav2Active()
        self._publish_status({"state": "ACTIVE"})

        for index, goal in enumerate(goals):
            goal_name = str(goal.get("name", f"goal_{index}"))
            success = False

            for attempt in range(1, self.max_retries + 2):
                self._publish_status(
                    {
                        "state": "RUNNING",
                        "goal_index": index,
                        "goal_name": goal_name,
                        "attempt": attempt,
                    }
                )
                self.navigator.goToPose(self._make_pose(goal))

                while rclpy.ok() and not self.navigator.isTaskComplete():
                    time.sleep(0.2)

                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    success = True
                    self._publish_status(
                        {
                            "state": "SUCCEEDED",
                            "goal_index": index,
                            "goal_name": goal_name,
                            "attempt": attempt,
                        }
                    )
                    break

                self._publish_status(
                    {
                        "state": "RETRYING" if attempt <= self.max_retries else "FAILED",
                        "goal_index": index,
                        "goal_name": goal_name,
                        "attempt": attempt,
                        "result": int(result),
                    }
                )
                time.sleep(1.0)

            if not success and self.stop_on_failure:
                self._publish_status(
                    {
                        "state": "STOPPED",
                        "goal_index": index,
                        "goal_name": goal_name,
                    }
                )
                return

        self._publish_status({"state": "COMPLETED"})


def main() -> None:
    rclpy.init()
    node = MissionPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node.navigator, "destroy_node"):
            node.navigator.destroy_node()
        elif hasattr(node.navigator, "destroyNode"):
            node.navigator.destroyNode()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
