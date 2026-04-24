#!/usr/bin/env python3

import json
import math
import threading
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class MissionPlanner(Node):
    def __init__(self) -> None:
        super().__init__("mission_planner")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("goals_file", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("start_delay_sec", 5.0)
        self.declare_parameter("max_retries", 2)
        self.declare_parameter("stop_on_failure", False)
        self.declare_parameter("status_topic", "/mission_status")
        self.declare_parameter("wait_for_initial_pose", True)
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("goal_markers_topic", "/mission_goals")
        self.declare_parameter("align_yaw_to_path", True)

        self.use_sim_time = bool(self.get_parameter("use_sim_time").value)
        self.goals_file = Path(self.get_parameter("goals_file").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.start_delay_sec = float(self.get_parameter("start_delay_sec").value)
        self.max_retries = int(self.get_parameter("max_retries").value)
        self.stop_on_failure = bool(self.get_parameter("stop_on_failure").value)
        status_topic = str(self.get_parameter("status_topic").value)
        self.wait_for_initial_pose = bool(
            self.get_parameter("wait_for_initial_pose").value
        )
        initial_pose_topic = str(self.get_parameter("initial_pose_topic").value)
        amcl_pose_topic = str(self.get_parameter("amcl_pose_topic").value)
        goal_markers_topic = str(self.get_parameter("goal_markers_topic").value)
        self.align_yaw_to_path = bool(
            self.get_parameter("align_yaw_to_path").value
        )

        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(String, status_topic, status_qos)
        self.goal_markers_pub = self.create_publisher(
            MarkerArray, goal_markers_topic, marker_qos
        )
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._initial_pose_received = not self.wait_for_initial_pose
        self._goals = []
        self._last_status = None
        self._clock_wait_reported = set()
        self._started = False
        self._worker = None
        self._timer = self.create_timer(1.0, self._start_once)
        self._initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            initial_pose_topic,
            self._handle_initial_pose,
            10,
        )
        self._amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            amcl_pose_topic,
            self._handle_amcl_pose,
            10,
        )
        self._goal_markers_timer = self.create_timer(1.0, self._republish_goal_markers)
        self._status_timer = self.create_timer(1.0, self._republish_status)

    def _start_once(self) -> None:
        if self._started:
            return
        self._started = True
        self._timer.cancel()
        self._worker = threading.Thread(target=self._run_mission, daemon=True)
        self._worker.start()

    def _publish_status(self, payload: dict) -> None:
        self._last_status = payload
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)
        self.get_logger().info(msg.data)

    def _republish_status(self) -> None:
        if self._last_status is None:
            return
        msg = String()
        msg.data = json.dumps(self._last_status)
        self.status_pub.publish(msg)

    def _handle_initial_pose(self, _msg: PoseWithCovarianceStamped) -> None:
        if self._initial_pose_received:
            return
        self._initial_pose_received = True
        self._publish_status({"state": "INITIAL_POSE_RECEIVED"})

    def _handle_amcl_pose(self, _msg: PoseWithCovarianceStamped) -> None:
        if self._initial_pose_received:
            return
        self._initial_pose_received = True
        self._publish_status({"state": "LOCALIZED"})

    def _load_goals(self) -> list[dict]:
        if not self.goals_file.exists():
            raise FileNotFoundError(f"Goals file not found: {self.goals_file}")

        with self.goals_file.open("r", encoding="utf-8") as infile:
            data = json.load(infile)

        goals = data.get("goals", [])
        if not goals:
            raise ValueError(f"No goals found in {self.goals_file}")
        return goals

    def _goal_yaw(self, goal: dict, index: int) -> float:
        if not self.align_yaw_to_path or index >= len(self._goals) - 1:
            return float(goal.get("yaw", 0.0))

        next_goal = self._goals[index + 1]
        dx = float(next_goal["x"]) - float(goal["x"])
        dy = float(next_goal["y"]) - float(goal["y"])
        if abs(dx) < 1.0e-6 and abs(dy) < 1.0e-6:
            return float(goal.get("yaw", 0.0))
        return math.atan2(dy, dx)

    def _make_pose(self, goal: dict, index: int) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(goal["x"])
        pose.pose.position.y = float(goal["y"])
        pose.pose.position.z = 0.0
        yaw = self._goal_yaw(goal, index)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _publish_goal_markers(self, goals: list[dict]) -> None:
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        path_marker = Marker()
        path_marker.header.frame_id = self.frame_id
        path_marker.header.stamp = stamp
        path_marker.ns = "mission_goal_path"
        path_marker.id = 2000
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.pose.orientation.w = 1.0
        path_marker.scale.x = 0.12
        path_marker.color.a = 0.95
        path_marker.color.r = 1.0
        path_marker.color.g = 0.1
        path_marker.color.b = 0.1

        for index, goal in enumerate(goals):
            sphere = Marker()
            sphere.header.frame_id = self.frame_id
            sphere.header.stamp = stamp
            sphere.ns = "mission_goals"
            sphere.id = index
            sphere.type = Marker.CYLINDER
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(goal["x"])
            sphere.pose.position.y = float(goal["y"])
            sphere.pose.position.z = 0.35
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.7
            sphere.scale.y = 0.7
            sphere.scale.z = 0.7
            sphere.color.a = 0.9
            sphere.color.r = 0.0
            sphere.color.g = 1.0
            sphere.color.b = 0.0
            markers.markers.append(sphere)

            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = stamp
            label.ns = "mission_goal_labels"
            label.id = 1000 + index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(goal["x"])
            label.pose.position.y = float(goal["y"])
            label.pose.position.z = 1.1
            label.pose.orientation.w = 1.0
            label.scale.z = 0.55
            label.color.a = 1.0
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.text = str(goal.get("name", f"G{index + 1}"))
            markers.markers.append(label)

            point = Point()
            point.x = float(goal["x"])
            point.y = float(goal["y"])
            point.z = 0.12
            path_marker.points.append(point)

        if len(path_marker.points) >= 2:
            markers.markers.append(path_marker)

        self.goal_markers_pub.publish(markers)

    def _republish_goal_markers(self) -> None:
        if self._goals:
            self._publish_goal_markers(self._goals)

    def _result_to_status_value(self, result: object) -> str:
        if hasattr(result, "name"):
            return str(result.name)
        return str(result)

    def _goal_status_to_text(self, status: int) -> str:
        status_map = {
            GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
            GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
            GoalStatus.STATUS_EXECUTING: "EXECUTING",
            GoalStatus.STATUS_CANCELING: "CANCELING",
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }
        return status_map.get(status, str(status))

    def _wait_for_nav2_active(self) -> None:
        while rclpy.ok():
            if self._nav_client.wait_for_server(timeout_sec=1.0):
                return
            self._publish_status({"state": "WAITING_FOR_NAV2_ACTIVE"})

        raise RuntimeError("NavigateToPose action server not available")

    def _send_goal_and_wait(self, pose: PoseStamped) -> tuple[bool, str]:
        goal = NavigateToPose.Goal()
        goal.pose = pose

        goal_response_event = threading.Event()
        result_event = threading.Event()
        state: dict[str, object] = {"goal_handle": None, "accepted": False, "status": None}

        def _goal_response_cb(future):
            try:
                goal_handle = future.result()
                state["goal_handle"] = goal_handle
                state["accepted"] = bool(goal_handle and goal_handle.accepted)
                if state["accepted"]:
                    result_future = goal_handle.get_result_async()
                    result_future.add_done_callback(_result_cb)
            finally:
                goal_response_event.set()

        def _result_cb(future):
            try:
                result = future.result()
                state["status"] = result.status
            finally:
                result_event.set()

        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(_goal_response_cb)

        while rclpy.ok() and not goal_response_event.is_set():
            time.sleep(0.1)

        if not state["accepted"]:
            return False, "GOAL_REJECTED"

        while rclpy.ok() and not result_event.is_set():
            time.sleep(0.2)

        status = int(state["status"]) if state["status"] is not None else GoalStatus.STATUS_UNKNOWN
        return status == GoalStatus.STATUS_SUCCEEDED, self._goal_status_to_text(status)

    def _wait_for_clock(self, node: Node, label: str) -> None:
        if not self.use_sim_time:
            return

        while rclpy.ok() and node.get_clock().now().nanoseconds == 0:
            if label not in self._clock_wait_reported:
                self._publish_status({"state": "WAITING_FOR_CLOCK", "node": label})
                self._clock_wait_reported.add(label)
            time.sleep(0.1)

    def _run_mission(self) -> None:
        try:
            self._wait_for_clock(self, "mission_planner")
            goals = self._load_goals()
            self._goals = goals
            self._publish_goal_markers(goals)
            self._publish_status({"state": "STARTING", "goal_count": len(goals)})
            if self.wait_for_initial_pose:
                self._publish_status({"state": "WAITING_FOR_INITIAL_POSE"})
                while rclpy.ok() and not self._initial_pose_received:
                    time.sleep(0.2)

            self._publish_status(
                {"state": "START_DELAY", "seconds": self.start_delay_sec}
            )
            time.sleep(self.start_delay_sec)

            self._wait_for_nav2_active()
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
                            "target_yaw": round(self._goal_yaw(goal, index), 3),
                        }
                    )
                    success, result_text = self._send_goal_and_wait(
                        self._make_pose(goal, index)
                    )
                    if success:
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
                            "state": "RETRYING"
                            if attempt <= self.max_retries
                            else "FAILED",
                            "goal_index": index,
                            "goal_name": goal_name,
                            "attempt": attempt,
                            "result": result_text,
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
        except Exception as exc:
            self._publish_status({"state": "FAILED", "reason": str(exc)})


def main() -> None:
    rclpy.init()
    node = MissionPlanner()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
