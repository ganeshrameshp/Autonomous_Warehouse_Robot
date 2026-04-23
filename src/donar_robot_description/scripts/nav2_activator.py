#!/usr/bin/env python3

import time

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node


class Nav2Activator(Node):
    def __init__(self) -> None:
        super().__init__("nav2_activator")
        self.declare_parameter(
            "nodes",
            [
                "map_server",
                "amcl",
                "controller_server",
                "planner_server",
                "smoother_server",
                "behavior_server",
                "bt_navigator",
                "waypoint_follower",
                "velocity_smoother",
            ],
        )
        self.declare_parameter("startup_delay_sec", 4.0)
        self.declare_parameter("service_timeout_sec", 5.0)
        self.declare_parameter("max_retries", 5)
        self.declare_parameter("wait_for_all_services_sec", 30.0)

        self.nodes = list(self.get_parameter("nodes").value)
        self.startup_delay_sec = float(self.get_parameter("startup_delay_sec").value)
        self.service_timeout_sec = float(
            self.get_parameter("service_timeout_sec").value
        )
        self.max_retries = int(self.get_parameter("max_retries").value)
        self.wait_for_all_services_sec = float(
            self.get_parameter("wait_for_all_services_sec").value
        )

        self.timer = self.create_timer(self.startup_delay_sec, self._activate_once)
        self._started = False

    def _activate_once(self) -> None:
        if self._started:
            return
        self._started = True
        self.timer.cancel()

        if not self._wait_for_services():
            self.get_logger().error("Timed out waiting for lifecycle services")
            return

        for transition_id, name in (
            (Transition.TRANSITION_CONFIGURE, "configure"),
            (Transition.TRANSITION_ACTIVATE, "activate"),
        ):
            for node_name in self.nodes:
                if not self._transition_with_retries(node_name, transition_id, name):
                    self.get_logger().error(
                        f"Failed to {name} lifecycle node '{node_name}'"
                    )
                    return

        self.get_logger().info("Successfully configured and activated Nav2 nodes")
        self.destroy_node()

    def _wait_for_services(self) -> bool:
        deadline = time.time() + self.wait_for_all_services_sec
        pending = set(self.nodes)
        clients = {
            node_name: self.create_client(ChangeState, f"{node_name}/change_state")
            for node_name in self.nodes
        }

        while pending and time.time() < deadline:
            ready_now = {
                node_name
                for node_name in pending
                if clients[node_name].wait_for_service(timeout_sec=0.5)
            }
            pending -= ready_now
            if pending:
                self.get_logger().info(
                    "Waiting for lifecycle services: " + ", ".join(sorted(pending))
                )

        return not pending

    def _transition_with_retries(
        self, node_name: str, transition_id: int, transition_name: str
    ) -> bool:
        change_client = self.create_client(ChangeState, f"{node_name}/change_state")
        state_client = self.create_client(GetState, f"{node_name}/get_state")
        desired_state = "inactive" if transition_id == Transition.TRANSITION_CONFIGURE else "active"

        for attempt in range(1, self.max_retries + 1):
            current_state = self._get_state_label(state_client)
            if current_state == desired_state:
                self.get_logger().info(
                    f"'{node_name}' already in desired state '{desired_state}'"
                )
                return True

            if not change_client.wait_for_service(timeout_sec=self.service_timeout_sec):
                self.get_logger().warn(
                    f"Waiting for {node_name}/change_state ({attempt}/{self.max_retries})"
                )
                continue

            request = ChangeState.Request()
            request.transition.id = transition_id
            future = change_client.call_async(request)
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=self.service_timeout_sec
            )

            if future.result() is not None and future.result().success:
                self.get_logger().info(f"{transition_name.capitalize()}d '{node_name}'")
                self._log_state(node_name, state_client)
                return True

            current_state = self._get_state_label(state_client)
            if current_state == desired_state:
                self.get_logger().info(
                    f"'{node_name}' reached desired state '{desired_state}' after {transition_name}"
                )
                return True

            self.get_logger().warn(
                f"{transition_name.capitalize()} attempt {attempt} failed for '{node_name}'"
            )
            time.sleep(1.0)

        return False

    def _get_state_label(self, state_client: GetState) -> str | None:
        if not state_client.wait_for_service(timeout_sec=self.service_timeout_sec):
            return None

        future = state_client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.service_timeout_sec)
        if future.result() is not None:
            return future.result().current_state.label
        return None

    def _log_state(self, node_name: str, state_client: GetState) -> None:
        label = self._get_state_label(state_client)
        if label is not None:
            self.get_logger().info(f"{node_name} state: {label}")


def main() -> None:
    rclpy.init()
    node = Nav2Activator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
