#!/usr/bin/env python3

import math
import textwrap

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose, SpawnEntity


class DynamicObstacleController(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_obstacle_controller")

        self.declare_parameter("world_name", "warehouse_world")
        self.declare_parameter("obstacle_name", "final_demo_obstacle")
        self.declare_parameter("start_delay_sec", 20.0)
        self.declare_parameter("update_period_sec", 0.2)
        self.declare_parameter("speed_mps", 0.35)
        self.declare_parameter("start_x", -2.0)
        self.declare_parameter("start_y", -0.5)
        self.declare_parameter("end_x", 2.5)
        self.declare_parameter("end_y", -0.5)
        self.declare_parameter("z", 0.45)
        self.declare_parameter("size_x", 0.6)
        self.declare_parameter("size_y", 0.6)
        self.declare_parameter("size_z", 0.9)

        self.world_name = str(self.get_parameter("world_name").value)
        self.obstacle_name = str(self.get_parameter("obstacle_name").value)
        self.start_delay_sec = float(self.get_parameter("start_delay_sec").value)
        self.update_period_sec = float(self.get_parameter("update_period_sec").value)
        self.speed_mps = float(self.get_parameter("speed_mps").value)
        self.start_x = float(self.get_parameter("start_x").value)
        self.start_y = float(self.get_parameter("start_y").value)
        self.end_x = float(self.get_parameter("end_x").value)
        self.end_y = float(self.get_parameter("end_y").value)
        self.z = float(self.get_parameter("z").value)
        self.size_x = float(self.get_parameter("size_x").value)
        self.size_y = float(self.get_parameter("size_y").value)
        self.size_z = float(self.get_parameter("size_z").value)

        self.spawn_client = self.create_client(
            SpawnEntity, f"/world/{self.world_name}/create"
        )
        self.set_pose_client = self.create_client(
            SetEntityPose, f"/world/{self.world_name}/set_pose"
        )

        self.spawned = False
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(self.update_period_sec, self._tick)

    def _sdf(self) -> str:
        mass = 8.0
        return textwrap.dedent(
            f"""
            <sdf version="1.8">
              <model name="{self.obstacle_name}">
                <static>false</static>
                <link name="body">
                  <inertial>
                    <mass>{mass}</mass>
                    <inertia>
                      <ixx>0.5</ixx><iyy>0.5</iyy><izz>0.5</izz>
                      <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
                    </inertia>
                  </inertial>
                  <collision name="collision">
                    <pose>0 0 0 0 0 0</pose>
                    <geometry>
                      <box>
                        <size>{self.size_x} {self.size_y} {self.size_z}</size>
                      </box>
                    </geometry>
                  </collision>
                  <visual name="visual">
                    <pose>0 0 0 0 0 0</pose>
                    <geometry>
                      <box>
                        <size>{self.size_x} {self.size_y} {self.size_z}</size>
                      </box>
                    </geometry>
                    <material>
                      <ambient>0.8 0.1 0.1 1</ambient>
                      <diffuse>0.9 0.15 0.15 1</diffuse>
                    </material>
                  </visual>
                </link>
              </model>
            </sdf>
            """
        ).strip()

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9

    def _spawn(self) -> None:
        if self.spawned:
            return
        if not self.spawn_client.wait_for_service(timeout_sec=0.1):
            return

        req = SpawnEntity.Request()
        req.entity_factory.name = self.obstacle_name
        req.entity_factory.allow_renaming = True
        req.entity_factory.sdf = self._sdf()
        req.entity_factory.pose.position.x = self.start_x
        req.entity_factory.pose.position.y = self.start_y
        req.entity_factory.pose.position.z = self.z
        req.entity_factory.pose.orientation.w = 1.0
        future = self.spawn_client.call_async(req)
        future.add_done_callback(self._handle_spawn_response)

    def _handle_spawn_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to spawn obstacle: {exc}")
            return
        if response.success:
            self.spawned = True
            self.get_logger().info(f"Spawned dynamic obstacle '{self.obstacle_name}'")

    def _set_pose(self, x: float, y: float) -> None:
        if not self.set_pose_client.wait_for_service(timeout_sec=0.1):
            return
        req = SetEntityPose.Request()
        req.entity.name = self.obstacle_name
        req.entity.type = Entity.MODEL
        req.pose.position.x = x
        req.pose.position.y = y
        req.pose.position.z = self.z
        req.pose.orientation.w = 1.0
        self.set_pose_client.call_async(req)

    def _tick(self) -> None:
        elapsed = self._elapsed()
        if not self.spawned:
            self._spawn()
            return
        if elapsed < self.start_delay_sec:
            return

        dx = self.end_x - self.start_x
        dy = self.end_y - self.start_y
        length = max(math.hypot(dx, dy), 1e-6)
        travel_time = length / max(self.speed_mps, 1e-3)
        phase = ((elapsed - self.start_delay_sec) / travel_time) % 2.0
        alpha = phase if phase <= 1.0 else 2.0 - phase
        x = self.start_x + dx * alpha
        y = self.start_y + dy * alpha
        self._set_pose(x, y)


def main() -> None:
    rclpy.init()
    node = DynamicObstacleController()
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
