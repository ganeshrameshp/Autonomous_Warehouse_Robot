#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanRelay(Node):
    def __init__(self):
        super().__init__("scan_relay")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("input_topic", "/donar_robot/lidar")
        self.declare_parameter("output_topic", "/donar_robot/scan")
        self.declare_parameter("output_frame", "lidar_link")

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.output_frame = (
            self.get_parameter("output_frame").get_parameter_value().string_value
        )

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.publisher = self.create_publisher(LaserScan, output_topic, sensor_qos)
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self._relay_callback,
            sensor_qos,
        )

        self.get_logger().info(
            f"Relaying LaserScan messages from '{input_topic}' to '{output_topic}' "
            f"with frame '{self.output_frame}'"
        )

    def _relay_callback(self, msg: LaserScan) -> None:
        # Normalize the frame_id so downstream SLAM/Nav2 nodes can resolve it
        # against the TF tree published from the robot description. Refresh the
        # timestamp to the current simulation time so AMCL doesn't receive scans
        # that are already older than the available TF cache by the time they
        # work through the bridge and processing queues.
        msg.header.frame_id = self.output_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = ScanRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
