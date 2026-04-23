#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    def __init__(self):
        super().__init__("robot_description_publisher")

        self.declare_parameter("output_topic", "robot_description")
        self.declare_parameter("description_content", "")

        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        description_content = (
            self.get_parameter("description_content").get_parameter_value().string_value
        )

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.publisher = self.create_publisher(String, output_topic, qos)
        self.message = String(data=description_content)

        # Publish immediately and then periodically so late-joining tools like RViz
        # can always retrieve the robot model.
        self.publisher.publish(self.message)
        self.timer = self.create_timer(1.0, self._publish_description)

        self.get_logger().info(
            f"Publishing robot description on '{output_topic}' with transient local QoS"
        )

    def _publish_description(self) -> None:
        self.publisher.publish(self.message)


def main():
    rclpy.init()
    node = RobotDescriptionPublisher()
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
