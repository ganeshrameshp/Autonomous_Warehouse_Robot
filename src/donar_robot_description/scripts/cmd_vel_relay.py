#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__("cmd_vel_relay")

        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/donar_robot/cmd_vel")

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.subscription = self.create_subscription(
            Twist,
            input_topic,
            self._relay_callback,
            10,
        )

        self.get_logger().info(
            f"Relaying Twist messages from '{input_topic}' to '{output_topic}'"
        )

    def _relay_callback(self, msg: Twist) -> None:
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
