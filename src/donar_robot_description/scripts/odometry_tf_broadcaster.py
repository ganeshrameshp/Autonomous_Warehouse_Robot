#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdometryTfBroadcaster(Node):
    def __init__(self) -> None:
        super().__init__("odometry_tf_broadcaster")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("input_topic", "/robot1/odometry")
        self.declare_parameter("odom_frame", "robot1_odom")
        self.declare_parameter("base_frame", "robot1_base_footprint")

        input_topic = self.get_parameter("input_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self._logged_first_message = False

        self.broadcaster = TransformBroadcaster(self)
        odom_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.subscription = self.create_subscription(
            Odometry,
            input_topic,
            self._handle_odometry,
            odom_qos,
        )

        self.get_logger().info(
            f"Broadcasting TF from '{self.odom_frame}' to '{self.base_frame}' "
            f"using odometry topic '{input_topic}' "
            f"(use_sim_time={self.get_parameter('use_sim_time').value})"
        )

    def _handle_odometry(self, msg: Odometry) -> None:
        if not self._logged_first_message:
            self.get_logger().info(
                f"Received first odometry message for '{self.odom_frame}' at "
                f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
            )
            self._logged_first_message = True

        transform = TransformStamped()
        # Stamp with the current ROS clock time. With use_sim_time enabled this
        # is simulation time, which keeps the TF cache current even if bridged
        # odometry messages arrive slightly late or out of order.
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = OdometryTfBroadcaster()
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
