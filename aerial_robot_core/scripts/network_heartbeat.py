#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty


class NetworkHeartbeat(Node):
    """Publish a GCS heartbeat which traverses the vehicle network link."""

    def __init__(self):
        super().__init__("network_heartbeat")
        self.declare_parameter("rate_hz", 2.0)
        rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value
        if rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive")

        # Keep this reliable so it can traverse the generic namespace bridge and
        # reach the default-reliability subscriptions on both simulation and FC.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(Empty, "network/heartbeat", qos)
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_heartbeat)

    def publish_heartbeat(self):
        self.publisher.publish(Empty())


def main(args=None):
    rclpy.init(args=args)
    node = NetworkHeartbeat()
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
