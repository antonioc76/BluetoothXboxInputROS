#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from autonav_msgs.msg import ControllerInput


class ControllerListener(Node):

    def __init__(self):
        super().__init__('controller_subscriber')
        self.subscription = self.create_subscription(
            ControllerInput,
            'autonav/controller_input',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f"I heard: {msg.button_name}, {msg.value}")


def main(args=None):
    rclpy.init(args=args)

    controller_listener = ControllerListener()

    rclpy.spin(controller_listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()