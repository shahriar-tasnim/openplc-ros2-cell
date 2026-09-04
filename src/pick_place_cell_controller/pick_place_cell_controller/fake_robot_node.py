#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class FakeRobotNode(Node):

    def __init__(self):
        super().__init__('fake_robot')

        self.create_subscription(
            Bool,
            '/cell/robot_start',
            self.robot_start_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/cell/robot_reset',
            self.robot_reset_callback,
            10
        )

        self.busy_pub = self.create_publisher(
            Bool,
            '/cell/robot_busy',
            10
        )

        self.done_pub = self.create_publisher(
            Bool,
            '/cell/robot_done',
            10
        )

        self.fault_pub = self.create_publisher(
            Bool,
            '/cell/robot_fault',
            10
        )

        self.home_pub = self.create_publisher(
            Bool,
            '/cell/robot_home',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/cell/robot_status',
            10
        )

        self.state = 'IDLE'

        self.robot_busy = False
        self.robot_done = False
        self.robot_fault = False
        self.robot_home = True

        self.previous_start = False
        self.deadline = 0.0

        self.timer = self.create_timer(0.1, self.update)

        self.publish_status('Fake robot initialized')

    def robot_start_callback(self, msg):
        rising_edge = msg.data and not self.previous_start
        self.previous_start = msg.data

        if not rising_edge:
            return

        if self.robot_fault:
            self.publish_status('Start rejected: robot fault active')
            return

        if self.state != 'IDLE':
            self.publish_status(
                f'Start rejected: robot state is {self.state}'
            )
            return

        self.state = 'BUSY'
        self.robot_busy = True
        self.robot_done = False
        self.robot_home = False

        self.deadline = time.monotonic() + 3.0

        self.publish_status('RobotStart accepted')

    def robot_reset_callback(self, msg):
        if not msg.data:
            return

        self.state = 'IDLE'
        self.robot_busy = False
        self.robot_done = False
        self.robot_fault = False
        self.robot_home = True
        self.deadline = 0.0

        self.publish_status('Robot reset completed')

    def update(self):
        now = time.monotonic()

        if self.state == 'BUSY' and now >= self.deadline:
            self.state = 'DONE'
            self.robot_busy = False
            self.robot_done = True
            self.robot_home = True

            self.deadline = now + 1.0

            self.publish_status('Pick-and-place completed')

        elif self.state == 'DONE' and now >= self.deadline:
            self.state = 'IDLE'
            self.robot_done = False
            self.deadline = 0.0

            self.publish_status('Robot ready for next cycle')

        self.publish_outputs()

    def publish_outputs(self):
        busy_msg = Bool()
        busy_msg.data = self.robot_busy
        self.busy_pub.publish(busy_msg)

        done_msg = Bool()
        done_msg.data = self.robot_done
        self.done_pub.publish(done_msg)

        fault_msg = Bool()
        fault_msg.data = self.robot_fault
        self.fault_pub.publish(fault_msg)

        home_msg = Bool()
        home_msg.data = self.robot_home
        self.home_pub.publish(home_msg)

    def publish_status(self, event):
        text = (
            f'{event} | '
            f'state={self.state} '
            f'home={int(self.robot_home)} '
            f'busy={int(self.robot_busy)} '
            f'done={int(self.robot_done)} '
            f'fault={int(self.robot_fault)}'
        )

        self.get_logger().info(text)

        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = FakeRobotNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
