#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity


class ConveyorNode(Node):

    def __init__(self):
        super().__init__('conveyor_node')

        # ============================================================
        # Conveyor geometry
        # ============================================================

        self.start_x = 1.35
        self.pick_x = 0.75

        self.part_y = -0.55
        self.part_z = 0.31

        # m/s
        self.speed = 0.20

        # Timer = 50 ms
        self.dt = 0.05

        self.current_x = self.start_x

        # ============================================================
        # State
        # ============================================================

        self.conveyor_run = False
        self.part_at_pick = False
        self.robot_busy = False

        self.pose_request_pending = False

        # ============================================================
        # PLC / ROS communication
        # ============================================================

        self.create_subscription(
            Bool,
            '/cell/conveyor_run',
            self.conveyor_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/cell/robot_busy',
            self.robot_busy_callback,
            10
        )

        self.part_at_pick_pub = self.create_publisher(
            Bool,
            '/cell/part_at_pick',
            10
        )

        # ============================================================
        # Gazebo SetEntityPose service
        # ============================================================

        self.pose_client = self.create_client(
            SetEntityPose,
            '/world/cell_world/set_pose'
        )

        self.create_timer(
            self.dt,
            self.update
        )

        self.get_logger().info(
            'Conveyor initialized | '
            'part x=1.35 | pick x=0.75'
        )

    # ================================================================
    # Conveyor command from PLC
    # ================================================================

    def conveyor_callback(self, msg):

        previous = self.conveyor_run

        self.conveyor_run = msg.data

        if self.conveyor_run and not previous:

            self.get_logger().info(
                'ConveyorRun received from PLC'
            )

        elif previous and not self.conveyor_run:

            self.get_logger().info(
                'Conveyor stopped'
            )

    # ================================================================
    # Robot takes responsibility for the part
    # ================================================================

    def robot_busy_callback(self, msg):

        rising_edge = (
            msg.data and
            not self.robot_busy
        )

        self.robot_busy = msg.data

        if rising_edge and self.part_at_pick:

            # Once robot starts handling the part,
            # remove the pick sensor signal.
            self.part_at_pick = False

            self.get_logger().info(
                'RobotBusy received | '
                'PartAtPick cleared'
            )

    # ================================================================
    # Main conveyor update
    # ================================================================

    def update(self):

        # Always publish sensor state so that the
        # Modbus bridge has a persistent current value.
        sensor_msg = Bool()

        sensor_msg.data = self.part_at_pick

        self.part_at_pick_pub.publish(
            sensor_msg
        )

        # Don't move unless PLC commands the conveyor.
        if not self.conveyor_run:

            return

        # Stop once the part reaches the sensor.
        if self.part_at_pick:

            return

        # Don't move once robot is working.
        if self.robot_busy:

            return

        # Move toward robot.
        step = self.speed * self.dt

        self.current_x -= step

        if self.current_x <= self.pick_x:

            self.current_x = self.pick_x

            self.part_at_pick = True

            self.get_logger().info(
                'PART DETECTED at pick station | '
                'PartAtPick=TRUE'
            )

        self.set_part_pose()

    # ================================================================
    # Move cube inside Gazebo
    # ================================================================

    def set_part_pose(self):

        if self.pose_request_pending:

            return

        if not self.pose_client.service_is_ready():

            return

        request = SetEntityPose.Request()

        request.entity.name = 'part'
        request.entity.type = Entity.MODEL

        request.pose.position.x = self.current_x
        request.pose.position.y = self.part_y
        request.pose.position.z = self.part_z

        request.pose.orientation.x = 0.0
        request.pose.orientation.y = 0.0
        request.pose.orientation.z = 0.0
        request.pose.orientation.w = 1.0

        self.pose_request_pending = True

        future = self.pose_client.call_async(
            request
        )

        future.add_done_callback(
            self.pose_response
        )

    # ================================================================
    # Gazebo service response
    # ================================================================

    def pose_response(self, future):

        self.pose_request_pending = False

        try:

            result = future.result()

            if not result.success:

                self.get_logger().warn(
                    'Gazebo failed to move part'
                )

        except Exception as exc:

            self.get_logger().error(
                f'SetEntityPose failed: {exc}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = ConveyorNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
