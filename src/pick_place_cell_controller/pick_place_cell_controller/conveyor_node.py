#!/usr/bin/env python3
"""
conveyor_node.py -- moves the ACTIVE cube along the UR5e conveyor (in Y)
from the feed end to the pick point, then raises PartAtPick.

UR5e cell coordinates:
    x = 0.45 (constant),  z = 0.44 (belt top + half cube)
    feed  y = -0.85  ->  pick y = -0.20
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity


class ConveyorNode(Node):
    def __init__(self):
        super().__init__('conveyor_node')
        self.part_x = 0.45
        self.feed_y = -0.85
        self.pick_y = -0.20
        self.part_z = 0.44
        self.speed = 0.20
        self.dt = 0.05
        self.current_y = self.feed_y
        self.part_name = 'cube_0'

        self.conveyor_run = False
        self.part_at_pick = False
        self.robot_busy = False
        self.pose_request_pending = False

        self.create_subscription(Bool, '/cell/conveyor_run', self.conveyor_cb, 10)
        self.create_subscription(Bool, '/cell/robot_busy', self.busy_cb, 10)
        self.create_subscription(String, '/cell/active_part', self.active_cb, 10)
        self.part_at_pick_pub = self.create_publisher(Bool, '/cell/part_at_pick', 10)
        self.pose_client = self.create_client(SetEntityPose, '/world/cell_world/set_pose')
        self.create_timer(self.dt, self.update)
        self.get_logger().info(f'Conveyor initialized | feed y={self.feed_y} | pick y={self.pick_y}')

    def active_cb(self, msg):
        if msg.data != self.part_name:
            self.part_name = msg.data
            self.current_y = self.feed_y
            self.part_at_pick = False
            self.get_logger().info(f'New active part {self.part_name} | belt reset')

    def conveyor_cb(self, msg):
        prev = self.conveyor_run
        self.conveyor_run = msg.data
        if self.conveyor_run and not prev:
            self.get_logger().info('ConveyorRun received from PLC')

    def busy_cb(self, msg):
        rising = msg.data and not self.robot_busy
        self.robot_busy = msg.data
        if rising and self.part_at_pick:
            self.part_at_pick = False
            self.get_logger().info('RobotBusy | PartAtPick cleared')

    def update(self):
        self.part_at_pick_pub.publish(Bool(data=self.part_at_pick))
        if not self.conveyor_run or self.part_at_pick or self.robot_busy:
            return
        self.current_y += self.speed * self.dt      # move toward pick (+Y)
        if self.current_y >= self.pick_y:
            self.current_y = self.pick_y
            self.part_at_pick = True
            self.get_logger().info('PART DETECTED at pick | PartAtPick=TRUE')
        self.set_part_pose()

    def set_part_pose(self):
        if self.pose_request_pending or not self.pose_client.service_is_ready():
            return
        req = SetEntityPose.Request()
        req.entity.name = self.part_name
        req.entity.type = Entity.MODEL
        req.pose.position.x = self.part_x
        req.pose.position.y = self.current_y
        req.pose.position.z = self.part_z
        req.pose.orientation.w = 1.0
        self.pose_request_pending = True
        fut = self.pose_client.call_async(req)
        fut.add_done_callback(self.pose_resp)

    def pose_resp(self, future):
        self.pose_request_pending = False
        try:
            r = future.result()
            if not r.success:
                self.get_logger().warn('Gazebo failed to move part')
        except Exception as exc:
            self.get_logger().error(f'SetEntityPose failed: {exc}')


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
