import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from std_msgs.msg import Bool, String
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from pick_place_cell_controller import station_poses as sp

# lean 4-move cycle: down to the parcel, lift, straight to the station, back
# to the pick approach ready for the next parcel. No HOME, no PLACE_ABOVE.
MOVE_TIME = {
    'PICK':       1.0,
    'LIFT':       0.8,
    'PLACE_AT':   1.6,
    'PRE_PICK':   1.4,
}


class GazeboRobotNode(Node):

    def __init__(self):
        super().__init__('gazebo_robot')

        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')

        self.create_subscription(Bool, '/cell/robot_start',
                                 self.robot_start_callback, 10)
        self.create_subscription(Bool, '/cell/robot_reset',
                                 self.robot_reset_callback, 10)
        self.create_subscription(String, '/cell/destination',
                                 self.destination_callback, 10)

        self.home_pub = self.create_publisher(Bool, '/cell/robot_home', 10)
        self.busy_pub = self.create_publisher(Bool, '/cell/robot_busy', 10)
        self.done_pub = self.create_publisher(Bool, '/cell/robot_done', 10)
        self.fault_pub = self.create_publisher(Bool, '/cell/robot_fault', 10)
        self.status_pub = self.create_publisher(String, '/cell/robot_status', 10)

        self.robot_home = True
        self.robot_busy = False
        self.robot_done = False
        self.robot_fault = False
        self.previous_robot_start = False
        self.sequence_running = False
        self.sequence_index = 0
        self.done_until = 0.0

        self.destination = 'REJECT'      # latest routing from the sorter
        self.cycle_dest = 'REJECT'       # frozen for the running cycle
        self.sequence = []

        self.joint_names = sp.JOINT_NAMES

        self.create_timer(0.1, self.publish_feedback)
        self.publish_status('IDLE')
        self.get_logger().info('UR5e robot ready | 4-move sorting cycle')

    def destination_callback(self, msg):
        if msg.data in sp.PLACE:
            self.destination = msg.data

    def build_sequence(self):
        """PICK -> LIFT -> PLACE_AT(station) -> PRE_PICK (ready for next)."""
        place = sp.PLACE.get(self.cycle_dest, sp.PLACE['REJECT'])
        return [
            ('PICK',     sp.PICK,     MOVE_TIME['PICK']),
            ('LIFT',     sp.LIFT,     MOVE_TIME['LIFT']),
            ('PLACE_AT', place,       MOVE_TIME['PLACE_AT']),
            ('PRE_PICK', sp.PRE_PICK, MOVE_TIME['PRE_PICK']),
        ]

    def robot_start_callback(self, msg):
        rising_edge = msg.data and not self.previous_robot_start
        self.previous_robot_start = msg.data
        if not rising_edge:
            return
        if self.sequence_running:
            self.get_logger().warn('RobotStart ignored: robot already busy')
            return
        if self.robot_fault:
            self.get_logger().warn('RobotStart ignored: robot is faulted')
            return
        if not self.trajectory_client.wait_for_server(timeout_sec=2.0):
            self.set_fault('joint_trajectory_controller unavailable')
            return

        self.cycle_dest = self.destination          # latch for this parcel
        self.sequence = self.build_sequence()
        self.sequence_running = True
        self.sequence_index = 0
        self.robot_busy = True
        self.robot_home = False
        self.robot_done = False
        self.publish_status('BUSY')
        self.get_logger().info(f'Cycle start -> station {self.cycle_dest}')
        self.send_current_pose()

    def send_current_pose(self):
        if self.sequence_index >= len(self.sequence):
            self.complete_sequence()
            return
        pose_name, positions, duration = self.sequence[self.sequence_index]
        self.publish_status(pose_name)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(seconds=duration).to_msg()
        goal.trajectory.points = [point]
        fut = self.trajectory_client.send_goal_async(goal)
        fut.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.set_fault(f'Failed to send trajectory: {exc}')
            return
        if not goal_handle.accepted:
            self.set_fault('Trajectory goal rejected')
            return
        goal_handle.get_result_async().add_done_callback(
            self.trajectory_result_callback)

    def trajectory_result_callback(self, future):
        try:
            result = future.result().result
        except Exception as exc:
            self.set_fault(f'Trajectory execution failed: {exc}')
            return
        if result.error_code != 0:
            self.set_fault(f'Controller error {result.error_code}: '
                           f'{result.error_string}')
            return
        self.sequence_index += 1
        self.send_current_pose()

    def complete_sequence(self):
        self.sequence_running = False
        self.robot_busy = False
        self.robot_home = True
        self.robot_done = True
        self.done_until = time.monotonic() + 0.4
        self.publish_status('DONE')
        self.get_logger().info(f'Cycle complete ({self.cycle_dest})')

    def robot_reset_callback(self, msg):
        if not msg.data or self.sequence_running:
            return
        self.robot_fault = False
        self.robot_done = False
        self.robot_home = True
        self.publish_status('IDLE')
        self.get_logger().info('Robot fault reset')

    def set_fault(self, reason):
        self.sequence_running = False
        self.robot_busy = False
        self.robot_done = False
        self.robot_home = False
        self.robot_fault = True
        self.publish_status('FAULT')
        self.get_logger().error(f'Robot FAULT: {reason}')

    def publish_feedback(self):
        if self.robot_done and time.monotonic() >= self.done_until:
            self.robot_done = False
            if not self.robot_fault:
                self.publish_status('IDLE')
        self.home_pub.publish(Bool(data=self.robot_home))
        self.busy_pub.publish(Bool(data=self.robot_busy))
        self.done_pub.publish(Bool(data=self.robot_done))
        self.fault_pub.publish(Bool(data=self.robot_fault))

    def publish_status(self, status):
        self.status_pub.publish(String(data=status))


def main(args=None):
    rclpy.init(args=args)
    node = GazeboRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
