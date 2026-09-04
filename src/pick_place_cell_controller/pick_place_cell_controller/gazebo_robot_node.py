import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from std_msgs.msg import Bool, String
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from pick_place_cell_controller import grid_place

# Fixed poses (IK pre-solved, radians, joint1..joint6)
PRE_PICK = [-0.6327, -0.9544, 1.1143, 0.0, 1.4108, 0.0]
PICK     = [-0.6327, -0.8321, 1.2767, 0.0, 1.1262, 0.0]
LIFT     = [-0.6327, -0.9544, 1.1143, 0.0, 1.4108, 0.0]
HOME     = [ 0.0,    -1.0472, 1.5708, 0.0, 1.0472, 0.0]


class GazeboRobotNode(Node):

    def __init__(self):
        super().__init__('gazebo_robot')

        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')

        self.create_subscription(Bool, '/cell/robot_start',
                                 self.robot_start_callback, 10)
        self.create_subscription(Bool, '/cell/robot_reset',
                                 self.robot_reset_callback, 10)
        self.create_subscription(String, '/cell/active_part',
                                 self.active_part_callback, 10)

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

        self.active_index = 0          # which grid cell this cube fills
        self.sequence = []             # built per cycle

        self.joint_names = ['joint1', 'joint2', 'joint3',
                            'joint4', 'joint5', 'joint6']

        self.create_timer(0.1, self.publish_feedback)
        self.publish_status('IDLE')
        self.get_logger().info(
            'Gazebo robot initialized | state=IDLE home=1 busy=0 done=0 fault=0')

    # cube_<n> -> place cell n
    def active_part_callback(self, msg):
        try:
            self.active_index = int(msg.data.split('_')[-1]) % grid_place.NUM_CELLS
        except (ValueError, IndexError):
            self.active_index = 0

    def build_sequence(self, cell_index):
        seq = [('PRE_PICK', PRE_PICK, 1.5),
               ('PICK',     PICK,     1.5),
               ('LIFT',     LIFT,     1.5)]
        seq += grid_place.place_steps(cell_index)     # ABOVE, AT, LIFT
        seq += [('HOME', HOME, 2.0)]
        return seq

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
        self.get_logger().info('RobotStart received from PLC')
        if not self.trajectory_client.wait_for_server(timeout_sec=2.0):
            self.set_fault('arm_controller action server unavailable')
            return

        # Latch the target cell for THIS cube and build its motion plan.
        self.sequence = self.build_sequence(self.active_index)
        self.sequence_running = True
        self.sequence_index = 0
        self.robot_busy = True
        self.robot_home = False
        self.robot_done = False
        self.publish_status('BUSY')
        self.get_logger().info(
            f'Starting cycle -> grid cell index {self.active_index}')
        self.send_current_pose()

    def send_current_pose(self):
        if self.sequence_index >= len(self.sequence):
            self.complete_sequence()
            return
        pose_name, positions, duration = self.sequence[self.sequence_index]
        self.publish_status(pose_name)
        self.get_logger().info(f'Moving to {pose_name}')
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
            self.set_fault(f'Trajectory controller error {result.error_code}: '
                           f'{result.error_string}')
            return
        self.get_logger().info(f'{self.sequence[self.sequence_index][0]} reached')
        self.sequence_index += 1
        self.send_current_pose()

    def complete_sequence(self):
        self.sequence_running = False
        self.robot_busy = False
        self.robot_home = True
        self.robot_done = True
        self.done_until = time.monotonic() + 1.0
        self.publish_status('DONE')
        self.get_logger().info(
            'Pick-and-place completed | state=DONE home=1 busy=0 done=1 fault=0')

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
