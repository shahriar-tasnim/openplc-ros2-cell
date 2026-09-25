import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from pick_place_cell_controller import station_poses as sp

# Adaptive joint-space timing.  This project uses pi rad/s as a conservative
# planning-velocity baseline for each arm joint.  The global speed scale and
# a time margin keep normal operation below that baseline while allowing all
# four destinations to receive travel time proportional to their distance.
PLANNING_JOINT_VELOCITY = [math.pi] * 6
DEFAULT_SPEED_SCALE = 0.60
DEFAULT_MIN_MOVE_TIME = 0.45
DEFAULT_TRAJECTORY_MARGIN = 1.15


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
        # Keep the legacy destination subscription for monitoring/fallback,
        # but a cycle is armed only by the combined sort_decision event.
        self.create_subscription(String, '/cell/destination',
                                 self.destination_callback, 10)
        self.create_subscription(String, '/cell/sort_decision',
                                 self.sort_decision_callback, 10)
        self.create_subscription(JointState, '/joint_states',
                                 self.joint_state_callback, 10)

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

        self.destination = 'REJECT'
        self.cycle_dest = 'REJECT'
        self.sequence = []

        # PLC/sorter handshake state.  OpenPLC holds RobotStart until Busy is
        # observed.  If RobotStart arrives a little before vision completes,
        # remember it and start as soon as the current sort_decision arrives.
        self.pending_start = False
        self.route_ready = False
        self.route_label = None

        self.joint_names = sp.JOINT_NAMES
        self.current_joints = None

        # One live speed control for the complete cell.  A change made with
        # `ros2 param set /gazebo_robot speed_scale ...` is used when the next
        # parcel cycle is planned; an already-running cycle is not retimed.
        self.declare_parameter('speed_scale', DEFAULT_SPEED_SCALE)
        self.declare_parameter('min_move_time', DEFAULT_MIN_MOVE_TIME)
        self.declare_parameter('trajectory_margin',
                               DEFAULT_TRAJECTORY_MARGIN)

        self.create_timer(0.1, self.publish_feedback)
        self.publish_status('IDLE')
        self.get_logger().info(
            f'UR5e robot ready | adaptive 4-move cycle | '
            f'speed_scale={self.get_speed_scale():.2f} | '
            'waiting for sort decision')

    def destination_callback(self, msg):
        if msg.data in sp.PLACE:
            self.destination = msg.data

    def joint_state_callback(self, msg):
        """Cache the actual six arm joints for the next motion plan."""
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in self.joint_names):
            self.current_joints = [positions[name] for name in self.joint_names]

    def sort_decision_callback(self, msg):
        label = None
        dest = None
        for tok in msg.data.split():
            if tok.startswith('label='):
                try:
                    label = int(tok.split('=', 1)[1])
                except ValueError:
                    return
            elif tok.startswith('dest='):
                dest = tok.split('=', 1)[1]

        if dest not in sp.PLACE:
            return

        # Do not let a late camera update alter an already-running cycle.
        if self.sequence_running:
            return

        self.destination = dest
        self.route_label = label
        self.route_ready = True
        self.get_logger().info(
            f'Sort decision ready: label={label} -> {dest}')
        self.try_start_cycle()

    def get_speed_scale(self):
        """Return the requested global speed scale, clamped to 10-100%."""
        requested = float(self.get_parameter('speed_scale').value)
        return max(0.10, min(1.00, requested))

    def compute_move_time(self, start, target):
        """Compute a safe duration from joint displacement and velocity.

        The slowest-required joint determines the segment time.  This keeps
        short moves short while automatically giving farther stations more
        time.  A small time margin and minimum duration reduce abrupt motion.
        """
        speed_scale = self.get_speed_scale()
        margin = max(1.0, float(
            self.get_parameter('trajectory_margin').value))
        min_time = max(0.20, float(
            self.get_parameter('min_move_time').value))

        required = max(
            abs(goal - current) / (vmax * speed_scale)
            for current, goal, vmax in zip(
                start, target, PLANNING_JOINT_VELOCITY)
        )
        return max(min_time, required * margin)

    def build_sequence(self):
        place = sp.PLACE.get(self.cycle_dest, sp.PLACE['REJECT'])
        targets = [
            ('PICK', sp.PICK),
            ('LIFT', sp.LIFT),
            ('PLACE_AT', place),
            ('PRE_PICK', sp.PRE_PICK),
        ]

        # Use the measured robot state for the first segment whenever it is
        # available.  Subsequent segments use the preceding target pose.  This
        # also makes the very first cycle safe if Gazebo did not start exactly
        # at PRE_PICK.
        sequence = []
        previous = (list(self.current_joints)
                    if self.current_joints is not None
                    else list(sp.PRE_PICK))
        for pose_name, positions in targets:
            duration = self.compute_move_time(previous, positions)
            sequence.append((pose_name, positions, duration))
            previous = positions

        timing = ', '.join(
            f'{name}={duration:.2f}s'
            for name, _positions, duration in sequence
        )
        self.get_logger().info(
            f'Motion plan | speed_scale={self.get_speed_scale():.2f} | '
            f'{timing}')
        return sequence

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

        self.pending_start = True
        if not self.route_ready:
            self.get_logger().info(
                'RobotStart received; waiting for current ArUco sort decision')
        self.try_start_cycle()

    def try_start_cycle(self):
        if not self.pending_start or not self.route_ready:
            return
        if self.sequence_running or self.robot_fault:
            return
        if not self.trajectory_client.wait_for_server(timeout_sec=2.0):
            self.pending_start = False
            self.set_fault('joint_trajectory_controller unavailable')
            return

        self.pending_start = False
        self.cycle_dest = self.destination
        self.sequence = self.build_sequence()
        self.sequence_running = True
        self.sequence_index = 0
        self.robot_busy = True
        self.robot_home = False
        self.robot_done = False
        self.publish_status('BUSY')
        self.get_logger().info(
            f'Cycle start | label={self.route_label} -> station {self.cycle_dest}')
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
        self.done_until = time.monotonic() + 1.0

        # Consume the decision.  The next PLC RobotStart cannot use this
        # parcel's route; a fresh ArUco decision is required.
        completed_dest = self.cycle_dest
        completed_label = self.route_label
        self.route_ready = False
        self.route_label = None
        self.pending_start = False

        self.publish_status('DONE')
        self.get_logger().info(
            f'Cycle complete | label={completed_label} -> {completed_dest}')

    def robot_reset_callback(self, msg):
        if not msg.data or self.sequence_running:
            return
        self.robot_fault = False
        self.robot_done = False
        self.robot_home = True
        self.pending_start = False
        # IMPORTANT: preserve route_ready/route_label.  The sorter may already
        # have identified the current parcel before the PLC sends RobotReset.
        # Clearing the route here caused the following RobotStart to wait for
        # an ArUco decision that had already been published.
        self.publish_status('IDLE')
        if self.route_ready:
            self.get_logger().info(
                f'Robot fault reset | preserving current route: '
                f'label={self.route_label} -> {self.destination}')
        else:
            self.get_logger().info('Robot fault reset')

    def set_fault(self, reason):
        self.sequence_running = False
        self.robot_busy = False
        self.robot_done = False
        self.robot_home = False
        self.robot_fault = True
        self.pending_start = False
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
