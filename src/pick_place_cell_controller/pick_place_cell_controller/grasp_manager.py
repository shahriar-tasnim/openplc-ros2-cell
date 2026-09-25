#!/usr/bin/env python3
"""
grasp_manager.py -- TF-synchronised parcel attachment for the UR5e cell.

The old implementation moved the parcel along a second scripted trajectory
whose durations were chosen to match the arm.  That works approximately, but
small controller/physics delays make the box appear to lead or lag the gripper.

This version uses the robot's ACTUAL tool0 TF while the parcel is carried:

  PICK      -> latch the destination, but leave the parcel on the belt
  LIFT      -> PICK has completed; attach parcel to tool0
  PLACE_AT  -> parcel keeps following tool0 continuously
  PRE_PICK  -> PLACE_AT has completed; snap to pallet slot and release

The PLC / Modbus handshake is unchanged.  The parcel pose is updated at 50 Hz
from the robot state publisher, so it follows real arm motion rather than a
separate timer.
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
from tf2_ros import Buffer, TransformListener

from pick_place_cell_controller import sort_destinations as sd

PICK_XY = (0.45, -0.20)
Z_BELT = 0.44
TICK = 0.02                       # 50 Hz following
SET_POSE_SRV = "/world/cell_world/set_pose"

# The UR5e model is spawned by cell_sim.launch.py at z=0.40.  robot_state_publisher
# knows the arm kinematics but not the Gazebo spawn translation, so use
# base_link->tool0 TF and add this fixed world offset.
ROBOT_SPAWN_XYZ = (0.0, 0.0, 0.40)
TOOL_FRAME = "tool0"
BASE_FRAME = "base_link"


def _q_normalize(q):
    n = math.sqrt(sum(v * v for v in q))
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(v / n for v in q)


def _q_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def _q_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _q_rotate(q, v):
    """Rotate 3-vector v by quaternion q=(x,y,z,w)."""
    q = _q_normalize(q)
    vx, vy, vz = v
    r = _q_multiply(_q_multiply(q, (vx, vy, vz, 0.0)), _q_conjugate(q))
    return (r[0], r[1], r[2])


class GraspManager(Node):
    def __init__(self):
        super().__init__("grasp_manager")

        self.active_part = "cube_0"
        self.destination = "REJECT"
        self.cycle_dest = "REJECT"
        self.counts = {"A": 0, "B": 0, "C": 0, "REJECT": 0}

        self.last_status = None
        self.holding = False
        self.attach_pending = False
        self.grasp_offset_tool = (0.0, 0.0, 0.0)
        self.orientation_offset = (0.0, 0.0, 0.0, 1.0)
        self.tf_warned = False

        # Use a latest-pose queue rather than firing unlimited async service
        # calls.  If Gazebo takes longer than one 20 ms tick, intermediate
        # poses are simply replaced by the newest one.
        self.pose_request_in_flight = False
        self.queued_pose = None

        self.cli = self.create_client(SetEntityPose, SET_POSE_SRV)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(String, "/cell/active_part", self.on_active, 10)
        self.create_subscription(String, "/cell/destination", self.on_dest, 10)
        self.create_subscription(String, "/cell/sort_decision", self.on_sort_decision, 10)
        self.create_subscription(String, "/cell/robot_status", self.on_status, 10)
        self.create_timer(TICK, self.update)

        self.get_logger().info(
            "Grasp manager ready | TF-synchronised tool0 attachment at 50 Hz")

    def on_active(self, msg):
        self.active_part = msg.data

    def on_dest(self, msg):
        if msg.data in self.counts:
            self.destination = msg.data

    def on_sort_decision(self, msg):
        """Use the exact same latched route event as the robot node."""
        for tok in msg.data.split():
            if tok.startswith("dest="):
                dest = tok.split("=", 1)[1]
                if dest in self.counts:
                    self.destination = dest
                return

    def slot_for_cycle(self):
        return sd.slot(self.cycle_dest, self.counts[self.cycle_dest])

    def tool_pose_world(self):
        """Return ((x,y,z), (qx,qy,qz,qw)) for tool0 in Gazebo world."""
        try:
            t = self.tf_buffer.lookup_transform(
                BASE_FRAME, TOOL_FRAME, rclpy.time.Time())
        except Exception as exc:
            if not self.tf_warned:
                self.get_logger().warn(
                    f"Waiting for {BASE_FRAME}->{TOOL_FRAME} TF: {exc}")
                self.tf_warned = True
            return None

        self.tf_warned = False
        tr = t.transform.translation
        ro = t.transform.rotation
        pos = (
            tr.x + ROBOT_SPAWN_XYZ[0],
            tr.y + ROBOT_SPAWN_XYZ[1],
            tr.z + ROBOT_SPAWN_XYZ[2],
        )
        q = _q_normalize((ro.x, ro.y, ro.z, ro.w))
        return pos, q

    def capture_grasp_transform(self):
        """Freeze parcel pose relative to tool0 at the completed PICK pose."""
        tool = self.tool_pose_world()
        if tool is None:
            return False

        tool_pos, tool_q = tool
        parcel_pos = (PICK_XY[0], PICK_XY[1], Z_BELT)
        delta_world = (
            parcel_pos[0] - tool_pos[0],
            parcel_pos[1] - tool_pos[1],
            parcel_pos[2] - tool_pos[2],
        )

        inv_tool_q = _q_conjugate(tool_q)
        self.grasp_offset_tool = _q_rotate(inv_tool_q, delta_world)

        # Parcel is upright on the conveyor when grasped.  Store its initial
        # orientation relative to the tool so it remains rigidly attached if
        # the wrist orientation changes during the path.
        parcel_q_world = (0.0, 0.0, 0.0, 1.0)
        self.orientation_offset = _q_multiply(inv_tool_q, parcel_q_world)
        self.attach_pending = False

        self.get_logger().info(
            f"Attached {self.active_part} to tool0 | "
            f"offset=({self.grasp_offset_tool[0]:.3f}, "
            f"{self.grasp_offset_tool[1]:.3f}, "
            f"{self.grasp_offset_tool[2]:.3f})")
        return True

    def on_status(self, msg):
        s = msg.data
        if s == self.last_status:
            return
        self.last_status = s

        if s.startswith("PICK"):
            # Robot has just STARTED the pick move.  Freeze routing now, but do
            # not attach yet; the parcel must remain on the belt until PICK
            # trajectory completion.
            self.cycle_dest = self.destination
            self.holding = False
            self.attach_pending = False
            self.get_logger().info(
                f"Pick started: {self.active_part} -> station {self.cycle_dest}")

        elif s.startswith("LIFT"):
            # LIFT status is emitted only after PICK's trajectory result is
            # successful.  This is the physical grasp event.
            self.holding = True
            self.attach_pending = True
            self.capture_grasp_transform()

        elif s.startswith("PLACE_AT") and self.holding:
            # Nothing to script: the parcel is already rigidly following
            # tool0.  This log is useful when checking cycle timing.
            self.get_logger().info(
                f"Carrying {self.active_part} to {self.cycle_dest}")

        elif s.startswith("PRE_PICK") and (self.holding or self.attach_pending):
            # PRE_PICK is emitted only after PLACE_AT has completed.  Release
            # exactly now, snap to the next pallet slot, and let the arm return.
            x, y, z = self.slot_for_cycle()
            self.holding = False
            self.attach_pending = False
            self.queue_pose(x, y, z, (0.0, 0.0, 0.0, 1.0))
            self.counts[self.cycle_dest] += 1
            self.get_logger().info(
                f"Released {self.active_part} on {self.cycle_dest} at "
                f"({x:.2f}, {y:.2f}, {z:.2f}) | "
                f"A={self.counts['A']} B={self.counts['B']} "
                f"C={self.counts['C']} R={self.counts['REJECT']}")

    def update(self):
        if self.holding:
            if self.attach_pending and not self.capture_grasp_transform():
                self.pump_pose_request()
                return

            tool = self.tool_pose_world()
            if tool is not None:
                tool_pos, tool_q = tool
                offset_world = _q_rotate(tool_q, self.grasp_offset_tool)
                parcel_pos = (
                    tool_pos[0] + offset_world[0],
                    tool_pos[1] + offset_world[1],
                    tool_pos[2] + offset_world[2],
                )
                parcel_q = _q_normalize(
                    _q_multiply(tool_q, self.orientation_offset))
                self.queue_pose(*parcel_pos, parcel_q)

        self.pump_pose_request()

    def queue_pose(self, x, y, z, q):
        """Keep only the newest desired pose."""
        self.queued_pose = (
            self.active_part, float(x), float(y), float(z), tuple(q))

    def pump_pose_request(self):
        if self.pose_request_in_flight or self.queued_pose is None:
            return
        if not self.cli.service_is_ready():
            return

        entity_name, x, y, z, q = self.queued_pose
        self.queued_pose = None

        req = SetEntityPose.Request()
        req.entity = Entity()
        req.entity.name = entity_name
        req.entity.type = Entity.MODEL

        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = z
        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]
        req.pose = p

        self.pose_request_in_flight = True
        future = self.cli.call_async(req)
        future.add_done_callback(self.pose_response)

    def pose_response(self, future):
        self.pose_request_in_flight = False
        try:
            result = future.result()
            if not result.success:
                self.get_logger().warn("Gazebo rejected parcel pose update")
        except Exception as exc:
            self.get_logger().error(f"SetEntityPose failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = GraspManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
