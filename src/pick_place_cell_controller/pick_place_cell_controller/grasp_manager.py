#!/usr/bin/env python3
"""
grasp_manager.py -- duration-matched kinematic grasp with label sorting.

The parcel follows a scripted path (reliable, no TF tracking), but instead of
gliding at a fixed speed it interpolates from its current position to the
target over EXACTLY the time the arm is given for that move. Both therefore
start and finish together, whatever the distance.

Move durations must match MOVE_TIME in gazebo_robot_node.py.

  PICK      -> parcel at the belt, destination frozen
  LIFT      -> rises to travel height in step with the arm
  PLACE_AT  -> crosses to its slot and settles, in step with the arm
  PRE_PICK  -> released; arm returns for the next parcel
"""
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity

from pick_place_cell_controller import sort_destinations as sd

PICK_XY = (0.45, -0.20)
Z_BELT = 0.44
Z_HIGH = sd.Z_HIGH
TICK = 0.02                      # 50 Hz update

# how long the ARM takes for each move -- keep in step with the robot node
MOVE_TIME = {
    "LIFT":     0.8,
    "PLACE_AT": 1.6,
}
SET_POSE_SRV = "/world/cell_world/set_pose"


class GraspManager(Node):
    def __init__(self):
        super().__init__("grasp_manager")
        self.active_part = "cube_0"
        self.destination = "REJECT"
        self.cycle_dest = "REJECT"
        self.counts = {"A": 0, "B": 0, "C": 0, "REJECT": 0}
        self.holding = False
        self.last_status = None
        self.pos = np.array([PICK_XY[0], PICK_XY[1], Z_BELT])

        # interpolation state
        self.start = None            # where the move began
        self.target = None           # where it should end
        self.elapsed = 0.0
        self.duration = 1.0

        self.cli = self.create_client(SetEntityPose, SET_POSE_SRV)
        self.create_subscription(String, "/cell/active_part", self.on_active, 10)
        self.create_subscription(String, "/cell/destination", self.on_dest, 10)
        self.create_subscription(String, "/cell/robot_status", self.on_status, 10)
        self.create_timer(TICK, self.update)
        self.get_logger().info("Grasp manager ready (duration-matched)")

    def on_active(self, msg):
        self.active_part = msg.data

    def on_dest(self, msg):
        if msg.data in self.counts:
            self.destination = msg.data

    def slot_for_cycle(self):
        return sd.slot(self.cycle_dest, self.counts[self.cycle_dest])

    def begin_move(self, target, duration):
        """Interpolate from the current position to `target` over `duration`."""
        self.start = self.pos.copy()
        self.target = np.asarray(target, dtype=float)
        self.duration = max(duration, TICK)
        self.elapsed = 0.0

    def on_status(self, msg):
        s = msg.data
        if s == getattr(self, "last_status", None):
            return
        self.last_status = s

        if s.startswith("PICK"):
            self.cycle_dest = self.destination
            self.holding = True
            self.pos = np.array([PICK_XY[0], PICK_XY[1], Z_BELT])
            self.start = self.pos.copy()
            self.target = self.pos.copy()
            self.elapsed = 0.0
            self.duration = 1.0
            self.get_logger().info(
                f"{self.active_part} -> station {self.cycle_dest}")

        elif s.startswith("LIFT") and self.holding:
            self.begin_move([PICK_XY[0], PICK_XY[1], Z_HIGH],
                            MOVE_TIME["LIFT"])

        elif s.startswith("PLACE_AT") and self.holding:
            x, y, z = self.slot_for_cycle()
            self.begin_move([x, y, z], MOVE_TIME["PLACE_AT"])

        elif s.startswith("PRE_PICK") and self.holding:
            x, y, z = self.slot_for_cycle()
            self.pos = np.array([x, y, z])
            self.move(x, y, z)
            self.holding = False
            self.counts[self.cycle_dest] += 1
            self.get_logger().info(
                f"Placed on {self.cycle_dest} at ({x:.2f}, {y:.2f}, {z:.2f}) | "
                f"A={self.counts['A']} B={self.counts['B']} "
                f"C={self.counts['C']} R={self.counts['REJECT']}")

    def update(self):
        if not self.holding or self.start is None or self.target is None:
            return
        self.elapsed += TICK
        t = min(self.elapsed / self.duration, 1.0)
        # smoothstep easing so it accelerates and decelerates like the arm
        e = t * t * (3.0 - 2.0 * t)
        self.pos = self.start + (self.target - self.start) * e
        self.move(*self.pos)

    def move(self, x, y, z):
        if not self.cli.service_is_ready():
            return
        req = SetEntityPose.Request()
        req.entity = Entity()
        req.entity.name = self.active_part
        req.entity.type = Entity.MODEL
        p = Pose()
        p.position.x = float(x)
        p.position.y = float(y)
        p.position.z = float(z)
        p.orientation.w = 1.0
        req.pose = p
        self.cli.call_async(req)


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
