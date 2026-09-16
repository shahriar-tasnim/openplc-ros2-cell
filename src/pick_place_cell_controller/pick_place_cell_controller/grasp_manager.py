#!/usr/bin/env python3
"""
grasp_manager.py -- scripted kinematic grasp with LABEL-BASED SORTING.

The parcel is driven along a clean scripted path keyed to the robot status.
The destination is no longer a fixed tower: it comes from /cell/destination
(published by the sorter, derived from the parcel's detected label), so each
parcel lands on pallet A, B, C or in the reject bin.

  PICK        -> parcel at the belt pick point
  LIFT        -> straight up
  PLACE_ABOVE -> across to the routed pallet, high
  PLACE_AT    -> down onto its slot, release
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
Z_BELT  = 0.44
Z_HIGH  = sd.Z_HIGH
SET_POSE_SRV = "/world/cell_world/set_pose"


class GraspManager(Node):
    def __init__(self):
        super().__init__("grasp_manager")
        self.active_part = "cube_0"
        self.destination = "REJECT"     # latest from the sorter
        self.cycle_dest = "REJECT"      # frozen for the current parcel
        self.counts = {"A": 0, "B": 0, "C": 0, "REJECT": 0}
        self.holding = False
        self.last_status = ""
        self.target = None
        self.pos = None

        self.cli = self.create_client(SetEntityPose, SET_POSE_SRV)
        self.create_subscription(String, "/cell/active_part", self.on_active, 10)
        self.create_subscription(String, "/cell/destination", self.on_dest, 10)
        self.create_subscription(String, "/cell/robot_status", self.on_status, 10)
        self.create_timer(0.02, self.glide)
        self.get_logger().info("Grasp manager ready (label-based sorting)")

    def on_active(self, msg):
        self.active_part = msg.data

    def on_dest(self, msg):
        if msg.data in self.counts:
            self.destination = msg.data

    def slot_for_cycle(self):
        n = self.counts[self.cycle_dest]
        return sd.slot(self.cycle_dest, n)

    def on_status(self, msg):
        s = msg.data
        if s == self.last_status:
            return
        self.last_status = s

        if s.startswith("PICK"):
            # freeze the destination for this parcel at pick time
            self.cycle_dest = self.destination
            self.holding = True
            self.pos = np.array([PICK_XY[0], PICK_XY[1], Z_BELT])
            self.target = self.pos.copy()
            self.get_logger().info(
                f"{self.active_part} -> pallet {self.cycle_dest}")

        elif s.startswith("LIFT") and self.holding:
            self.target = np.array([PICK_XY[0], PICK_XY[1], Z_HIGH])

        elif s.startswith("PLACE_ABOVE") and self.holding:
            x, y, _ = self.slot_for_cycle()
            self.target = np.array([x, y, Z_HIGH])

        elif s.startswith("PLACE_AT") and self.holding:
            x, y, z = self.slot_for_cycle()
            self.target = np.array([x, y, z])

        elif s.startswith("PLACE_LIFT") and self.holding:
            self.holding = False
            self.counts[self.cycle_dest] += 1     # slot consumed
            self.get_logger().info(
                f"Placed on {self.cycle_dest} "
                f"(A={self.counts['A']} B={self.counts['B']} "
                f"C={self.counts['C']} R={self.counts['REJECT']})")

    def glide(self):
        if not self.holding or self.pos is None or self.target is None:
            return
        step = 0.003
        d = self.target - self.pos
        dist = np.linalg.norm(d)
        self.pos = self.pos + d / dist * step if dist > step else self.target.copy()
        self.move(*self.pos)

    def move(self, x, y, z):
        if not self.cli.service_is_ready():
            return
        req = SetEntityPose.Request()
        req.entity = Entity(); req.entity.name = self.active_part
        req.entity.type = Entity.MODEL
        p = Pose()
        p.position.x = float(x); p.position.y = float(y); p.position.z = float(z)
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
