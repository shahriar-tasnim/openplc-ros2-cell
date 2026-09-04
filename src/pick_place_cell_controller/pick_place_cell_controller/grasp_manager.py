#!/usr/bin/env python3
"""
grasp_manager.py -- simulated grasp/release, synced to the arm's waypoints.

Holds the active cube kinematically (teleport via SetEntityPose) while the
arm carries it, then releases it into its grid cell. The grid cell is taken
from the cube NAME (cube_<n> -> cell n), which the spawner sets -- so this
node never keeps its own counter and cannot drift out of sync.

Handoff with the conveyor is clean: the conveyor stops teleporting the cube
once robot_busy goes high, so only one node moves a given cube at a time.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity

from pick_place_cell_controller import grid_place

PICK_XY = (0.75, -0.55)
Z_PICK  = 0.31          # cube height on the conveyor
Z_CARRY = 0.47          # travel height while carried
Z_REST  = 0.09          # cube center when resting on the floor
SET_POSE_SRV = "/world/cell_world/set_pose"


class GraspManager(Node):
    def __init__(self):
        super().__init__("grasp_manager")
        self.active_part = "part"
        self.holding = False
        self.last_status = ""

        self.cli = self.create_client(SetEntityPose, SET_POSE_SRV)
        self.create_subscription(String, "/cell/active_part",
                                 self.on_active, 10)
        self.create_subscription(String, "/cell/robot_status",
                                 self.on_status, 10)
        self.get_logger().info("Grasp manager ready")

    def on_active(self, msg):
        self.active_part = msg.data

    def cell_index(self):
        try:
            return int(self.active_part.split("_")[-1]) % grid_place.NUM_CELLS
        except (ValueError, IndexError):
            return 0

    def on_status(self, msg):
        s = msg.data
        if s == self.last_status:
            return
        self.last_status = s

        if s.startswith("PICK"):
            self.holding = True
            self.move(PICK_XY[0], PICK_XY[1], Z_PICK)
            self.get_logger().info(f"Holding {self.active_part}")

        elif s.startswith("LIFT") and self.holding:
            self.move(PICK_XY[0], PICK_XY[1], Z_CARRY)

        elif s.startswith("PLACE_ABOVE") and self.holding:
            x, y = grid_place.GRID_CELLS[self.cell_index()]["xy"]
            self.move(x, y, Z_CARRY)

        #elif s.startswith("PLACE_AT") and self.holding:
        elif (s.startswith("PLACE_AT") or s == "PLACE") and self.holding:
            cell = grid_place.GRID_CELLS[self.cell_index()]
            x, y = cell["xy"]
            self.move(x, y, Z_REST)
            self.holding = False
            self.get_logger().info(
                f"Released {self.active_part} at cell {cell['rc']}")

    def move(self, x, y, z):
        if not self.cli.service_is_ready() and \
           not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("set_pose service unavailable")
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
