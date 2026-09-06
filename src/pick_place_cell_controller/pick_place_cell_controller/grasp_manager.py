#!/usr/bin/env python3
"""
grasp_manager.py -- kinematic grasp for the UR5e cell, WITH live coordinate
readout so you can verify/correct poses.

At PICK and PLACE it prints:
  * tool0 position (where the gripper actually is, from TF)
  * cube position (where the cube actually is, from TF/known target)
  * the gap between them  --> tells you exactly how much to adjust.

It also teleports the cube to follow the tool while carried, and releases it
at the tower slot (same behaviour as before), so the demo still works.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
from tf2_ros import Buffer, TransformListener

# tower geometry in WORLD frame (base_link at pedestal top z=0.40)
PICK_XY = (0.45, -0.20)
TC = (0.45, 0.20); PITCH = 0.10
Z_BELT = 0.44       # cube center height on the conveyor (world)
Z_CARRY = 0.72      # carry height (world)
Z_T0 = 0.44; Z_STEP = 0.08   # tower layer heights (world)
SET_POSE_SRV = "/world/cell_world/set_pose"
GRIP_OFFSET_Z = -0.02   # cube sits 2 cm below the tool0 flange (in the grip)


def tower_xyz(i):
    L = i // 9; w = i % 9; r = w // 3
    c = (w % 3) if r % 2 == 0 else (2 - w % 3)
    return TC[0]+(c-1)*PITCH, TC[1]+(r-1)*PITCH, Z_T0 + L*Z_STEP


class GraspManager(Node):
    def __init__(self):
        super().__init__("grasp_manager")
        self.active_part = "part"
        self.holding = False
        self.last_status = ""
        self.cli = self.create_client(SetEntityPose, SET_POSE_SRV)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(String, "/cell/active_part", self.on_active, 10)
        self.create_subscription(String, "/cell/robot_status", self.on_status, 10)
        self.create_timer(0.05, self.follow_tool)   # 20 Hz smooth carry
        self.get_logger().info("Grasp manager ready (with coordinate readout)")

    def on_active(self, msg):
        self.active_part = msg.data

    def cube_index(self):
        try:
            return int(self.active_part.split("_")[-1]) % 36
        except (ValueError, IndexError):
            return 0

    def tool0(self):
        """Live tool0 position in world/base_link frame from TF, or None."""
        for parent in ("world", "base_link"):
            try:
                t = self.tf_buffer.lookup_transform(
                    parent, "tool0", rclpy.time.Time())
                p = t.transform.translation
                # base_link is at z=0.40 world; convert if needed
                z = p.z + (0.40 if parent == "base_link" else 0.0)
                return np.array([p.x, p.y, z]), parent
            except Exception:
                continue
        return None, None

    def report(self, label, cube_xyz):
        tp, frame = self.tool0()
        if tp is None:
            self.get_logger().warn(f"[{label}] tool0 TF not available yet")
            return
        gap = np.array(cube_xyz) - tp
        self.get_logger().info(
            f"[{label}] tool0={np.round(tp,3).tolist()} (via {frame}) | "
            f"cube_target={list(np.round(cube_xyz,3))} | "
            f"gap(cube-tool)={np.round(gap,3).tolist()}  "
            f"=> dx={gap[0]*100:+.1f}cm dy={gap[1]*100:+.1f}cm dz={gap[2]*100:+.1f}cm")

    def on_status(self, msg):
        s = msg.data
        if s == self.last_status:
            return
        self.last_status = s
        i = self.cube_index()

        if s.startswith("PICK"):
            self.holding = True
            cube = (PICK_XY[0], PICK_XY[1], Z_BELT)
            self.report("PICK", cube)        # <-- shows grasp alignment

        elif s.startswith("PLACE_AT") and self.holding:
            x, y, z = tower_xyz(i)
            self.report("PLACE", (x, y, z))  # <-- shows drop alignment
            self.holding = False              # stop following FIRST
            self.move(x, y, z)                # set cube down on the slot
            self.get_logger().info(f"Released {self.active_part} at cell {i}")

    def follow_tool(self):
        """While holding, glue the cube to the live tool0 pose (smooth carry)."""
        if not self.holding:
            return
        tp, _ = self.tool0()
        if tp is not None:
            self.move(tp[0], tp[1], tp[2] + GRIP_OFFSET_Z)

    def move(self, x, y, z):
        if not self.cli.service_is_ready() and \
           not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("set_pose service unavailable")
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
