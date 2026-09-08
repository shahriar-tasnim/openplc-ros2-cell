#!/usr/bin/env python3
"""
grasp_manager.py -- scripted kinematic grasp for the UR5e cell.

The gripper's actual tool0 position doesn't line up with the belt/tower
coordinates, so instead of following it we drive the cube along a clean
scripted path keyed to the robot status. Fully deterministic, no drift:

  PICK        -> cube at belt pick point (0.45,-0.20), grasp height
  LIFT        -> cube straight up (same XY, high)
  PLACE_ABOVE -> cube moves over the target tower slot (high)
  PLACE_AT    -> cube lowers onto the tower slot, release

Cube heights are WORLD frame (pedestal/table tops at 0.40).
"""
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity

# world coordinates
PICK_XY = (0.45, -0.20)
TC = (0.45, 0.20); PITCH = 0.10          # tower center + pitch
Z_BELT  = 0.44                            # cube on the belt
Z_HIGH  = 0.75                            # travel/lift height
Z_T0    = 0.44; Z_STEP = 0.08            # tower layer base + step
SET_POSE_SRV = "/world/cell_world/set_pose"


def tower_xyz(i):
    L = i // 9; w = i % 9; r = w // 3
    c = (w % 3) if r % 2 == 0 else (2 - w % 3)
    return TC[0]+(c-1)*PITCH, TC[1]+(r-1)*PITCH, Z_T0 + L*Z_STEP


class GraspManager(Node):
    def __init__(self):
        super().__init__("grasp_manager")
        self.active_part = "cube_0"
        self.holding = False
        self.last_status = ""
        self.target = None          # (x,y,z) the cube should glide toward
        self.pos = None             # current smoothed cube position
        self.cli = self.create_client(SetEntityPose, SET_POSE_SRV)
        self.create_subscription(String, "/cell/active_part", self.on_active, 10)
        self.create_subscription(String, "/cell/robot_status", self.on_status, 10)
        self.create_timer(0.02, self.glide)   # 50 Hz smooth glide
        self.get_logger().info("Grasp manager ready (scripted path)")

    def on_active(self, msg):
        self.active_part = msg.data

    def cube_index(self):
        try:
            return int(self.active_part.split("_")[-1]) % 36
        except (ValueError, IndexError):
            return 0

    def on_status(self, msg):
        s = msg.data
        if s == self.last_status:
            return
        self.last_status = s
        i = self.cube_index()
        tx, ty, tz = tower_xyz(i)

        if s.startswith("PICK"):
            self.holding = True
            self.pos = np.array([PICK_XY[0], PICK_XY[1], Z_BELT])  # start on belt
            self.target = np.array([PICK_XY[0], PICK_XY[1], Z_BELT])
        elif s.startswith("LIFT") and self.holding:
            self.target = np.array([PICK_XY[0], PICK_XY[1], Z_HIGH])   # straight up
        elif s.startswith("PLACE_ABOVE") and self.holding:
            self.target = np.array([tx, ty, Z_HIGH])                   # over slot
        elif s.startswith("PLACE_AT") and self.holding:
            self.target = np.array([tx, ty, tz])                       # down onto slot
        elif s.startswith("PLACE_LIFT"):
            self.holding = False                                        # released, stays

    def glide(self):
        if not self.holding or self.pos is None or self.target is None:
            return
        # move current position toward target smoothly (rate-limited)
        step = 0.003   # m per tick (~1 m/s at 50 Hz)
        d = self.target - self.pos
        dist = np.linalg.norm(d)
        if dist > step:
            self.pos = self.pos + d / dist * step
        else:
            self.pos = self.target.copy()
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
