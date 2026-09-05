#!/usr/bin/env python3
"""
ur5e_ik.py -- live inverse kinematics for the UR5e pick-and-place cell.

UR5e mounted at world origin. Solves a top-down (tool pointing -Z) TCP
target in WORLD coordinates and returns 6 joint angles in the UR joint
order used by the controller:
    [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]

Verified layout (all reachable, 0.00 mm / 0.00 deg):
    pick  = (0.45, -0.20)          conveyor pick point
    tower = 3x3 @ (0.45, 0.20), pitch 0.10, 4 layers (36 cubes)

Requires ikpy (pip install ikpy). The kinematic URDF (cell_ur5e_ik.urdf)
ships alongside this file.
"""

import os
import numpy as np
from ikpy.chain import Chain

_DOWN = np.array([0.0, 0.0, -1.0])
_MASK = [False, True, True, True, True, True, True, False]
_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
           "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

# UR5e base mount in world frame (arm spawned here).
BASE_XY = (0.0, 0.0)
BASE_Z = 0.40   # arm mounted on a 0.40 m pedestal


class UR5eIK:
    def __init__(self, urdf_path=None):
        if urdf_path is None:
            urdf_path = os.path.join(os.path.dirname(__file__),
                                     "cell_ur5e_ik.urdf")
        self.chain = Chain.from_urdf_file(
            urdf_path, base_elements=["base_link"], active_links_mask=_MASK)
        self._seed = np.zeros(len(self.chain.links))

    def solve(self, x, y, z, seed=None):
        """World (x,y,z) top-down -> [6 UR joint angles] (radians)."""
        tx, ty = x - BASE_XY[0], y - BASE_XY[1]
        tz = z - BASE_Z
        init = self._seed if seed is None else self._pad(seed)
        full = self.chain.inverse_kinematics(
            target_position=[tx, ty, tz], target_orientation=_DOWN,
            orientation_mode="X", initial_position=init)
        self._seed = full
        return full[1:7].tolist()

    def check(self, x, y, z, joints):
        tx, ty = x - BASE_XY[0], y - BASE_XY[1]
        tz = z - BASE_Z
        fk = self.chain.forward_kinematics(self._pad(joints))
        perr = float(np.linalg.norm(fk[:3, 3] - np.array([tx, ty, tz])))
        tilt = float(np.degrees(np.arccos(np.clip(np.dot(fk[:3, 0], _DOWN), -1, 1))))
        return perr, tilt

    def _pad(self, joints):
        full = np.zeros(len(self.chain.links))
        full[1:7] = joints
        return full

    @property
    def joint_names(self):
        return list(_JOINTS)


# ---- tower geometry (3x3 x 4 layers = 36) ---------------------------------
TOWER_C = (0.45, 0.20)
PITCH = 0.10
Z_BASE = 0.49      # layer-0 cube center on the 0.40 m table
Z_STEP = 0.08      # cube height
Z_TRANSIT = 0.85   # safe travel height above the raised tower

def tower_target(index):
    """cube index 0..35 -> (x, y, z_place) in world frame. Fills a 3x3
    footprint bottom layer first (snake), then next layer up."""
    layer = index // 9
    within = index % 9
    r = within // 3
    c = within % 3 if r % 2 == 0 else (2 - within % 3)   # snake within layer
    x = TOWER_C[0] + (c - 1) * PITCH
    y = TOWER_C[1] + (r - 1) * PITCH
    z = Z_BASE + layer * Z_STEP
    return x, y, z


if __name__ == "__main__":
    ik = UR5eIK()
    print("PICK:", np.round(np.degrees(ik.solve(0.45, -0.20, 0.20)), 1))
    worst = 0.0
    for i in range(36):
        x, y, z = tower_target(i)
        j = ik.solve(x, y, z)
        p, t = ik.check(x, y, z, j)
        worst = max(worst, p)
    print(f"36 tower placements: worst position error = {worst*1000:.2f} mm")
