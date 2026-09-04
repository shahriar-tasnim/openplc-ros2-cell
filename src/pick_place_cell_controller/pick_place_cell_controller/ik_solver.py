#!/usr/bin/env python3
"""
ik_solver.py  -  Cartesian -> joint-angle solver for the 6-DOF pick-place cell.

Why this exists:
    The old arm used hardcoded joint angles for each waypoint, so "exact"
    placement was impossible. This module takes a Cartesian target
    (x, y, z in world frame) and returns the six joint angles that put the
    gripper's tool-center-point (TCP) there, pointing straight down.

Usage in a node:
    from ik_solver import CellIK
    ik = CellIK("/absolute/path/to/cell_robot.urdf")
    j = ik.solve(0.75, -0.55, 0.34)      # -> [j1, j2, j3, j4, j5, j6]  (radians)
    # feed j straight into a JointTrajectoryPoint.positions

    # or use the precomputed named waypoints:
    j = ik.waypoints["PICK"]

Notes:
  * Output order is [joint1..joint6] -- matches controllers.yaml exactly.
  * The URDF must be the EXPANDED urdf, not the xacro. Generate it once:
        xacro cell_robot_urdf.xacro > cell_robot.urdf
    or pass your live robot_description string via CellIK.from_string(...).
  * Orientation is "approach axis down" (tool points -Z), tool spin free.
    That's the right constraint for dropping a cube and keeps IK robust.
"""

import numpy as np
from ikpy.chain import Chain

# ----- world geometry, read from cell_world.sdf -----------------------------
# part on conveyor (pick), destination bin (place). z tuned to grasp the
# 0.08 m cube at its mid-height and to clear on approach.
PICK_XY   = (0.75, -0.55)
PLACE_XY  = (0.65,  0.65)
Z_GRASP   = 0.34      # TCP height when closing on the cube
Z_CLEAR   = 0.55      # safe travel / approach height above both stations

# HOME is a joint pose, not a Cartesian target: a safe tucked configuration.
HOME_JOINTS = np.radians([0.0, -60.0, 90.0, 0.0, 60.0, 0.0])

_DOWN = np.array([0.0, 0.0, -1.0])            # approach axis target
_MASK = [False, True, True, True, True, True, True, False]  # origin + 6 joints + tcp


class CellIK:
    def __init__(self, urdf_path):
        self.chain = Chain.from_urdf_file(
            urdf_path, base_elements=["base_link"], active_links_mask=_MASK)
        self._seed = np.zeros(len(self.chain.links))
        self.waypoints = self._build_waypoints()

    @classmethod
    def from_string(cls, robot_description, tmp="/tmp/cell_robot.urdf"):
        with open(tmp, "w") as f:
            f.write(robot_description)
        return cls(tmp)

    def solve(self, x, y, z, seed=None):
        """Return [j1..j6] radians for a top-down TCP at (x,y,z)."""
        init = self._seed if seed is None else self._pad(seed)
        full = self.chain.inverse_kinematics(
            target_position=[x, y, z],
            target_orientation=_DOWN,
            orientation_mode="X",
            initial_position=init)
        self._seed = full                     # warm-start next call
        return full[1:7].tolist()

    def check(self, x, y, z, joints):
        """Return (position_error_m, tilt_error_deg) for a solution."""
        full = self._pad(joints)
        fk = self.chain.forward_kinematics(full)
        perr = float(np.linalg.norm(fk[:3, 3] - np.array([x, y, z])))
        tilt = float(np.degrees(np.arccos(np.clip(np.dot(fk[:3, 0], _DOWN), -1, 1))))
        return perr, tilt

    def _pad(self, joints):
        full = np.zeros(len(self.chain.links))
        full[1:7] = joints
        return full

    def _build_waypoints(self):
        px, py = PICK_XY
        qx, qy = PLACE_XY
        # solve in motion order so each warm-starts from the previous pose
        order = [
            ("HOME",      None),
            ("PRE_PICK",  (px, py, Z_CLEAR)),
            ("PICK",      (px, py, Z_GRASP)),
            ("LIFT",      (px, py, Z_CLEAR)),
            ("PRE_PLACE", (qx, qy, Z_CLEAR)),
            ("PLACE",     (qx, qy, Z_GRASP)),
            ("RETREAT",   (qx, qy, Z_CLEAR)),
        ]
        wp = {}
        for name, tgt in order:
            if tgt is None:
                wp[name] = HOME_JOINTS.tolist()
                self._seed = self._pad(HOME_JOINTS)
            else:
                wp[name] = self.solve(*tgt)
        return wp


if __name__ == "__main__":
    import sys
    urdf = sys.argv[1] if len(sys.argv) > 1 else "cell_robot_ik.urdf"
    ik = CellIK(urdf)
    print(f"{'waypoint':<12}{'posErr(mm)':>11}{'tilt(deg)':>10}   joint1..joint6 (deg)")
    coords = {  # for error reporting only
        "PRE_PICK": (*PICK_XY, Z_CLEAR), "PICK": (*PICK_XY, Z_GRASP),
        "LIFT": (*PICK_XY, Z_CLEAR), "PRE_PLACE": (*PLACE_XY, Z_CLEAR),
        "PLACE": (*PLACE_XY, Z_GRASP), "RETREAT": (*PLACE_XY, Z_CLEAR),
    }
    for name, j in ik.waypoints.items():
        deg = np.round(np.degrees(j), 1)
        if name in coords:
            perr, tilt = ik.check(*coords[name], j)
            print(f"{name:<12}{perr*1000:>11.2f}{tilt:>10.2f}   {deg}")
        else:
            print(f"{name:<12}{'--':>11}{'--':>10}   {deg}   (fixed joint pose)")
