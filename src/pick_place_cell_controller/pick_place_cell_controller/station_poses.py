"""station_poses.py -- UR5e arm poses aimed at the REAL cell coordinates.

Solved with analytic UR5e IK for the actual pick point and the four
sorting stations, accounting for the 0.12 m gripper length.

    PICK     (0.45, -0.20)   parcel on the belt
    A        ( 0.58,  0.12)
    B        ( 0.33,  0.45)
    C        ( 0.00,  0.52)
    REJECT   (-0.30,  0.40)
"""

JOINT_NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

PRE_PICK = [-0.6923, -1.606, 1.7277, -1.6926, -1.5708, -2.2631]
PICK     = [-0.6923, -1.2583, 2.1586, -2.4712, -1.5708, -2.2631]
LIFT     = [-0.6923, -1.606, 1.7277, -1.6926, -1.5708, -2.2631]

PLACE = {
    "A": [-0.023, -1.0531, 1.8748, -2.3925, -1.5708, -1.5938],
    "B": [0.6968, -1.1119, 1.9785, -2.4374, -1.5708, -0.874],
    "C": [1.3116, -1.1769, 2.0898, -2.4837, -1.5708, -0.2592],
    "REJECT": [1.9444, -1.2113, 2.147, -2.5065, -1.5708, 0.3736],
}
