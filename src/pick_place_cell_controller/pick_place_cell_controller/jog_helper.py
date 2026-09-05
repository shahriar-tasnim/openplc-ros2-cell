#!/usr/bin/env python3
"""
jog_helper.py -- interactive teach pendant for the UR5e in Gazebo.

Move one joint at a time, watch the arm in Gazebo, and print/save the
current pose when the tool is where you want it.

Run (with the cell launched + controller active):
    python3 jog_helper.py

Commands (type at the prompt):
    1 +      nudge joint 1 by +step
    3 -      nudge joint 3 by -step
    step 0.05   set the nudge size (radians)
    show     print current pose (6 joint angles)
    save PICK   save current pose under a label
    list     show all saved poses
    dump     print all saved poses (copy this to send back)
    home     go to a safe tucked pose
    q        quit (also prints dump)

Joint index: 1=pan 2=lift 3=elbow 4=wrist1 5=wrist2 6=wrist3
"""
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

JOINTS = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
          "wrist_1_joint","wrist_2_joint","wrist_3_joint"]
HOME = [0.0, -1.2, 1.4, -1.75, -1.57, 0.0]


class Jog(Node):
    def __init__(self):
        super().__init__("jog_helper")
        self.pos = list(HOME)
        self.have_state = False
        self.step = 0.10
        self.saved = {}
        self.pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)
        self.create_subscription(JointState, "/joint_states", self.on_state, 10)

    def on_state(self, msg):
        # map incoming names to our order
        idx = {n: i for i, n in enumerate(msg.name)}
        if all(j in idx for j in JOINTS):
            self.pos = [msg.position[idx[j]] for j in JOINTS]
            self.have_state = True

    def send(self, positions, secs=2):
        jt = JointTrajectory()
        jt.joint_names = JOINTS
        p = JointTrajectoryPoint()
        p.positions = [float(v) for v in positions]
        p.time_from_start = Duration(sec=int(secs))
        jt.points = [p]
        self.pub.publish(jt)


def main():
    rclpy.init()
    node = Jog()
    print(__doc__)
    # spin briefly to get first joint state
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.have_state:
            break
    print("Ready. Current pose:", [round(v, 3) for v in node.pos])
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.01)
            cmd = input("jog> ").strip().split()
            if not cmd:
                continue
            if cmd[0] == "q":
                break
            elif cmd[0] == "step" and len(cmd) == 2:
                node.step = float(cmd[1]); print("step =", node.step)
            elif cmd[0] == "show":
                print("pose:", [round(v, 4) for v in node.pos])
            elif cmd[0] == "home":
                node.send(HOME, 3); print("moving HOME...")
            elif cmd[0] == "save" and len(cmd) == 2:
                node.saved[cmd[1]] = [round(v, 4) for v in node.pos]
                print(f"saved {cmd[1]} = {node.saved[cmd[1]]}")
            elif cmd[0] == "list":
                for k in node.saved: print(" ", k)
            elif cmd[0] == "dump":
                print("\n=== COPY BELOW ===")
                for k, v in node.saved.items():
                    print(f"{k}: {v}")
                print("=== END ===\n")
            elif cmd[0] in ("1","2","3","4","5","6") and len(cmd) == 2:
                j = int(cmd[0]) - 1
                d = node.step if cmd[1] == "+" else -node.step
                node.pos[j] += d
                node.send(node.pos, 1)
                print(f"joint {j+1} -> {round(node.pos[j],4)}")
            else:
                print("?? see commands at top")
    except (KeyboardInterrupt, EOFError):
        pass
    print("\n=== SAVED POSES (copy this) ===")
    for k, v in node.saved.items():
        print(f"{k}: {v}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
