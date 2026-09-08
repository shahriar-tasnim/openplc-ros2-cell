#!/usr/bin/env python3
"""gripper_follower.py -- glues the visual gripper model to the UR5e tool0
frame every 50 ms, so it moves and rotates with the arm. Visual only."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
from tf2_ros import Buffer, TransformListener

GRIPPER_NAME = "gripper"
BASE_Z = 0.40   # base_link sits at pedestal top


class GripperFollower(Node):
    def __init__(self):
        super().__init__("gripper_follower")
        self.cli = self.create_client(SetEntityPose, "/world/cell_world/set_pose")
        self.tf = Buffer(); TransformListener(self.tf, self)
        self.create_timer(0.02, self.tick)  # 50 Hz to reduce lag
        self.get_logger().info("Gripper follower ready")

    def tick(self):
        for parent, dz in (("world", 0.0), ("base_link", BASE_Z)):
            try:
                t = self.tf.lookup_transform(parent, "tool0", rclpy.time.Time())
                tr = t.transform.translation; ro = t.transform.rotation
                if not self.cli.service_is_ready():
                    return
                req = SetEntityPose.Request()
                req.entity = Entity(); req.entity.name = GRIPPER_NAME
                req.entity.type = Entity.MODEL
                p = Pose()
                p.position.x = tr.x; p.position.y = tr.y; p.position.z = tr.z + dz
                p.orientation = ro
                req.pose = p
                self.cli.call_async(req)
                return
            except Exception:
                continue


def main(args=None):
    rclpy.init(args=args)
    n = GripperFollower()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
