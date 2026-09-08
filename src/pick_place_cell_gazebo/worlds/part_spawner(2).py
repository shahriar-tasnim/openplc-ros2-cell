#!/usr/bin/env python3
"""
part_spawner.py -- FEEDER for the UR5e stacking demo.

All 36 cubes (cube_0..cube_35) are pre-spawned in the world's magazine.
This node feeds them one at a time by teleporting the next cube to the
conveyor feed point (reliable set_pose, no spawn race), announcing it on
/cell/active_part. Advances on each /cell/robot_done rising edge.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity

FEED = (0.45, -0.85, 0.44)   # conveyor feed point (UR5e cell)
MAX_CUBES = 36


class PartSpawner(Node):
    def __init__(self):
        super().__init__("part_spawner")
        self.n = 0
        self.prev_done = False
        self.fed = False
        self.cli = self.create_client(SetEntityPose, "/world/cell_world/set_pose")
        self.active_pub = self.create_publisher(String, "/cell/active_part", 10)
        self.create_subscription(Bool, "/cell/robot_done", self.on_done, 10)
        self.create_timer(1.0, self.tick)
        self.get_logger().info("Part feeder ready (36 cubes pre-spawned)")

    def tick(self):
        # feed cube_0 once the service is up
        if not self.fed and self.cli.service_is_ready():
            self.feed(self.n)
            self.fed = True
        # keep announcing the current active cube
        self.active_pub.publish(String(data=f"cube_{self.n}"))

    def on_done(self, msg):
        rising = msg.data and not self.prev_done
        self.prev_done = msg.data
        if not rising:
            return
        if self.n + 1 >= MAX_CUBES:
            self.get_logger().info("Tower full (36/36)")
            return
        self.n += 1
        self.feed(self.n)

    def feed(self, n):
        if not self.cli.service_is_ready() and \
           not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("set_pose not available")
            return
        req = SetEntityPose.Request()
        req.entity = Entity(); req.entity.name = f"cube_{n}"
        req.entity.type = Entity.MODEL
        p = Pose()
        p.position.x = float(FEED[0]); p.position.y = float(FEED[1]); p.position.z = float(FEED[2])
        p.orientation.w = 1.0
        req.pose = p
        self.cli.call_async(req)
        self.active_pub.publish(String(data=f"cube_{n}"))
        self.get_logger().info(f"Fed cube_{n} to conveyor")


def main(args=None):
    rclpy.init(args=args)
    node = PartSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
