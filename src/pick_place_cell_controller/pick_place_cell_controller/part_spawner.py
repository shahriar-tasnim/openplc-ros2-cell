#!/usr/bin/env python3
"""
part_spawner.py -- strict FEEDER for the UR5e stacking demo.

36 cubes pre-spawned in the world magazine. Feeds them ONE at a time:
  - feeds cube_0 once at startup
  - advances to the next cube only after a COMPLETE robot cycle
    (robot goes busy, then done) -- not on any stray robot_done blip.

This state-machine gating prevents the feeder from racing ahead.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity

FEED = (0.45, -0.85, 0.44)
MAX_CUBES = 36


class PartSpawner(Node):
    def __init__(self):
        super().__init__("part_spawner")
        self.n = -1                 # nothing fed yet
        self.saw_busy = False       # has the robot picked up THIS cube?
        self.prev_done = False
        self.started = False
        self.cli = self.create_client(SetEntityPose, "/world/cell_world/set_pose")
        self.active_pub = self.create_publisher(String, "/cell/active_part", 10)
        self.create_subscription(Bool, "/cell/robot_busy", self.on_busy, 10)
        self.create_subscription(Bool, "/cell/robot_done", self.on_done, 10)
        self.create_timer(0.5, self.startup)
        self.get_logger().info("Part feeder ready (strict, 36 cubes)")

    def startup(self):
        # feed the first cube ONCE, when the set_pose service is ready
        if not self.started and self.cli.service_is_ready():
            self.n = 0
            self.feed(self.n)
            self.started = True

    def on_busy(self, msg):
        # robot has taken THIS cube -> arm it for advancing on done
        if msg.data:
            self.saw_busy = True

    def on_done(self, msg):
        rising = msg.data and not self.prev_done
        self.prev_done = msg.data
        if not rising:
            return
        # only advance if the robot actually picked this cube up first
        if not self.saw_busy:
            return
        self.saw_busy = False
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
