#!/usr/bin/env python3
"""
part_spawner.py -- parcel feeder for the sorting cell.

All 36 parcels (cube_0..cube_35) are pre-spawned in the world magazine.
This node feeds them one at a time to the conveyor in a SHUFFLED order, so
parcels arrive with mixed labels and the sorter visibly routes them to
different pallets (A / B / C / REJECT) rather than filling one pallet first.

Advances only after a complete robot cycle (busy -> done), so it cannot
race ahead of the cell.
"""
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity

FEED = (0.45, -0.85, 0.44)     # conveyor feed point
NUM_PARCELS = 36
SHUFFLE = True                 # set False to feed 0,1,2,... in order
SEED = None                    # set an int for a repeatable demo order
DEFECT_IDS = {3, 9, 17, 22, 28, 34}


class PartSpawner(Node):
    def __init__(self):
        super().__init__("part_spawner")
        self.order = list(range(NUM_PARCELS))
        if SHUFFLE:
            random.Random(SEED).shuffle(self.order)
        self.idx = -1              # position in the shuffled order
        self.saw_busy = False
        self.prev_done = False
        self.started = False

        self.cli = self.create_client(SetEntityPose, "/world/cell_world/set_pose")
        self.active_pub = self.create_publisher(String, "/cell/active_part", 10)
        self.create_subscription(Bool, "/cell/robot_busy", self.on_busy, 10)
        self.create_subscription(Bool, "/cell/robot_done", self.on_done, 10)
        self.create_timer(0.5, self.startup)
        self.get_logger().info(
            f"Feeder ready | {'shuffled' if SHUFFLE else 'sequential'} order: "
            f"{self.order[:8]}...")

    def startup(self):
        if not self.started and self.cli.service_is_ready():
            self.idx = 0
            self.feed(self.order[self.idx])
            self.started = True

    def on_busy(self, msg):
        if msg.data:
            self.saw_busy = True

    def on_done(self, msg):
        rising = msg.data and not self.prev_done
        self.prev_done = msg.data
        if not rising or not self.saw_busy:
            return
        self.saw_busy = False
        if self.idx + 1 >= NUM_PARCELS:
            self.get_logger().info("All 36 parcels processed")
            return
        self.idx += 1
        self.feed(self.order[self.idx])

    def feed(self, n):
        if not self.cli.service_is_ready() and \
           not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("set_pose not available")
            return
        req = SetEntityPose.Request()
        req.entity = Entity(); req.entity.name = f"cube_{n}"
        req.entity.type = Entity.MODEL
        p = Pose()
        p.position.x = float(FEED[0]); p.position.y = float(FEED[1])
        p.position.z = float(FEED[2]); p.orientation.w = 1.0
        req.pose = p
        self.cli.call_async(req)
        self.active_pub.publish(String(data=f"cube_{n}"))
        if n in DEFECT_IDS:
            dest = "REJECT"
        else:
            dest = "A" if n <= 11 else ("B" if n <= 23 else "C")
        self.get_logger().info(
            f"Fed cube_{n} (label {n} -> pallet {dest})  "
            f"[{self.idx+1}/{NUM_PARCELS}]")


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
