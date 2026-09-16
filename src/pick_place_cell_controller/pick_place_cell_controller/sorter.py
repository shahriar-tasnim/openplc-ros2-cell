#!/usr/bin/env python3
"""
sorter.py -- decides which pallet each parcel goes to, from its label.

Reliability measures:
  * POSITION FILTER -- the camera sees several parcels, so only detections
    whose image position is near the pick station are considered. Everything
    else (magazine parcels, parcels still travelling) is ignored.
  * LATCH -- once the robot starts handling a parcel (robot_busy rises), the
    destination is frozen for that whole cycle so it cannot change mid-move.
    It unlatches when the cycle completes (robot_done).

Publishes the current destination on /cell/destination: A, B, C or REJECT.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

# ---- image-space window around the pick station -------------------------
# camera is 640x480 looking down at the pick point, so the parcel being
# picked appears near the image centre. Tune HALF if needed.
PICK_PX, PICK_PY = 320, 240
HALF = 140          # accept detections within +/-140 px of centre

def route(label_id):
    if label_id is None:            return "REJECT"
    if 0  <= label_id <= 11:        return "A"
    if 12 <= label_id <= 23:        return "B"
    if 24 <= label_id <= 35:        return "C"
    return "REJECT"


class Sorter(Node):
    def __init__(self):
        super().__init__("sorter")
        self.candidate = None      # label seen at the pick station
        self.locked = None         # destination latched for this cycle
        self.prev_busy = False
        self.prev_done = False

        self.dest_pub = self.create_publisher(String, "/cell/destination", 10)
        self.create_subscription(String, "/cell/detected_label", self.on_label, 10)
        self.create_subscription(Bool, "/cell/robot_busy", self.on_busy, 10)
        self.create_subscription(Bool, "/cell/robot_done", self.on_done, 10)
        self.create_timer(0.2, self.publish_dest)
        self.get_logger().info(
            f"Sorter ready | pick window {PICK_PX}+/-{HALF}, {PICK_PY}+/-{HALF}")

    def on_label(self, msg):
        lid = px = py = None
        for tok in msg.data.split():
            try:
                if tok.startswith("label="): lid = int(tok.split("=")[1])
                elif tok.startswith("px="):  px = float(tok.split("=")[1])
                elif tok.startswith("py="):  py = float(tok.split("=")[1])
            except ValueError:
                return
        if lid is None or px is None or py is None:
            return
        # POSITION FILTER: ignore parcels that are not at the pick station
        if abs(px - PICK_PX) > HALF or abs(py - PICK_PY) > HALF:
            return
        if lid != self.candidate:
            self.candidate = lid
            self.get_logger().info(
                f"Parcel at pick station: label={lid} -> {route(lid)}")

    def on_busy(self, msg):
        # LATCH the destination when the robot takes the parcel
        if msg.data and not self.prev_busy:
            self.locked = route(self.candidate)
            self.get_logger().info(
                f"LATCHED destination {self.locked} (label={self.candidate})")
        self.prev_busy = msg.data

    def on_done(self, msg):
        if msg.data and not self.prev_done:
            self.locked = None           # release for the next parcel
            self.candidate = None
        self.prev_done = msg.data

    def publish_dest(self):
        dest = self.locked if self.locked else route(self.candidate)
        self.dest_pub.publish(String(data=dest))


def main(args=None):
    rclpy.init(args=args)
    node = Sorter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
