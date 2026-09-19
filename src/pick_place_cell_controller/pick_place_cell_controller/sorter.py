#!/usr/bin/env python3
"""
sorter.py -- decides which station each parcel goes to, from its label.

The camera sees several parcels at once (on the belt, in the magazine, already
stacked), so a pixel window alone is not enough to tell which one is being
picked. This node therefore gates on the cell's own sensor:

  * GATE      -- labels are only considered while PartAtPick is TRUE, i.e. a
                 parcel is actually sitting at the pick station.
  * NEAREST   -- among labels seen in that window, the one closest to the
                 image centre (the pick point) wins.
  * LATCH     -- when the robot takes the parcel (robot_busy rises) the
                 destination is frozen for the whole cycle.
  * NO LABEL  -- if nothing readable was seen while the parcel waited, the
                 parcel is routed to REJECT.

Publishes A, B, C or REJECT on /cell/destination.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

PICK_PX, PICK_PY = 320, 240      # pick station in image coordinates
HALF = 180                       # generous window; the gate does the real work


def route(label_id):
    if label_id is None:      return "REJECT"
    if 0 <= label_id <= 11:   return "A"
    if 12 <= label_id <= 23:  return "B"
    if 24 <= label_id <= 35:  return "C"
    return "REJECT"


class Sorter(Node):
    def __init__(self):
        super().__init__("sorter")
        self.part_at_pick = False
        self.candidate = None          # best label seen while gated
        self.best_dist = None
        self.locked = None
        self.prev_busy = False
        self.prev_done = False

        self.dest_pub = self.create_publisher(String, "/cell/destination", 10)
        self.create_subscription(String, "/cell/detected_label", self.on_label, 10)
        self.create_subscription(Bool, "/cell/part_at_pick", self.on_part, 10)
        self.create_subscription(Bool, "/cell/robot_busy", self.on_busy, 10)
        self.create_subscription(Bool, "/cell/robot_done", self.on_done, 10)
        self.create_timer(0.2, self.publish_dest)
        self.get_logger().info("Sorter ready (gated on PartAtPick)")

    def on_part(self, msg):
        # a new parcel has arrived -> start looking for its label
        if msg.data and not self.part_at_pick:
            self.candidate = None
            self.best_dist = None
        self.part_at_pick = msg.data

    def on_label(self, msg):
        if not self.part_at_pick:
            return                     # GATE: ignore unless a parcel is here
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
        dx, dy = px - PICK_PX, py - PICK_PY
        if abs(dx) > HALF or abs(dy) > HALF:
            return
        d = (dx * dx + dy * dy) ** 0.5
        # NEAREST to the pick point wins
        if self.best_dist is None or d < self.best_dist:
            self.best_dist = d
            if lid != self.candidate:
                self.candidate = lid
                self.get_logger().info(
                    f"Parcel at pick station: label={lid} -> {route(lid)} "
                    f"(offset {d:.0f} px)")

    def on_busy(self, msg):
        if msg.data and not self.prev_busy:
            self.locked = route(self.candidate)
            self.get_logger().info(
                f"LATCHED {self.locked} (label={self.candidate})"
                + ("  [no readable label]" if self.candidate is None else ""))
        self.prev_busy = msg.data

    def on_done(self, msg):
        if msg.data and not self.prev_done:
            self.locked = None
            self.candidate = None
            self.best_dist = None
        self.prev_done = msg.data

    def publish_dest(self):
        self.dest_pub.publish(
            String(data=self.locked if self.locked else route(self.candidate)))


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
