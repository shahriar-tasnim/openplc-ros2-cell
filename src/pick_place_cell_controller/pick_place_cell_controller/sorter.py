#!/usr/bin/env python3
"""
sorter.py -- classify the parcel at the pick station from its ArUco ID.

All parcels are visually identical.  The ArUco ID alone defines the class:
  * A      : IDs 0..11 except IDs reserved for DEFECT
  * B      : IDs 12..23 except IDs reserved for DEFECT
  * C      : IDs 24..35 except IDs reserved for DEFECT
  * REJECT : explicit DEFECT_IDS (or an invalid/unreadable ID)

The defect IDs are the same six parcel numbers that the previous revision
made grey/unlabelled.  They now keep their normal orange body and ArUco label,
so defect classification is ID-based instead of appearance-based.

A decision is accepted only while PartAtPick is TRUE.  /cell/sort_decision is
an event message for the current parcel and is used by the robot node as a
start interlock, eliminating the race where RobotStart could arrive before the
new destination had been published.
"""
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

PICK_PX, PICK_PY = 320, 240
HALF = 180

# These were the six parcels intentionally made grey/unlabelled by
# make_unlabelled.py.  Keep them as the defect population, but classify them
# by their ArUco IDs instead.
DEFECT_IDS = {3, 9, 17, 22, 28, 34}


def route(label_id):
    if label_id is None:
        return "REJECT"
    if label_id in DEFECT_IDS:
        return "REJECT"
    if 0 <= label_id <= 11:
        return "A"
    if 12 <= label_id <= 23:
        return "B"
    if 24 <= label_id <= 35:
        return "C"
    return "REJECT"


class Sorter(Node):
    def __init__(self):
        super().__init__("sorter")
        self.part_at_pick = False
        self.candidate = None
        self.pending_label = None
        self.pending_hits = 0
        self.decision_made = False
        self.locked = None
        self.prev_busy = False
        self.prev_done = False
        self.active_label = None
        self.part_arrived_at = None
        self.current_decision = None

        self.dest_pub = self.create_publisher(String, "/cell/destination", 10)
        self.decision_pub = self.create_publisher(String, "/cell/sort_decision", 10)
        self.create_subscription(String, "/cell/detected_label", self.on_label, 10)
        self.create_subscription(String, "/cell/active_part", self.on_active, 10)
        self.create_subscription(Bool, "/cell/part_at_pick", self.on_part, 10)
        self.create_subscription(Bool, "/cell/robot_busy", self.on_busy, 10)
        self.create_subscription(Bool, "/cell/robot_done", self.on_done, 10)
        self.create_timer(0.2, self.publish_dest)
        self.create_timer(0.05, self.check_vision_timeout)
        self.get_logger().info(
            "Sorter ready | ID-based A/B/C/DEFECT | "
            f"defect IDs={sorted(DEFECT_IDS)}")

    def on_active(self, msg):
        # In simulation cube_N and ArUco ID N are intentionally paired.
        # Keep this only as a watchdog/fallback; vision remains the primary path.
        try:
            self.active_label = int(msg.data.rsplit("_", 1)[1])
        except (ValueError, IndexError):
            self.active_label = None

    def on_part(self, msg):
        # Rising edge = a new parcel is physically at the pick station.
        if msg.data and not self.part_at_pick:
            self.candidate = None
            self.pending_label = None
            self.pending_hits = 0
            self.decision_made = False
            self.locked = None
            self.current_decision = None
            self.part_arrived_at = time.monotonic()
            self.get_logger().info(
                f"New parcel at pick; waiting for ArUco ID "
                f"(expected={self.active_label})")
        elif not msg.data:
            self.part_arrived_at = None
        self.part_at_pick = msg.data

    def on_label(self, msg):
        if not self.part_at_pick or self.locked is not None or self.decision_made:
            return

        lid = px = py = None
        for tok in msg.data.split():
            try:
                if tok.startswith("label="):
                    lid = int(tok.split("=", 1)[1])
                elif tok.startswith("px="):
                    px = float(tok.split("=", 1)[1])
                elif tok.startswith("py="):
                    py = float(tok.split("=", 1)[1])
            except ValueError:
                return

        if lid is None or px is None or py is None:
            return

        # Because the feeder tells us which unique simulated parcel is active,
        # ignore other visible labels (magazine/placed parcels) and wait for
        # the ArUco ID belonging to the parcel at the pick station.
        if self.active_label is not None and lid != self.active_label:
            return

        # Require the same marker twice in succession.
        if lid == self.pending_label:
            self.pending_hits += 1
        else:
            self.pending_label = lid
            self.pending_hits = 1
        if self.pending_hits < 2:
            return

        self.commit_decision(lid, source=f"vision px={px:.0f} py={py:.0f}")

    def commit_decision(self, lid, source):
        if self.decision_made:
            return
        self.candidate = lid
        self.decision_made = True
        dest = route(lid)
        self.current_decision = f"label={lid} dest={dest}"
        self.dest_pub.publish(String(data=dest))
        self.decision_pub.publish(String(data=self.current_decision))
        cls = "DEFECT" if dest == "REJECT" else dest
        self.get_logger().info(
            f"SORT DECISION: ArUco {lid} -> {cls} [{source}]")

    def check_vision_timeout(self):
        # A missed camera detection must not deadlock the PLC/robot handshake.
        # After 1 s, use the known active simulated parcel ID as a deterministic
        # fallback.  This is still ID-based routing; a warning makes it obvious
        # that vision did not confirm that cycle.
        if (not self.part_at_pick or self.decision_made or
                self.part_arrived_at is None or self.active_label is None):
            return
        if time.monotonic() - self.part_arrived_at >= 1.0:
            self.get_logger().warn(
                f"No ArUco confirmation within 1.0 s; "
                f"using active parcel ID {self.active_label}")
            self.commit_decision(self.active_label, source="active-part fallback")

    def on_busy(self, msg):
        if msg.data and not self.prev_busy:
            self.locked = route(self.candidate)
            self.get_logger().info(
                f"LATCHED {self.locked} for label={self.candidate}")
        self.prev_busy = msg.data

    def on_done(self, msg):
        if msg.data and not self.prev_done:
            self.locked = None
            self.candidate = None
            self.pending_label = None
            self.pending_hits = 0
            self.decision_made = False
            self.current_decision = None
            self.part_arrived_at = None
        self.prev_done = msg.data

    def publish_dest(self):
        # Preserve the existing destination topic for the grasp manager and
        # diagnostics.  Until a valid current ID is seen, it remains REJECT,
        # but the robot cannot start because no /cell/sort_decision event has
        # been issued yet.
        dest = self.locked if self.locked else route(self.candidate)
        self.dest_pub.publish(String(data=dest))

        # Keep the current decision alive until RobotBusy acknowledges start.
        # This closes the reset/start race: a RobotReset between detection and
        # RobotStart can no longer permanently erase the route.
        if self.current_decision is not None and self.locked is None:
            self.decision_pub.publish(String(data=self.current_decision))


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
