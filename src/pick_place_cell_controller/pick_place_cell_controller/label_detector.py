#!/usr/bin/env python3
"""
label_detector.py -- detect the ArUco label nearest the pick point.

The overhead camera can see several labels at once.  OpenCV does not promise
that ids[0] is the parcel at the pick station, so choosing the first detection
can route the wrong parcel.  This node evaluates every visible marker and
publishes only the marker nearest the image centre (the calibrated pick area).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import cv2

CAMERA_TOPIC = "/cell/camera/image"
DETECT_TOPIC = "/cell/detected_label"
PICK_PX, PICK_PY = 320.0, 240.0


class LabelDetector(Node):
    def __init__(self):
        super().__init__("label_detector")
        self.adict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(
            self.adict, cv2.aruco.DetectorParameters())
        self.create_subscription(Image, CAMERA_TOPIC, self.on_image, 10)
        self.pub = self.create_publisher(String, DETECT_TOPIC, 10)
        self.last_id = None
        self.frames = 0
        self.create_timer(5.0, self.heartbeat)
        self.get_logger().info(
            f"Label detector ready | listening on {CAMERA_TOPIC}")

    def heartbeat(self):
        self.get_logger().info(f"frames received: {self.frames}")

    def on_image(self, msg):
        self.frames += 1
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, -1)
            if msg.encoding in ("rgb8", "rgba8"):
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
        except Exception as exc:
            self.get_logger().warn(f"image convert failed: {exc}")
            return

        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return

        best = None
        for mc, mid in zip(corners, ids.flatten()):
            c = mc[0]
            cx, cy = float(c[:, 0].mean()), float(c[:, 1].mean())
            d2 = (cx - PICK_PX) ** 2 + (cy - PICK_PY) ** 2
            if best is None or d2 < best[0]:
                best = (d2, int(mid), cx, cy)

        if best is None:
            return

        _, mid, cx, cy = best
        out = String()
        out.data = f"label={mid} px={cx:.0f} py={cy:.0f}"
        self.pub.publish(out)
        if mid != self.last_id:
            self.get_logger().info(f"Detected nearest marker: {out.data}")
            self.last_id = mid


def main(args=None):
    rclpy.init(args=args)
    node = LabelDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
