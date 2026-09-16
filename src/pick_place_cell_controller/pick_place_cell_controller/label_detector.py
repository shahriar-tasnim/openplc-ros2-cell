#!/usr/bin/env python3
"""
label_detector.py -- parcel label detection for the sorting cell.

Subscribes to the overhead camera, detects the ArUco marker (the parcel's
shipping label), and publishes the label ID plus its image position on
/cell/detected_label.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import cv2

CAMERA_TOPIC = "/cell/camera/image"
DETECT_TOPIC = "/cell/detected_label"


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
        for mc, mid in zip(corners, ids.flatten()):
            c = mc[0]
            cx, cy = float(c[:, 0].mean()), float(c[:, 1].mean())
            out = String()
            out.data = f"label={int(mid)} px={cx:.0f} py={cy:.0f}"
            self.pub.publish(out)
            if mid != self.last_id:
                self.get_logger().info(f"Detected {out.data}")
                self.last_id = int(mid)
            break


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
