import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer

from vision import analyze


class Perception(Node):
    def __init__(self):
        super().__init__("hri2_camera_perception")
        self.bridge = CvBridge()
        self.info = None
        self.last_stamp = None
        self.last_frame = time.monotonic()
        self.output = self.create_publisher(String, "/hri2/world_state", 10)
        self.image = self.create_publisher(Image, "/hri_camera/annotated", qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/hri_camera/camera_info", self.calibration,
                                 qos_profile_sensor_data)
        self.rgb = Subscriber(self, Image, "/hri_camera/image", qos_profile=qos_profile_sensor_data)
        self.depth = Subscriber(self, Image, "/hri_camera/depth_image", qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([self.rgb, self.depth], 5, 0.025)
        self.sync.registerCallback(self.frame)
        self.create_timer(2.0, self.watchdog)

    def calibration(self, msg):
        self.info = msg

    def watchdog(self):
        if time.monotonic() - self.last_frame > 3:
            self.get_logger().warning("No fresh RGB-D frames: motion must wait")

    def frame(self, rgb_msg, depth_msg):
        if self.info is None:
            return
        stamp = (rgb_msg.header.stamp.sec, rgb_msg.header.stamp.nanosec)
        if stamp == self.last_stamp:
            return
        self.last_stamp = stamp
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "rgb8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
            if rgb.shape[:2] != depth.shape or self.info.k[0] <= 0:
                raise ValueError("Invalid RGB-D calibration or image sizes")
            state, annotated = analyze(rgb, depth, self.info.k)
            state["stamp"] = list(stamp)
            self.last_frame = time.monotonic()
            self.output.publish(String(data=json.dumps(state)))
            image = self.bridge.cv2_to_imgmsg(annotated, "rgb8")
            image.header = rgb_msg.header
            self.image.publish(image)
        except Exception as exc:
            self.get_logger().error(f"Camera processing failed: {exc}")


def main():
    rclpy.init()
    node = Perception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
