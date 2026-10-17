#!/usr/bin/env python3
"""Detect real camera images with YOLO; publish annotations and JSON detections."""
import argparse
from collections import Counter
import json
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
import torch
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parents[1]
WINDOW = 'TurtleBot 4 - YOLO camera'


class CameraDetector(Node):
    def __init__(self, args):
        super().__init__('turtlebot4_yolo_camera')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        torch.set_num_threads(args.threads)
        self.model = YOLO(str(args.model))
        self.args = args
        self.bridge = CvBridge()
        self.latest = None
        self.received = self.processed = self.detected_frames = 0
        self.counts = Counter()
        self.best = {}
        self.last_log = 0
        self.started = time.monotonic()
        self.annotated = self.create_publisher(Image, '/yolo/annotated_image', 2)
        self.detections = self.create_publisher(String, '/yolo/detections', 10)
        self.subscription = self.create_subscription(
            Image, args.topic, self.receive, qos_profile_sensor_data)
        self.timer = self.create_timer(1 / args.fps, self.detect)
        args.output.mkdir(parents=True, exist_ok=True)
        if args.display:
            cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
            cv2.resizeWindow(WINDOW, 640, 480)
        self.get_logger().info(f'YOLO ready: {args.model.name}; input {args.topic}')

    def receive(self, message):
        self.latest = message
        self.received += 1

    def detect(self):
        if self.latest is None:
            if self.args.display:
                cv2.waitKey(1)
            return
        message, self.latest = self.latest, None
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        start = time.monotonic()
        result = self.model.predict(image, imgsz=self.args.imgsz,
                                    conf=self.args.conf, device='cpu', verbose=False)[0]
        elapsed = time.monotonic() - start
        objects = []
        for box in result.boxes:
            name = result.names[int(box.cls.item())]
            confidence = float(box.conf.item())
            item = {'class': name, 'confidence': confidence,
                    'xyxy': [float(v) for v in box.xyxy[0].tolist()]}
            objects.append(item)
            self.counts[name] += 1
        self.processed += 1
        self.detected_frames += bool(objects)
        annotated = cv2.copyMakeBorder(result.plot(), 30, 0, 0, 0, cv2.BORDER_CONSTANT, value=(25, 32, 42))
        cv2.putText(annotated, f'YOLO11n | frame {self.processed} | {elapsed*1000:.0f} ms',
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, .48, (0, 255, 255), 1)
        published = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        published.header = message.header
        self.annotated.publish(published)
        payload = {'frame_id': message.header.frame_id,
                   'stamp': [message.header.stamp.sec, message.header.stamp.nanosec],
                   'wall_time': time.time(), 'objects': objects,
                   'inference_ms': elapsed * 1000}
        self.detections.publish(String(data=json.dumps(payload)))
        with (self.args.output / 'detections.jsonl').open('a') as stream:
            stream.write(json.dumps(payload) + '\n')
        for item in objects:
            name, confidence = item['class'], item['confidence']
            if confidence > self.best.get(name, 0):
                self.best[name] = confidence
                cv2.imwrite(str(self.args.output / f'best_{name.replace(" ", "_")}.jpg'), annotated)
        if time.monotonic() - self.last_log > 2:
            text = ', '.join(f"{obj['class']} {obj['confidence']:.2f}" for obj in objects)
            self.get_logger().info(f'frame={self.processed}: {text or "no detections"}')
            self.last_log = time.monotonic()
            self.save_summary()
        if self.args.display:
            cv2.imshow(WINDOW, cv2.resize(annotated, (640, 540), interpolation=cv2.INTER_LINEAR))
            cv2.waitKey(1)

    def save_summary(self):
        report = {'model': str(self.args.model), 'input_topic': self.args.topic,
                  'output_topic': '/yolo/annotated_image', 'frames_received': self.received,
                  'frames_processed': self.processed, 'frames_with_detections': self.detected_frames,
                  'class_counts': dict(self.counts), 'best_confidence': self.best,
                  'elapsed_s': time.monotonic() - self.started}
        path = self.args.output / 'summary.json'
        temp = path.with_suffix('.tmp')
        temp.write_text(json.dumps(report, indent=2) + '\n')
        temp.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/oakd/rgb/preview/image_raw')
    parser.add_argument('--model', type=Path, default=ROOT / '.runtime/weights/yolo11n.pt')
    parser.add_argument('--output', type=Path, default=ROOT / 'reports/yolo')
    parser.add_argument('--fps', type=float, default=5)
    parser.add_argument('--conf', type=float, default=.3)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--threads', type=int, default=2)
    parser.add_argument('--display', action='store_true')
    args = parser.parse_args()
    if args.fps <= 0 or not 0 < args.conf < 1 or args.threads < 1:
        parser.error('fps/threads must be positive and conf must be between 0 and 1')
    if not args.model.is_file():
        parser.error('Missing model weights; run ./run.sh setup-yolo')
    rclpy.init()
    node = CameraDetector(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_summary()
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
