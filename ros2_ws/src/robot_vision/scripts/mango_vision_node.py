#!/usr/bin/env python3
"""Run the original mango pipeline and publish ROS 2 observations/targets."""
from dataclasses import asdict
import json
from pathlib import Path
import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from robot_arm_interfaces.msg import MangoTarget
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

HERE = Path(__file__).resolve().parent
CORE = next(path for path in (HERE / 'yolo_mango', HERE.parent / 'yolo_mango') if path.is_dir())
sys.path.insert(0, str(CORE))
from decision import decide, validate_config
from frame_transform import rotate_image, rotated_pixel_to_source
from stability import StabilityGate
from vision import detect
from target_contract import TargetTracker, load_calibration


class MangoVisionNode(Node):
    def __init__(self):
        super().__init__('mango_vision_node')
        share = Path(get_package_share_directory('robot_vision'))
        defaults = {
            'weights': '', 'decision_config': '', 'source': 'realsense',
            'device': '0', 'imgsz': 640, 'width': 640, 'height': 480, 'fps': 30,
            'process_hz': 5.0, 'serial': '', 'rotate': 'cw',
            'publish_targets': False, 'calibration': '', 'target_frame': 'base_link',
            'min_target_confidence': 0.75, 'track_match_distance_m': 0.08}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.settings = p = {key: self.get_parameter(key).value for key in defaults}
        if min(p[key] for key in ('imgsz', 'width', 'height', 'fps', 'process_hz')) <= 0:
            raise ValueError('Image dimensions, fps and process_hz must be positive')
        if p['rotate'] not in ('none', 'cw', 'ccw'):
            raise ValueError('rotate must be none, cw or ccw')
        if not 0 <= p['min_target_confidence'] <= 1 or p['track_match_distance_m'] <= 0:
            raise ValueError('Invalid target confidence or tracking distance')
        self.cfg = json.loads(Path(p['decision_config'] or CORE / 'configs/vision.json')
                              .read_text(encoding='utf-8'))
        validate_config(self.cfg)
        self.matrix = load_calibration(p['calibration'], p['target_frame']) if p['calibration'] else None
        if p['publish_targets'] and (self.matrix is None or p['source'] != 'realsense'):
            raise ValueError('publish_targets requires calibrated RealSense depth and camera-to-base calibration')
        weights = Path(p['weights'] or share / 'models/best.pt')
        if not weights.is_file():
            raise FileNotFoundError(f'Trained weights not found: {weights}')
        import cv2
        from ultralytics import YOLO

        self.cv2 = cv2
        self.model = YOLO(str(weights))
        if self.model.task != 'segment' or self.model.names != {
                0: 'green_mango', 1: 'tree_trunk', 2: 'yellow_mango'}:
            raise ValueError('Expected three-class mango segmentation weights')
        self.gate = StabilityGate(self.cfg)
        self.tracker = TargetTracker(p['track_match_distance_m'])
        self.target_pub = self.create_publisher(MangoTarget, '/vision/mango_target', qos_profile_sensor_data)
        self.observation_pub = self.create_publisher(String, '/vision/observations', 10)
        self.pipeline = self.capture = self.image = None
        self.frame = self.error_count = 0
        try:
            self.open_source()
            self.timer = self.create_timer(1.0 / p['process_hz'], self.process_frame)
        except Exception:
            self.close_source()
            raise
        self.get_logger().info(f'Mango recognition ready: device={p["device"]}, source={p["source"]}, '
                               f'publish_targets={p["publish_targets"]}')

    def open_source(self):
        p = self.settings
        if p['source'] == 'realsense':
            import pyrealsense2 as rs

            self.rs = rs
            stream = rs.config()
            if p['serial']:
                stream.enable_device(p['serial'])
            stream.enable_stream(rs.stream.color, p['width'], p['height'], rs.format.bgr8, p['fps'])
            stream.enable_stream(rs.stream.depth, p['width'], p['height'], rs.format.z16, p['fps'])
            pipeline = rs.pipeline()
            profile = pipeline.start(stream)
            self.pipeline = pipeline
            self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
            self.align = rs.align(rs.stream.color)
        else:
            source = p['source']
            if Path(source).is_file() and Path(source).suffix.lower() in ('.png', '.jpg', '.jpeg', '.bmp'):
                self.image = self.cv2.imread(source)
                if self.image is None:
                    raise ValueError(f'Cannot decode image: {source}')
            else:
                self.capture = self.cv2.VideoCapture(int(source) if source.isdecimal() else source)
                if not self.capture.isOpened():
                    raise RuntimeError(f'Cannot open image source: {source}')

    def publish_invalid(self):
        message = MangoTarget()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.settings['target_frame']
        message.valid = False
        self.target_pub.publish(message)

    def process_frame(self):
        import numpy as np

        p = self.settings
        try:
            depth = intr = rs = pixel_to_source = None
            if self.pipeline is not None:
                frames = self.align.process(self.pipeline.wait_for_frames(1500))
                color, depth_frame = frames.get_color_frame(), frames.get_depth_frame()
                if not color or not depth_frame:
                    raise RuntimeError('Missing aligned color or depth frame')
                original = np.asanyarray(color.get_data())
                depth = rotate_image(np.asanyarray(depth_frame.get_data()).astype(np.float32)
                                     * self.depth_scale, p['rotate'])
                intr = depth_frame.profile.as_video_stream_profile().intrinsics
                height, width = original.shape[:2]
                pixel_to_source = lambda x, y: rotated_pixel_to_source(x, y, width, height, p['rotate'])
                rs = self.rs
            elif self.image is not None:
                original = self.image.copy()
            else:
                ok, original = self.capture.read()
                if not ok:
                    self.gate.reset()
                    self.publish_invalid()
                    self.timer.cancel()
                    self.close_source()
                    self.get_logger().info('Image stream ended')
                    return
            image = rotate_image(original, p['rotate']) if rs is not None else original
            fruits, _ = detect(self.model, image, self.cfg, p['imgsz'], p['device'],
                               depth, intr, rs, pixel_to_source)
            result = decide(fruits, image.shape[1], self.cfg)
            stable = self.gate.update(result, fruits)
            target = self.tracker.select(result, fruits, stable, self.matrix,
                                         p['min_target_confidence']) if p['publish_targets'] else None
            message = MangoTarget()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = p['target_frame']
            if target is not None:
                message.center.x, message.center.y, message.center.z = target['point']
                message.confidence = float(target['confidence'])
                message.track_id = target['track_id']
                message.valid = True
            self.target_pub.publish(message)
            self.frame += 1
            payload = {'frame': self.frame, 'decision': result, 'temporal_stable': stable,
                       'coordinate_frame': 'camera_color_optical_frame' if rs is not None else None,
                       'fruits': [asdict(f) for f in fruits], 'target_valid': message.valid}
            self.observation_pub.publish(String(data=json.dumps(payload, allow_nan=False)))
            if self.frame == 1:
                self.get_logger().info(f'First inference complete: fruits={len(fruits)}, status={result["status"]}')
        except Exception as exc:
            self.gate.reset()
            self.publish_invalid()
            self.error_count += 1
            if self.error_count == 1 or self.error_count % 30 == 0:
                self.get_logger().error(f'Frame rejected: {exc}')

    def close_source(self):
        if self.pipeline is not None:
            self.pipeline.stop()
            self.pipeline = None
        if self.capture is not None:
            self.capture.release()
            self.capture = None


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MangoVisionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # The terminal and ROS launch can both deliver SIGINT during cleanup.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            if rclpy.ok():
                node.publish_invalid()
            node.close_source()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
