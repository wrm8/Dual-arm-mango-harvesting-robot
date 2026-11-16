#!/usr/bin/env python3
"""Run the original mango pipeline and publish ROS 2 observations/targets."""
from dataclasses import asdict
import json
import math
from pathlib import Path
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from robot_arm_interfaces.msg import (MangoTarget, VisionRowObservation, VisionStopState,
                                      HarvestTarget, HarvestTargetArray)
from rcl_interfaces.msg import SetParametersResult
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
from harvest_observation import fruits_in_depth, tree_observation, selected_targets, selected_camera_targets


class MangoVisionNode(Node):
    def __init__(self):
        super().__init__('mango_vision_node')
        share = Path(get_package_share_directory('robot_vision'))
        defaults = {
            'weights': '', 'decision_config': '', 'source': 'realsense',
            'device': '0', 'imgsz': 640, 'width': 640, 'height': 480, 'fps': 30,
            'process_hz': 5.0, 'serial': '', 'rotate': 'cw',
            'publish_targets': False, 'calibration': '', 'target_frame': 'base_link',
            'min_target_confidence': 0.75, 'track_match_distance_m': 0.08,
            'harvest_min_depth_m': 0.40, 'harvest_max_depth_m': 0.60,
            'selection_stable_frames': 3, 'tree_center_x_ratio': 0.5,
            'stop_state_timeout_s': 0.6,
            'publish_legacy_target': False}
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
        self.validate_live_settings(p)
        self.matrix = load_calibration(p['calibration'], p['target_frame']) if p['calibration'] else None
        if p['publish_targets'] and p['source'] != 'realsense':
            raise ValueError('publish_targets requires RealSense depth')
        if p['publish_legacy_target'] and self.matrix is None:
            raise ValueError('publish_legacy_target requires a calibrated single-base transform')
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
        self.cfg['stable_frames'] = p['selection_stable_frames']
        self.tracker = TargetTracker(p['track_match_distance_m'])
        self.stop_state = None
        self.stop_state_received = 0.0
        self.holding_since_ns = 0
        self.snapshot = None
        self.next_track_id = 0
        self.target_pub = self.create_publisher(MangoTarget, '/vision/mango_target', qos_profile_sensor_data)
        self.scene_pub = self.create_publisher(VisionRowObservation, '/vision/row_observation', qos_profile_sensor_data)
        self.harvest_pub = self.create_publisher(HarvestTargetArray, '/vision/harvest_targets', 10)
        self.stop_sub = self.create_subscription(VisionStopState, '/base/vision_stop_state', self.on_stop_state, 10)
        self.add_on_set_parameters_callback(self.on_parameters)
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

    @staticmethod
    def validate_live_settings(p):
        for key in ('harvest_min_depth_m', 'harvest_max_depth_m', 'tree_center_x_ratio',
                    'stop_state_timeout_s', 'min_target_confidence'):
            if not math.isfinite(p[key]):
                raise ValueError(f'{key} must be finite')
        if not 0 < p['harvest_min_depth_m'] < p['harvest_max_depth_m']:
            raise ValueError('Require 0 < harvest_min_depth_m < harvest_max_depth_m')
        if (not 0 < p['tree_center_x_ratio'] < 1 or p['stop_state_timeout_s'] <= 0 or
                not 0 <= p['min_target_confidence'] <= 1):
            raise ValueError('Invalid centering, timeout or confidence setting')
        if type(p['selection_stable_frames']) is not int or not 1 <= p['selection_stable_frames'] <= 60:
            raise ValueError('selection_stable_frames must be an integer from 1 to 60')

    def on_parameters(self, parameters):
        dynamic = {'harvest_min_depth_m', 'harvest_max_depth_m', 'selection_stable_frames',
                   'tree_center_x_ratio', 'stop_state_timeout_s',
                   'min_target_confidence', 'publish_targets', 'publish_legacy_target'}
        candidate = dict(self.settings)
        for param in parameters:
            if param.name not in dynamic:
                return SetParametersResult(successful=False, reason=f'{param.name} requires restart')
            candidate[param.name] = param.value
        try:
            self.validate_live_settings(candidate)
            if candidate['publish_targets'] and candidate['source'] != 'realsense':
                raise ValueError('Publishing targets requires RealSense depth')
            if candidate['publish_legacy_target'] and self.matrix is None:
                raise ValueError('Publishing the legacy target requires a calibrated transform')
        except (ValueError, TypeError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        self.settings.update(candidate)
        self.cfg['stable_frames'] = candidate['selection_stable_frames']
        self.gate.reset()
        self.snapshot = None
        return SetParametersResult(successful=True)

    def on_stop_state(self, message):
        previous = self.stop_state
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        if stamp_ns > now_ns + 100000000 or now_ns - stamp_ns > self.settings['stop_state_timeout_s'] * 1e9:
            return
        entered = (message.state == VisionStopState.HOLDING and
                   (previous is None or previous.state != message.HOLDING or
                    previous.stage_id != message.stage_id or
                    previous.stop_id != message.stop_id))
        if entered or message.state != VisionStopState.HOLDING:
            self.gate.reset()
            self.snapshot = None
        if entered:
            self.holding_since_ns = now_ns
            self.get_logger().info(f'Tree stop confirmed at route step {message.stage_id}; collecting fresh frames')
        self.stop_state = message
        self.stop_state_received = time.monotonic()

    def holding(self, frame_ns):
        return (self.stop_state is not None and self.stop_state.state == VisionStopState.HOLDING and
                time.monotonic() - self.stop_state_received <= self.settings['stop_state_timeout_s'] and
                frame_ns >= self.holding_since_ns)

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
        scene = VisionRowObservation()
        scene.header.stamp = message.header.stamp
        scene.header.frame_id = 'camera_color_optical_frame'
        self.scene_pub.publish(scene)
        invalid = HarvestTargetArray()
        invalid.header = scene.header
        self.harvest_pub.publish(invalid)
        self.snapshot = None

    def process_frame(self):
        import numpy as np

        p = self.settings
        try:
            # Include frame retrieval and inference in the age seen by navigation.
            acquired = self.get_clock().now()
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
            trunks = []
            fruits, _ = detect(self.model, image, self.cfg, p['imgsz'], p['device'],
                               depth, intr, rs, pixel_to_source, trunks=trunks)
            nearby = fruits_in_depth(fruits, p['harvest_min_depth_m'], p['harvest_max_depth_m'])
            # Navigation uses displayed image error, independently of the arm transform.
            # Mangoes and trunks share the same absolute RealSense depth interval.
            tree = tree_observation(trunks, nearby, image.shape[1], p['tree_center_x_ratio'],
                                    p['harvest_min_depth_m'], p['harvest_max_depth_m']) if rs else None
            scene = VisionRowObservation()
            scene.header.stamp = acquired.to_msg()
            scene.header.frame_id = 'camera_color_optical_frame'
            scene.frame_sequence = (self.frame + 1) % 4294967296
            scene.mango_in_range = bool(nearby) and rs is not None
            scene.mango_count_in_range = len(nearby)
            scene.nearest_mango_depth_m = min((f.depth_m for f in nearby), default=0.0)
            if tree is not None:
                scene.tree_valid = True
                scene.tree_center_error_ratio = tree['error_ratio']
                scene.tree_confidence = float(tree['confidence'])
            self.scene_pub.publish(scene)
            result = decide(nearby, image.shape[1], self.cfg)
            holding = self.holding(acquired.nanoseconds)
            if not holding:
                self.gate.reset()
                self.snapshot = None
            stable = self.gate.update(result, nearby) if holding else False
            target = self.tracker.select(result, fruits, stable, self.matrix,
                                         p['min_target_confidence']) if p['publish_targets'] and p['publish_legacy_target'] else None
            message = MangoTarget()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = p['target_frame']
            if target is not None:
                message.center.x, message.center.y, message.center.z = target['point']
                message.confidence = float(target['confidence'])
                message.track_id = target['track_id']
                message.valid = True
            self.target_pub.publish(message)
            if holding and stable and p['publish_targets'] and self.snapshot is None:
                selected = selected_camera_targets(result, nearby, p['min_target_confidence'])
                if selected is not None:
                    self.snapshot = HarvestTargetArray()
                    self.snapshot.header.stamp = acquired.to_msg()
                    self.snapshot.header.frame_id = 'camera_color_optical_frame'
                    self.snapshot.stage_id = self.stop_state.stage_id
                    self.snapshot.stop_id = self.stop_state.stop_id
                    self.snapshot.valid = True
                    for item in selected:
                        fruit = HarvestTarget()
                        self.next_track_id = self.next_track_id % 4294967295 + 1
                        fruit.track_id = self.next_track_id
                        fruit.surface_point.x, fruit.surface_point.y, fruit.surface_point.z = item['point']
                        for key in ('slot', 'color', 'is_small', 'is_green', 'confidence', 'depth_m',
                                    'width_px', 'height_px', 'width_m', 'height_m'):
                            setattr(fruit, key, item[key])
                        self.snapshot.targets.append(fruit)
                    self.get_logger().info(f'Harvest snapshot ready: route step={self.stop_state.stage_id}, targets={len(selected)}')
            if self.snapshot is not None and holding:
                self.harvest_pub.publish(self.snapshot)
            else:
                empty = HarvestTargetArray()
                empty.header.stamp = acquired.to_msg()
                empty.header.frame_id = 'camera_color_optical_frame'
                self.harvest_pub.publish(empty)
            self.frame += 1
            payload = {'frame': self.frame, 'decision': result, 'temporal_stable': stable,
                       'coordinate_frame': 'camera_color_optical_frame' if rs is not None else None,
                       'fruits': [asdict(f) for f in fruits], 'trunks': trunks,
                       'in_range_count': len(nearby), 'tree_alignment': tree,
                       'base_holding': holding,
                       'harvest_snapshot_valid': self.snapshot is not None,
                       'target_valid': message.valid}
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
