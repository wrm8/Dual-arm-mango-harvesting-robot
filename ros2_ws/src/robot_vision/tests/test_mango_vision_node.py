"""Exercise the real frame/stop handshake without a camera, GPU or moving base."""
import copy
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('rclpy')
pytest.importorskip('robot_arm_interfaces.msg')
from rclpy.parameter import Parameter
from rclpy.time import Time
from robot_arm_interfaces.msg import VisionStopState

ROOT = Path(os.environ.get('MANGO_TEST_SOURCE_ROOT', Path(__file__).resolve().parents[2])) / 'robot_vision'
sys.path[:0] = [str(ROOT / 'scripts'), str(ROOT / 'yolo_mango')]
import mango_vision_node as vision_node
from decision import Fruit
from stability import StabilityGate


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))


class FrameHarness:
    process_frame = vision_node.MangoVisionNode.process_frame
    on_stop_state = vision_node.MangoVisionNode.on_stop_state
    holding = vision_node.MangoVisionNode.holding
    on_parameters = vision_node.MangoVisionNode.on_parameters
    publish_invalid = vision_node.MangoVisionNode.publish_invalid
    validate_live_settings = staticmethod(vision_node.MangoVisionNode.validate_live_settings)

    def __init__(self):
        self.seconds = 100.0
        self.settings = dict(
            publish_targets=True, publish_legacy_target=False, source='realsense',
            target_frame='base_link', harvest_min_depth_m=.4, harvest_max_depth_m=.6,
            selection_stable_frames=3, tree_center_x_ratio=.5,
            stop_state_timeout_s=.6,
            min_target_confidence=.75, rotate='none', imgsz=640, device='0')
        self.cfg = json.loads((ROOT / 'yolo_mango/configs/vision.json').read_text())
        self.cfg.update(selection_mode='per_side', target_policy='green_or_small',
                        tie_relative_tolerance=.05, stable_frames=3)
        self.gate = StabilityGate(self.cfg)
        self.matrix = np.eye(4)
        self.pipeline = self.capture = None
        self.image = np.zeros((480, 640, 3), dtype=np.uint8)
        self.model = None
        self.stop_state = None
        self.stop_state_received = 0.0
        self.holding_since_ns = 0
        self.snapshot = None
        self.next_track_id = self.frame = self.error_count = 0
        self.errors = []
        for name in ('target_pub', 'scene_pub', 'harvest_pub', 'observation_pub'):
            setattr(self, name, Publisher())

    def get_clock(self):
        return SimpleNamespace(now=lambda: Time(nanoseconds=int(self.seconds * 1e9)))

    def get_logger(self):
        return SimpleNamespace(info=lambda _: None, error=self.errors.append)

    def state(self, state=VisionStopState.HOLDING, stage=4, stop_id=0):
        message = VisionStopState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.state, message.stage_id = state, stage
        message.stop_id = stop_id
        self.on_stop_state(message)


@pytest.fixture
def frames(monkeypatch):
    harness = FrameHarness()
    fruits = [Fruit(i + 1, color, x, y, float(h), .95, .5,
                    surface_xyz_m=[x / 1000, y / 1000, .5],
                    width_px=40., width_m=.04, height_m=.08)
              for i, (color, x, y, h) in enumerate([
                  ('green_mango', 120, 100, 100), ('green_mango', 130, 250, 70),
                  ('yellow_mango', 450, 100, 100), ('yellow_mango', 460, 250, 70)])]
    harness.fruits = fruits
    monkeypatch.setattr(vision_node, 'detect', lambda *a, **k: (harness.fruits, None))
    monkeypatch.setattr(vision_node, 'tree_observation', lambda *a, **k: None)
    monkeypatch.setattr(vision_node.time, 'monotonic', lambda: harness.seconds)
    return harness


@pytest.mark.parametrize('stage', [4, 6, 8, 10, 12, 14])
def test_new_parking_session_replaces_previous_snapshot_in_same_route_step(frames, stage):
    frames.state(stage=stage, stop_id=1001)
    for _ in range(3):
        frames.process_frame()
    assert frames.snapshot.stop_id == 1001
    first_ids = {target.track_id for target in frames.snapshot.targets}
    frames.seconds += .1
    frames.state(stage=stage, stop_id=1002)
    assert frames.snapshot is None
    for _ in range(2):
        frames.process_frame()
        assert not frames.harvest_pub.messages[-1].valid
    frames.process_frame()
    assert frames.snapshot.stop_id == 1002
    assert first_ids.isdisjoint({target.track_id for target in frames.snapshot.targets})


def test_only_fresh_frames_after_hold_form_one_frozen_multi_target_snapshot(frames):
    for _ in range(4):
        frames.process_frame()
    assert not any(m.valid for m in frames.harvest_pub.messages)
    frames.state()
    for _ in range(2):
        frames.process_frame()
        assert not frames.harvest_pub.messages[-1].valid
    frames.process_frame()
    snapshot = frames.harvest_pub.messages[-1]
    assert snapshot.valid and snapshot.stage_id == 4 and len(snapshot.targets) == 3, frames.errors
    assert len({t.track_id for t in snapshot.targets}) == 3
    assert any(t.is_small and t.is_green for t in snapshot.targets)
    frames.seconds += .1
    frames.process_frame()
    assert frames.harvest_pub.messages[-1] == snapshot
    assert not any(m.valid for m in frames.target_pub.messages)


def test_stale_or_revoked_hold_discards_snapshot_and_requires_new_frames(frames):
    frames.state()
    for _ in range(3):
        frames.process_frame()
    assert frames.snapshot is not None, frames.errors
    frames.seconds += .7
    frames.process_frame()
    assert frames.snapshot is None and not frames.harvest_pub.messages[-1].valid
    frames.state()
    frames.process_frame()
    assert not frames.harvest_pub.messages[-1].valid
    frames.state(VisionStopState.SETTLING)
    frames.process_frame()
    assert frames.snapshot is None and not frames.harvest_pub.messages[-1].valid


def test_depth_change_validates_atomically_and_invalidates_old_selection(frames):
    frames.state()
    for _ in range(3):
        frames.process_frame()
    assert frames.snapshot is not None, frames.errors
    result = frames.on_parameters([Parameter('harvest_min_depth_m', value=.7)])
    assert not result.successful and frames.settings['harvest_min_depth_m'] == .4
    result = frames.on_parameters([Parameter('harvest_min_depth_m', value=.51)])
    assert result.successful and frames.snapshot is None
    for _ in range(4):
        frames.process_frame()
    assert not frames.harvest_pub.messages[-1].valid


def test_missing_one_of_four_fruits_never_publishes_valid_snapshot(frames):
    frames.state()
    frames.fruits.pop()
    for _ in range(5):
        frames.process_frame()
    assert not any(m.valid for m in frames.harvest_pub.messages)


@pytest.mark.parametrize('rotation,width,trunk_x', [('none', 640, 420), ('cw', 480, 340)])
def test_depth_navigation_does_not_require_arm_calibration(frames, monkeypatch,
                                                         rotation, width, trunk_x):
    from harvest_observation import tree_observation

    frames.matrix = None
    frames.settings.update(publish_targets=False, rotate=rotation)
    color = SimpleNamespace(get_data=lambda: frames.image)
    depth = SimpleNamespace(
        get_data=lambda: np.full((480, 640), 500, dtype=np.uint16),
        profile=SimpleNamespace(as_video_stream_profile=lambda: SimpleNamespace(intrinsics=None)))
    camera_frames = SimpleNamespace(get_color_frame=lambda: color, get_depth_frame=lambda: depth)
    frames.pipeline = SimpleNamespace(wait_for_frames=lambda timeout: camera_frames)
    frames.align = SimpleNamespace(process=lambda received: received)
    frames.depth_scale = .001
    frames.rs = SimpleNamespace()

    def detected(*args, **kwargs):
        kwargs['trunks'].append(dict(cx=trunk_x, cy=200, depth_m=.5, confidence=.95))
        return frames.fruits, None

    monkeypatch.setattr(vision_node, 'detect', detected)
    monkeypatch.setattr(vision_node, 'tree_observation', tree_observation)
    frames.process_frame()
    scene = frames.scene_pub.messages[-1]
    assert frames.error_count == 0
    assert scene.mango_in_range and scene.tree_valid
    assert scene.header.frame_id == 'camera_color_optical_frame'
    assert scene.tree_center_error_ratio == pytest.approx(100 / width)
    assert scene.tree_lateral_error_m == 0
    assert not frames.target_pub.messages[-1].valid
    assert not frames.harvest_pub.messages[-1].valid
    # Once approaching a trunk, the mango may leave the view before centering.
    frames.fruits = []
    frames.process_frame()
    assert not frames.scene_pub.messages[-1].mango_in_range
    assert frames.scene_pub.messages[-1].tree_valid


def test_rgb_without_depth_cannot_enable_visual_motion(frames):
    frames.matrix = None
    frames.process_frame()
    assert not frames.scene_pub.messages[-1].mango_in_range
    assert not frames.scene_pub.messages[-1].tree_valid
