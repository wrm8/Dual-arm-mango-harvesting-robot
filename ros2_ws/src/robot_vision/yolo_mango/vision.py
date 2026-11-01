"""Images, RGB streams and RealSense observation. Never sends robot commands."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time

from decision import Fruit, decide, validate_config
from stability import StabilityGate
from rgb_source import run_rgb
from frame_transform import rotated_pixel_to_source, rotate_image

ROOT = Path(__file__).resolve().parent


def detect(model, bgr, cfg, imgsz, device, depth=None, intr=None, rs=None,
           pixel_to_source=None):
    import cv2
    import numpy as np
    prediction = model.predict(bgr, conf=cfg['confidence'], imgsz=imgsz,
                               device=device, retina_masks=True, agnostic_nms=True, verbose=False)[0]
    fruits = []
    overlay = prediction.plot()
    if prediction.masks is None:
        return fruits, overlay
    if len(prediction.masks.data) != len(prediction.boxes):
        raise RuntimeError('Mismatch between masks and boxes')
    masks = prediction.masks.data.cpu().numpy()
    for i, box in enumerate(prediction.boxes):
        class_name = model.names[int(box.cls.item())]
        if class_name == 'tree_trunk':
            continue
        mask = (masks[i] > 0.5).astype('uint8')
        if mask.shape != bgr.shape[:2]:
            raise RuntimeError('Expected masks in original image dimensions')
        ys, xs = np.nonzero(mask)
        if not len(xs):
            continue
        x1, y1, x2, y2 = box.xyxy[0].cpu().tolist()
        f = Fruit(i, class_name, (x1+x2)/2, (y1+y2)/2,
                  y2-y1, float(box.conf.item()))
        if depth is not None:
            inner = cv2.erode(mask, np.ones((5, 5), dtype='uint8'))
            valid = ((inner > 0) & np.isfinite(depth) &
                     (depth >= cfg['depth_min_m']) & (depth <= cfg['depth_max_m']))
            py, px = np.nonzero(valid)
            if len(px) >= cfg['depth_min_pixels']:
                values = depth[py, px]
                median = float(np.median(values))
                mad = float(np.median(np.abs(values-median)))
                keep = np.abs(values-median) <= max(3 * mad, 0.005)
                if mad <= cfg['depth_mad_max_m'] and int(keep.sum()) >= cfg['depth_min_pixels']:
                    # Representative surface point: actual valid pixel near mask centroid.
                    py, px, values = py[keep], px[keep], values[keep]
                    k = int(np.argmin((px-xs.mean())**2 + (py-ys.mean())**2))
                    f.depth_m = float(np.median(values))
                    pixel = pixel_to_source(float(px[k]), float(py[k])) if pixel_to_source else (
                        float(px[k]), float(py[k]))
                    f.surface_xyz_m = list(rs.rs2_deproject_pixel_to_point(
                        intr, pixel, float(values[k])))
                    f.range_m = float(sum(value * value for value in f.surface_xyz_m) ** 0.5)
        fruits.append(f)
        cv2.putText(overlay, f'id={i} h={f.height_px:.1f}', (int(x1), max(15, int(y1)-10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return fruits, overlay


def save_observation(path, result, fruits, cfg, frame, stable, camera,
                     camera_to_robot=None, camera_intrinsics=None):
    fruit_payload = []
    robot_by_id = {}
    range_by_id = {}
    for fruit in fruits:
        payload = asdict(fruit)
        range_by_id[fruit.id] = fruit.range_m
        if camera_to_robot is not None and fruit.surface_xyz_m is not None:
            robot_xyz = transform_point(camera_to_robot, fruit.surface_xyz_m)
            payload['robot_xyz_m'] = robot_xyz
            robot_by_id[fruit.id] = robot_xyz
        fruit_payload.append(payload)
    decision = json.loads(json.dumps(result))
    for target in decision.get('targets', []):
        if target.get('id') in robot_by_id:
            target['robot_xyz_m'] = robot_by_id[target['id']]
        if target.get('id') in range_by_id:
            target['range_m'] = range_by_id[target['id']]
    observation = {
        'schema_version': 2, 'timestamp_unix_s': time.time(), 'frame': frame,
        'coordinate_frame': 'camera_color_optical' if camera else None,
        'axis_convention': 'x right, y down, z forward; meters' if camera else None,
        'robot_coordinate_frame': 'robot_base' if camera_to_robot is not None else None,
        'camera_to_robot_transform': camera_to_robot,
        'camera_intrinsics': camera_intrinsics,
        'height_mode': cfg['height_mode'], 'decision': decision,
        'temporal_stable': stable,
        'target_surface_positions_available': bool(result['targets']) and all(
            t['surface_xyz_m'] is not None for t in result['targets']),
        'robot_command_ready': camera_to_robot is not None and bool(robot_by_id),
        'fruits': fruit_payload
    }
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(observation, ensure_ascii=False, indent=2, allow_nan=False),
                         encoding='utf-8')
    temporary.replace(path)
    return observation


def transform_point(matrix, point):
    """Apply a 4x4 camera-to-robot homogeneous transform to an XYZ point."""
    import numpy as np

    values = np.asarray(matrix, dtype=float)
    if values.shape != (4, 4):
        raise ValueError('camera_to_robot must be a 4x4 matrix')
    xyz = values @ np.array([point[0], point[1], point[2], 1.0], dtype=float)
    if abs(xyz[3]) < 1e-12:
        raise ValueError('camera_to_robot produced an invalid homogeneous coordinate')
    return [float(value / xyz[3]) for value in xyz[:3]]


def load_robot_calibration(path):
    if path is None:
        return None
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    matrix = data.get('camera_to_robot') if isinstance(data, dict) else data
    if matrix is None:
        raise ValueError('Calibration JSON must contain camera_to_robot')
    transform_point(matrix, [0.0, 0.0, 0.0])
    return matrix


def annotate(image, result, cfg, count, fruits=()):
    import cv2
    center = round(image.shape[1] * cfg['center_x_ratio'])
    cv2.line(image, (center, 0), (center, image.shape[0]), (255, 255, 0), 2)
    label = f"{result['status']} {result['reason']} stable={count}"
    cv2.putText(image, label, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
    ratios = ' '.join(f'{side}={value:.3f}' for side, value in result['ratios'].items())
    for fruit in fruits:
        side = next((key for key, ids in result.get('pairs', {}).items()
                     if fruit.id in ids['ids']), '?')
        size = result.get('size_labels', {}).get(str(fruit.id), '?')
        text = f'{side} {size} id={fruit.id}'
        cv2.putText(image, text, (max(0, int(fruit.cx)-60), max(65, int(fruit.cy))),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 0, 255) if size == 'small' else (255, 255, 255), 2)
        if fruit.surface_xyz_m is not None:
            x, y, z = fruit.surface_xyz_m
            range_text = f' r={fruit.range_m:.2f}m' if fruit.range_m is not None else ''
            cv2.putText(image, f'xyz {x:.2f},{y:.2f},{z:.2f}m{range_text}',
                        (max(0, int(fruit.cx)-60), max(85, int(fruit.cy)+22)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)
    ids = ','.join(str(t['id']) for t in result['targets'])
    cv2.putText(image, f'{ratios} targets={ids}', (10, 44),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', required=True, type=Path)
    parser.add_argument('--source', required=True, help='Image, image directory, video, camera index (0), or realsense')
    parser.add_argument('--config', type=Path, default=ROOT / 'configs/vision.json')
    parser.add_argument('--output', type=Path, default=ROOT / 'outputs')
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--serial', help='Optional RealSense serial number')
    parser.add_argument('--rotate', choices=('none', 'cw', 'ccw'), default='cw',
                        help='Rotate RealSense frames; default cw for portrait mounting')
    parser.add_argument('--robot-calibration', type=Path,
                        help='JSON containing a calibrated camera_to_robot 4x4 matrix')
    parser.add_argument('--show', action='store_true')
    parser.add_argument('--selection-mode', choices=('per_side', 'unique_small'))
    parser.add_argument('--height-mode', choices=('pixel', 'depth_scaled'))
    parser.add_argument('--max-frames', type=int, default=0, help='0 means unlimited')
    parser.add_argument('--save-video', action='store_true')
    parser.add_argument('--target-policy', choices=('small_only', 'green_or_small'))
    args = parser.parse_args()
    if args.max_frames < 0 or min(args.width, args.height, args.fps, args.imgsz) <= 0:
        parser.error('Frame limit must be nonnegative; dimensions and fps must be positive')

    if args.save_video and args.source == 'realsense':
        parser.error('--save-video currently supports RGB cameras and video files')
    cfg = json.loads(args.config.read_text(encoding='utf-8'))
    camera_to_robot = load_robot_calibration(args.robot_calibration)
    if args.target_policy:
        cfg['target_policy'] = args.target_policy
    if args.selection_mode:
        cfg['selection_mode'] = args.selection_mode
    if args.height_mode:
        cfg['height_mode'] = args.height_mode
    # Validate configuration before loading weights or starting hardware.
    validate_config(cfg)
    gate = StabilityGate(cfg)
    if not args.weights.is_file():
        raise SystemExit(f'Trained weights not found: {args.weights}')
    import cv2
    import numpy as np
    from ultralytics import YOLO
    model = YOLO(str(args.weights))
    if model.task != 'segment' or model.names != {
            0: 'green_mango', 1: 'tree_trunk', 2: 'yellow_mango'}:
        raise SystemExit('Expected custom segmentation weights: 0=green_mango, 1=tree_trunk, 2=yellow_mango')
    args.output.mkdir(parents=True, exist_ok=True)
    output = args.output / 'latest.json'
    if args.source != 'realsense':
        run_rgb(model, args, cfg, gate)
        return

    import pyrealsense2 as rs
    pipeline = rs.pipeline()
    stream = rs.config()
    if args.serial:
        stream.enable_device(args.serial)
    stream.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    stream.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
    started = False
    try:
        # Invalidate any previous session's observation before opening camera.
        save_observation(output, {'status': 'uncertain', 'reason': 'starting', 'targets': []},
                         [], cfg, 0, False, True, camera_to_robot)
        profile = pipeline.start(stream)
        started = True
        scale = profile.get_device().first_depth_sensor().get_depth_scale()
        align = rs.align(rs.stream.color)
        frame = 0
        while True:
            frames = align.process(pipeline.wait_for_frames())
            color, depth_frame = frames.get_color_frame(), frames.get_depth_frame()
            if not color or not depth_frame:
                gate.reset()
                save_observation(output, {'status': 'uncertain', 'reason': 'missing_frame', 'targets': []},
                                 [], cfg, frame, False, True)
                continue
            frame += 1
            source_bgr = np.asanyarray(color.get_data())
            source_depth = np.asanyarray(depth_frame.get_data()).astype(np.float32) * scale
            bgr = rotate_image(source_bgr, args.rotate)
            depth = rotate_image(source_depth, args.rotate)
            intr = depth_frame.profile.as_video_stream_profile().intrinsics
            source_height, source_width = source_bgr.shape[:2]
            pixel_to_source = lambda x, y: rotated_pixel_to_source(
                x, y, source_width, source_height, args.rotate)
            fruits, overlay = detect(model, bgr, cfg, args.imgsz, args.device, depth, intr, rs,
                                     pixel_to_source)
            result = decide(fruits, bgr.shape[1], cfg)
            stable = gate.update(result, fruits)
            intrinsics = {
                'width': intr.width, 'height': intr.height,
                'fx': intr.fx, 'fy': intr.fy, 'ppx': intr.ppx, 'ppy': intr.ppy,
                'model': str(intr.model), 'rotation': args.rotate,
            }
            save_observation(output, result, fruits, cfg, frame, stable, True,
                             camera_to_robot, intrinsics)
            if args.max_frames and frame >= args.max_frames:
                break
            if args.show:
                annotate(overlay, result, cfg, gate.count, fruits)
                cv2.imshow('Mango observation - Q to quit', overlay)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        try:
            save_observation(output, {'status': 'uncertain', 'reason': 'camera_stopped', 'targets': []},
                         [], cfg, 0, False, True, camera_to_robot)
        finally:
            try:
                if started:
                    pipeline.stop()
            finally:
                cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
