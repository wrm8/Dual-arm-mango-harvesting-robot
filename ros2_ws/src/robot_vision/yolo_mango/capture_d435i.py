"""Save aligned D435i RGB and depth images with keyboard-triggered capture."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

from frame_transform import rotate_image

DEFAULT_OUTPUT = Path.home() / 'Desktop' / 'D435i_Captures'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT,
                        help='Parent folder for timestamped capture sessions (default: Desktop/D435i_Captures)')
    parser.add_argument('--count', type=int, default=100)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--serial')
    parser.add_argument('--rotate', choices=('none', 'cw', 'ccw'), default='cw',
                        help='Rotate saved/preview frames; default cw for portrait mounting')
    parser.add_argument('--show', action='store_true', help='Show preview; Space captures, Q quits (requires desktop)')
    args = parser.parse_args()
    if min(args.count, args.width, args.height, args.fps) <= 0:
        parser.error('Count, dimensions and fps must be positive')
    if not args.show:
        parser.error('Keyboard capture requires --show and a desktop preview window')
    try:
        import cv2
        import pyrealsense2 as rs
        import numpy as np
    except ImportError as exc:
        parser.error(f'{exc}. Install: python -m pip install -r requirements-realsense.txt')
    session = args.output / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    pipeline = rs.pipeline()
    config = rs.config()
    if args.serial:
        config.enable_device(args.serial)
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
    align_to_color = rs.align(rs.stream.color)
    started = False
    saved = 0
    try:
        profile = pipeline.start(config)
        started = True
        (session / 'rgb').mkdir(parents=True, exist_ok=False)
        (session / 'depth').mkdir()
        device = profile.get_device()
        depth_sensor = device.first_depth_sensor()
        metadata = {
            'camera': device.get_info(rs.camera_info.name),
            'serial': device.get_info(rs.camera_info.serial_number),
            'width': args.width, 'height': args.height, 'fps': args.fps,
            'depth_scale_m_per_unit': depth_sensor.get_depth_scale(),
            'depth_alignment': 'aligned_to_color',
            'depth_format': '16-bit unsigned raw sensor units; multiply by depth_scale_m_per_unit for meters',
            'rotation': args.rotate,
            'capture_mode': 'manual_space_key', 'purpose': 'RGB and depth capture',
        }
        (session / 'session.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
        print(f'Saving RGB and depth images to: {session}', flush=True)
        print('Press Space in the preview window to save a pair; Q or Ctrl+C stops.', flush=True)
        # Allow initial automatic exposure/white balance to settle.
        for _ in range(args.fps):
            pipeline.wait_for_frames(5000)
        with (session / 'frames.jsonl').open('w', encoding='utf-8') as manifest:
            while saved < args.count:
                frames = align_to_color.process(pipeline.wait_for_frames(5000))
                color = frames.get_color_frame()
                depth = frames.get_depth_frame()
                if not color or not depth:
                    continue
                bgr = np.asanyarray(color.get_data())
                depth_raw = np.asanyarray(depth.get_data())
                bgr = rotate_image(bgr, args.rotate)
                depth_raw = rotate_image(depth_raw, args.rotate)
                cv2.imshow('D435i RGB and depth - Space: save, Q: quit', bgr)
                key = cv2.waitKey(1) & 0xff
                if key == ord('q'):
                    break
                if key == ord(' '):
                    filename = f'{session.name}_{saved + 1:05d}.png'
                    rgb_path = session / 'rgb' / filename
                    depth_path = session / 'depth' / filename
                    if not cv2.imwrite(str(rgb_path), bgr):
                        raise RuntimeError(f'Cannot save RGB image: {rgb_path}')
                    if not cv2.imwrite(str(depth_path), depth_raw):
                        raise RuntimeError(f'Cannot save depth image: {depth_path}')
                    manifest.write(json.dumps({
                        'rgb': f'rgb/{filename}',
                        'depth': f'depth/{filename}',
                        'timestamp_unix_s': time.time(),
                    }) + '\n')
                    manifest.flush()
                    saved += 1
                    print(f'{saved}/{args.count}: saved RGB and depth pair {filename}', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if started:
                pipeline.stop()
        finally:
            if args.show:
                cv2.destroyAllWindows()
            print(f'Saved {saved} images. Folder: {session}', flush=True)


if __name__ == '__main__':
    main()
