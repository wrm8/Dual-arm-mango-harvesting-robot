"""Image, folder, video and USB camera runner."""
import json
from pathlib import Path

IMAGE_SUFFIXES = {'.jpg', '.jpeg', '.png', '.bmp', '.webp', '.tif', '.tiff'}


def run_rgb(model, args, cfg, gate):
    import cv2
    import numpy as np
    from decision import decide
    from vision import detect, annotate, save_observation

    output = args.output / 'latest.json'
    source = Path(args.source)
    if source.is_dir() or source.suffix.lower() in IMAGE_SUFFIXES:
        paths = sorted(p for p in source.iterdir() if p.suffix.lower() in IMAGE_SUFFIXES) if source.is_dir() else [source]
        if not paths:
            raise ValueError(f'No images found: {source}')
        for frame, path in enumerate(paths, 1):
            bgr = cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                raise ValueError(f'Cannot read image: {path}')
            fruits, overlay = detect(model, bgr, cfg, args.imgsz, args.device)
            result = decide(fruits, bgr.shape[1], cfg)
            annotate(overlay, result, cfg, 0, fruits)
            save_observation(output, result, fruits, cfg, frame, False, False)
            stem = f'{frame:05d}_{path.stem}' if source.is_dir() else 'annotated'
            save_observation(args.output / f'{stem}.json', result, fruits, cfg, frame, False, False)
            ok, encoded = cv2.imencode('.jpg', overlay)
            if not ok:
                raise RuntimeError('Failed to encode annotated image')
            encoded.tofile(str(args.output / f'{stem}.jpg'))
            print(json.dumps({'source': str(path), 'decision': result}, ensure_ascii=False))
            if args.show:
                cv2.imshow('Mango - press any key', overlay)
                if cv2.waitKey(0) & 0xff == ord('q'):
                    break
            if args.max_frames and frame >= args.max_frames:
                break
        cv2.destroyAllWindows()
        return

    camera = args.source.isdecimal()
    capture = cv2.VideoCapture(int(args.source) if camera else args.source)
    writer = None
    frame = 0
    stop_reason = 'source_stopped'
    try:
        save_observation(output, {'status': 'uncertain', 'reason': 'starting', 'targets': []},
                         [], cfg, 0, False, False)
        if not capture.isOpened():
            raise RuntimeError(f'Cannot open source: {args.source}')
        if camera:
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
            capture.set(cv2.CAP_PROP_FPS, args.fps)
        fps = capture.get(cv2.CAP_PROP_FPS)
        if not np.isfinite(fps) or fps <= 0:
            fps = args.fps
        with (args.output / 'observations.jsonl').open('w', encoding='utf-8') as history:
            while True:
                ok, bgr = capture.read()
                if not ok:
                    if camera or frame == 0:
                        raise RuntimeError('Failed to read frame')
                    stop_reason = 'end_of_video'
                    break
                frame += 1
                fruits, overlay = detect(model, bgr, cfg, args.imgsz, args.device)
                result = decide(fruits, bgr.shape[1], cfg)
                stable = gate.update(result, fruits)
                annotate(overlay, result, cfg, gate.count, fruits)
                observation = save_observation(output, result, fruits, cfg, frame, stable, False)
                history.write(json.dumps(observation, ensure_ascii=False, allow_nan=False) + '\n')
                history.flush()
                if args.save_video:
                    if writer is None:
                        writer = cv2.VideoWriter(str(args.output / 'annotated.mp4'),
                                                 cv2.VideoWriter_fourcc(*'mp4v'), fps,
                                                 (bgr.shape[1], bgr.shape[0]))
                        if not writer.isOpened():
                            raise RuntimeError('Cannot create video writer')
                    writer.write(overlay)
                if args.show:
                    cv2.imshow('Mango - Q to quit', overlay)
                    if cv2.waitKey(1) & 0xff == ord('q'):
                        break
                if args.max_frames and frame >= args.max_frames:
                    break
    except KeyboardInterrupt:
        pass
    except Exception:
        stop_reason = 'source_error'
        raise
    finally:
        try:
            save_observation(output, {'status': 'uncertain', 'reason': stop_reason, 'targets': []},
                             [], cfg, frame, False, False)
        finally:
            capture.release()
            if writer is not None:
                writer.release()
            cv2.destroyAllWindows()