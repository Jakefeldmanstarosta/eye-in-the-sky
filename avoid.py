"""Live obstacle-avoidance path MVP.

The webcam stands in for a drone's forward camera. Each depth frame is turned into 3D points; the planner keeps the
path straight ahead unless the drone's safety tube would hit something within the reaction distance, in which case
it bends the path left/right/up/down (smallest turn), or shows STOP if there is no gap.

    python avoid.py                          # live webcam
    python avoid.py --calibrate 1.0          # calibrate distances against a wall 1.0 m away
    python avoid.py --video clip.mp4         # replay a recording
    python avoid.py --record out.mp4         # also save the annotated view
    python avoid.py --depth metric           # use the indoor metric depth model instead of relative + calibration

Keys: q/Esc quit, space pause, h status text on/off.
"""
import argparse
import os
import sys
import threading
import time

if sys.platform.startswith('linux'):
    os.environ.setdefault('QT_QPA_PLATFORM', 'xcb')  # OpenCV's Qt has no Wayland plugin; use XWayland

import cv2
import numpy as np

from eyeinthesky import viz
from eyeinthesky.depth import DepthEngine, VideoSource, WebcamSource, wait_for_frame
from eyeinthesky.planner import Planner, PlannerConfig

WINDOW = 'Eye in the Sky'


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--video', help='video file to replay instead of the webcam')
    p.add_argument('--record', help='save the annotated view to this MP4 file')
    p.add_argument('--camera', type=int, default=0, help='webcam index (default 0 = built-in camera)')
    p.add_argument('--calibrate', type=float, metavar='METRES',
                   help='calibration mode: aim the centre box at a flat surface this far away and press c')
    p.add_argument('--hfov', type=float, default=70.0, help='camera horizontal field of view in degrees')
    p.add_argument('--react-dist', type=float, default=1.5, help='react to obstacles closer than this (m)')
    p.add_argument('--drone-width', type=float, default=0.3, help='drone width (m)')
    p.add_argument('--clearance', type=float, default=0.3, help='extra margin around the drone (m)')
    p.add_argument('--input-size', type=int, default=252, help='depth model input height (multiple of 14)')
    p.add_argument('--depth', choices=['relative', 'metric'], default='metric',
                   help='relative: general model, metres via --calibrate (default). '
                        'metric: indoor model that outputs metres directly (calibration optional)')
    p.add_argument('--backend', choices=['openvino', 'torch'],
                   default='torch' if sys.platform == 'darwin' else 'openvino',
                   help='torch uses the Apple GPU (MPS) when available; openvino runs on the CPU '
                        '(default: torch on macOS, openvino elsewhere)')
    p.add_argument('--no-display', action='store_true', help='no window (use with --record)')
    p.add_argument('--duration', type=float, help='stop after this many seconds')
    return p.parse_args()


def window_closed():
    """True if the window was closed with its X button.

    OpenCV's macOS window backend doesn't report window visibility, so this check would end the program on the
    first frame there. On macOS it is skipped: quit with q or Esc instead.
    """
    if sys.platform == 'darwin':
        return False
    return cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1


def smooth_fps(fps, dt):
    """Running FPS estimate. Averages frame intervals, not 1/dt, so one tiny interval can't cause a huge spike."""
    dt = max(dt, 1e-6)
    return 1 / (0.9 / fps + 0.1 * dt) if fps else 1 / dt


class Worker:
    """Background thread: newest webcam frame -> depth -> plan. The display never waits for it."""

    def __init__(self, source, engine, planner):
        self.source, self.engine, self.planner = source, engine, planner
        self.plan, self.depth_fps, self.error = None, 0.0, None
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        last_id, last_t = 0, None
        try:
            while not self.stopped.is_set():
                frame, frame_id = self.source.read()
                if frame is None or frame_id == last_id:
                    time.sleep(0.002)
                    continue
                last_id = frame_id
                now = time.time()
                dt = now - last_t if last_t else 1 / self.source.fps
                self.plan = self.planner.update(self.engine.estimate(frame), dt)
                if last_t:  # FPS needs two frames
                    self.depth_fps = smooth_fps(self.depth_fps, dt)
                last_t = now
        except Exception as e:
            self.error = e
            self.stopped.set()

    def stop(self):
        self.stopped.set()
        self.thread.join(timeout=5)


def status_lines(plan, engine, view_fps, depth_fps):
    nearest = f'{plan.nearest:.2f} m' if plan.nearest is not None else 'none'
    lines = [f'obstacle ahead: {nearest}',
             f'yaw {plan.yaw:+.0f} deg  pitch {plan.pitch:+.0f} deg',
             f'speed {plan.speed:.0%}',
             f'view {view_fps:.0f} FPS  depth {depth_fps:.0f} FPS']
    lines.append(f'depth model: {engine.kind}')
    if not engine.calibrated and engine.kind == 'relative':
        lines.append('UNCALIBRATED: run --calibrate')
    return lines


def calibrate(args, engine, source):
    """Live view with a centre box; pressing c sets k so the box reads the given distance."""
    print(f'Aim the centre box at a flat surface (wall, door) {args.calibrate} m from the camera, then press c. '
          'q to quit.')
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    while True:
        frame = wait_for_frame(source)
        raw = engine.raw(frame)
        h, w = raw.shape
        med = float(np.median(raw[int(h * 0.4):int(h * 0.6), int(w * 0.4):int(w * 0.6)]))
        estimate = float(engine.to_metres(np.float32(med)))
        fh, fw = frame.shape[:2]
        view = frame.copy()
        cv2.rectangle(view, (int(fw * 0.4), int(fh * 0.4)), (int(fw * 0.6), int(fh * 0.6)), (0, 255, 255), 2)
        viz.draw_hud(view, f'target {args.calibrate:.2f} m | box reads {estimate:.2f} m | '
                           f'{"calibrated" if engine.calibrated else "UNCALIBRATED" if engine.kind == "relative" else "model metres"}'
                           ' | c = set, q = quit')
        cv2.imshow(WINDOW, view)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            engine.calibrate(med, args.calibrate)
            print(f'Saved k = {engine.k:.3f} for {engine.calib_key}. The box should now read {args.calibrate} m.')
        elif key in (ord('q'), 27) or window_closed():
            break


def main():
    args = parse_args()
    source = VideoSource(args.video) if args.video else WebcamSource(args.camera)
    engine = DepthEngine(input_size=args.input_size, aspect=source.width / source.height, backend=args.backend,
                         kind=args.depth)
    cfg = PlannerConfig(hfov=args.hfov, react_dist=args.react_dist, drone_width=args.drone_width,
                        clearance=args.clearance)
    planner = Planner(cfg)
    calib = 'calibrated' if engine.calibrated else (
        'UNCALIBRATED (run --calibrate)' if engine.kind == 'relative' else 'not calibrated, using model metres')
    backend = f'{engine.backend} ({engine.device})' if engine.backend == 'torch' else engine.backend
    print(f'Depth: {engine.kind} model on {backend}, input {engine.w}x{engine.h}, {calib}, k={engine.k:.3f}')

    if args.calibrate:
        try:
            calibrate(args, engine, source)
        finally:
            source.close()
            cv2.destroyAllWindows()
        return

    writer = None
    if args.record:
        writer = cv2.VideoWriter(args.record, cv2.VideoWriter_fourcc(*'mp4v'), source.fps,
                                 (source.width + viz.PANEL_W, source.height))
    display = not args.no_display
    if display:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)

    worker = Worker(source, engine, planner) if source.live else None
    show_hud, paused, view, plan = True, False, None, None
    view_fps, depth_fps, last_t, last_id = 0.0, 0.0, time.time(), 0
    t_start = time.time()
    try:
        while True:
            if args.duration and time.time() - t_start > args.duration:
                break
            if not paused:
                if worker:  # live: draw the newest plan over every new camera frame
                    if worker.error:
                        raise worker.error
                    frame, frame_id = source.read()
                    new = frame is not None and frame_id != last_id
                    if new:
                        last_id, plan, depth_fps = frame_id, worker.plan, worker.depth_fps
                else:       # video: every frame, in order
                    frame, _ = source.read()
                    if frame is None:
                        break
                    t0 = time.time()
                    plan = planner.update(engine.estimate(frame), 1 / source.fps)
                    depth_fps = 1 / max(time.time() - t0, 1e-6)
                    new = True
                if new:
                    now = time.time()
                    view_fps = smooth_fps(view_fps, now - last_t)
                    last_t = now
                    if plan is None:  # first depth frame not ready yet
                        view = np.hstack([frame, np.zeros((frame.shape[0], viz.PANEL_W, 3), np.uint8)])
                    else:
                        lines = status_lines(plan, engine, view_fps, depth_fps) if show_hud else ()
                        view = viz.render(frame, plan, cfg, lines)
                    if writer:
                        writer.write(view)
                    if display:
                        cv2.imshow(WINDOW, view)
                elif not display:
                    time.sleep(0.002)

            if display:
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):
                    break
                elif key == ord(' '):
                    paused = not paused
                elif key == ord('h'):
                    show_hud = not show_hud  # status text in the side panel
                if window_closed():  # window closed with the X button
                    break
    except KeyboardInterrupt:
        pass
    finally:
        if worker:
            worker.stop()
        source.close()
        if writer:
            writer.release()
            print(f'Saved {args.record}')
        cv2.destroyAllWindows()


if __name__ == '__main__':
    sys.exit(main())
