"""Camera/video sources and the depth engine (Depth Anything V2, relative or metric model, output in metres)."""
import json
import os
import sys
import threading
import time
import warnings

import cv2
import numpy as np
import torch

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO_DIR = os.path.join(ROOT, 'Depth-Anything-V2')
MODELS_DIR = os.path.join(ROOT, 'models')
CALIBRATION_FILE = os.path.join(MODELS_DIR, 'calibration.json')

MODEL_CONFIGS = {
    'vits': {'encoder': 'vits', 'features': 64,  'out_channels': [48, 96, 192, 384]},
    'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
    'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
}
MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)

# Relative model: depth_m = k / disparity. Uncalibrated default estimated from a head ~0.35 m away on the laptop
# webcam at 336x252. Metric model: depth_m = k * model output, uncalibrated k = 1 (trust the model).
# Run `python avoid.py --calibrate <metres>` to measure k for your camera.
DEFAULT_K = {'relative': 1.3, 'metric': 1.0}
MAX_DEPTH = 20.0  # also the indoor (Hypersim) metric model's range


def load_calibration(key):
    try:
        with open(CALIBRATION_FILE) as f:
            return json.load(f).get(key)
    except (OSError, ValueError):
        return None


def save_calibration(key, k):
    data = {}
    try:
        with open(CALIBRATION_FILE) as f:
            data = json.load(f)
    except (OSError, ValueError):
        pass
    data[key] = k
    os.makedirs(MODELS_DIR, exist_ok=True)
    with open(CALIBRATION_FILE, 'w') as f:
        json.dump(data, f, indent=2)


def import_dpt(package_dir):
    """Import DepthAnythingV2 from the relative (repo root) or metric (metric_depth/) package.

    Both packages are called `depth_anything_v2`, so drop any previously imported copy first.
    """
    for name in [m for m in sys.modules if m == 'depth_anything_v2' or m.startswith('depth_anything_v2.')]:
        del sys.modules[name]
    sys.path.insert(0, package_dir)
    try:
        from depth_anything_v2.dpt import DepthAnythingV2
    finally:
        sys.path.remove(package_dir)
    return DepthAnythingV2


class DepthEngine:
    """Frame (BGR uint8) -> depth in metres at the model's input resolution.

    kind='relative': the general model outputs disparity (bigger = closer) with an unknown scale, so
        depth = k / disparity, with k from a one-off calibration against a known distance (avoid.py --calibrate).
    kind='metric': the indoor (Hypersim) metric model outputs metres directly; calibration optionally rescales it.
    """

    def __init__(self, input_size=252, aspect=4 / 3, encoder='vits', backend='openvino', kind='relative'):
        # Fixed input size with the camera's aspect ratio; both sides must be multiples of 14
        self.h = round(input_size / 14) * 14
        self.w = round(input_size * aspect / 14) * 14
        self.backend, self.kind = backend, kind
        if kind == 'relative':
            name = f'{encoder}_{self.w}x{self.h}'  # same as the notebook's, so the converted model is shared
            DepthAnythingV2 = import_dpt(REPO_DIR)
            self.model = DepthAnythingV2(**MODEL_CONFIGS[encoder])
            ckpt = f'depth_anything_v2_{encoder}.pth'
        elif kind == 'metric':
            name = f'metric_hypersim_{encoder}_{self.w}x{self.h}'
            DepthAnythingV2 = import_dpt(os.path.join(REPO_DIR, 'metric_depth'))
            self.model = DepthAnythingV2(**MODEL_CONFIGS[encoder], max_depth=MAX_DEPTH)
            ckpt = f'depth_anything_v2_metric_hypersim_{encoder}.pth'
        else:
            raise ValueError(f'unknown depth model kind {kind!r}')
        self.model.load_state_dict(torch.load(os.path.join(REPO_DIR, 'checkpoints', ckpt), map_location='cpu'))
        self.model.eval()
        torch.set_num_threads(os.cpu_count())

        self.calib_key = name
        k = load_calibration(self.calib_key)
        self.calibrated = k is not None
        self.k = k if k is not None else DEFAULT_K[kind]

        self.ov_model = None
        if backend == 'openvino':
            try:
                self.ov_model = self._load_openvino(f'da2_{name}.xml')
            except Exception as e:
                print(f'OpenVINO unavailable ({e!r}), falling back to torch')
                self.backend = 'torch'

    def _load_openvino(self, name):
        import openvino as ov
        core = ov.Core()
        path = os.path.join(MODELS_DIR, name)
        if os.path.exists(path):
            ir = core.read_model(path)
        else:
            print('Converting depth model to OpenVINO (first run only)...')
            with torch.no_grad(), warnings.catch_warnings():
                warnings.simplefilter('ignore')  # tracing warnings from the repo's shape asserts are harmless
                ir = ov.convert_model(self.model, example_input=torch.zeros(1, 3, self.h, self.w))
            os.makedirs(MODELS_DIR, exist_ok=True)
            ov.save_model(ir, path)
            print(f'Saved {path}')
        return core.compile_model(ir, 'CPU', {'PERFORMANCE_HINT': 'LATENCY'})

    def preprocess(self, frame_bgr):
        rgb = cv2.cvtColor(cv2.resize(frame_bgr, (self.w, self.h), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB)
        x = (rgb.astype(np.float32) / 255.0 - MEAN) / STD
        return np.ascontiguousarray(x.transpose(2, 0, 1)[None])

    def raw(self, frame_bgr, backend=None):
        """Model output before calibration: disparity (relative) or metres (metric)."""
        x = self.preprocess(frame_bgr)
        if (backend or self.backend) == 'openvino' and self.ov_model is not None:
            return self.ov_model(x)[0][0]
        with torch.inference_mode():
            return self.model(torch.from_numpy(x))[0].numpy()

    def to_metres(self, raw, k=None):
        k = self.k if k is None else k
        depth = k / np.maximum(raw, 1e-3) if self.kind == 'relative' else k * raw
        return np.minimum(depth, MAX_DEPTH).astype(np.float32)

    def calibrate(self, raw_value, metres):
        """Set and save k so that a raw model value reads as the given distance."""
        self.k = metres * raw_value if self.kind == 'relative' else metres / max(raw_value, 1e-6)
        self.calibrated = True
        save_calibration(self.calib_key, self.k)

    def estimate(self, frame_bgr):
        return self.to_metres(self.raw(frame_bgr))


def open_camera(index=0, width=640, height=480):
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # keep latency low: always grab the newest frame
    if not cap.isOpened():
        raise RuntimeError(f'Could not open camera {index}. Is another app using it?')
    return cap


class WebcamSource:
    """Background thread that always holds the newest camera frame. read() never blocks on the camera."""
    live = True

    def __init__(self, index=0, width=640, height=480):
        self.cap = open_camera(index, width, height)
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.lock, self.stopped = threading.Lock(), threading.Event()
        self.frame, self.frame_id, self.error = None, 0, None
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while not self.stopped.is_set():
            ok, frame = self.cap.read()
            if not ok:
                self.error = RuntimeError('Lost camera frame')
                self.stopped.set()
                break
            with self.lock:
                self.frame, self.frame_id = frame, self.frame_id + 1

    def read(self):
        """Returns (frame, frame_id); frame is None until the first one arrives."""
        with self.lock:
            return self.frame, self.frame_id

    def close(self):
        self.stopped.set()
        self.thread.join(timeout=5)
        self.cap.release()


class VideoSource:
    """Reads every frame of a video file in order (deterministic, for repeatable tests)."""
    live = False

    def __init__(self, path):
        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            raise RuntimeError(f'Could not open video {path}')
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.frame_id = 0

    def read(self):
        """Returns (frame, frame_id); frame is None at the end of the file."""
        ok, frame = self.cap.read()
        if not ok:
            return None, self.frame_id
        self.frame_id += 1
        return frame, self.frame_id

    def close(self):
        self.cap.release()


def wait_for_frame(source, timeout=5.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        frame, _ = source.read()
        if frame is not None:
            return frame
        time.sleep(0.01)
    raise RuntimeError('No frame from camera')
