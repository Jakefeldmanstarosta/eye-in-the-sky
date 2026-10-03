"""Camera/video sources and the depth engine (Depth Anything V2 relative model, converted to metres by calibration)."""
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

# depth_m = k / disparity. Uncalibrated default estimated from a head ~0.35 m away on the laptop webcam at 336x252.
# Run `python avoid.py --calibrate <metres>` to measure it for your camera.
DEFAULT_K = 1.3
MAX_DEPTH = 20.0


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


class DepthEngine:
    """Frame (BGR uint8) -> depth in metres at the model's input resolution.

    The relative model outputs disparity (bigger = closer) with an unknown scale, so depth = k / disparity,
    where k comes from a one-off calibration against a known distance (see avoid.py --calibrate).
    """

    def __init__(self, input_size=252, aspect=4 / 3, encoder='vits', backend='openvino'):
        # Fixed input size with the camera's aspect ratio; both sides must be multiples of 14
        self.h = round(input_size / 14) * 14
        self.w = round(input_size * aspect / 14) * 14
        self.backend = backend
        self.calib_key = f'{encoder}_{self.w}x{self.h}'
        k = load_calibration(self.calib_key)
        self.calibrated = k is not None
        self.k = k if k is not None else DEFAULT_K

        if REPO_DIR not in sys.path:
            sys.path.insert(0, REPO_DIR)
        from depth_anything_v2.dpt import DepthAnythingV2

        ckpt = os.path.join(REPO_DIR, 'checkpoints', f'depth_anything_v2_{encoder}.pth')
        self.model = DepthAnythingV2(**MODEL_CONFIGS[encoder])
        self.model.load_state_dict(torch.load(ckpt, map_location='cpu'))
        self.model.eval()
        torch.set_num_threads(os.cpu_count())

        self.ov_model = None
        if backend == 'openvino':
            try:
                # Same file name as the notebook uses, so the converted model is shared
                self.ov_model = self._load_openvino(f'da2_{encoder}_{self.w}x{self.h}.xml')
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

    def disparity(self, frame_bgr, backend=None):
        x = self.preprocess(frame_bgr)
        if (backend or self.backend) == 'openvino' and self.ov_model is not None:
            return self.ov_model(x)[0][0]
        with torch.inference_mode():
            return self.model(torch.from_numpy(x))[0].numpy()

    def to_metres(self, disparity):
        return np.minimum(self.k / np.maximum(disparity, 1e-3), MAX_DEPTH).astype(np.float32)

    def estimate(self, frame_bgr):
        return self.to_metres(self.disparity(frame_bgr))


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
