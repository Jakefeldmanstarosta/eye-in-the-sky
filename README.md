# Eye in the Sky

Live monocular depth (Depth Anything V2) for autonomous drone obstacle avoidance.

- `camp_qmind.ipynb`: live webcam depth map viewer (see the notebook for its own instructions).
- `avoid.py`: obstacle-avoidance path MVP (below).

## Setup (once)

```bash
python -m venv .venv
.venv/bin/pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
.venv/bin/pip install opencv-python matplotlib ipykernel openvino pytest
git clone https://github.com/DepthAnything/Depth-Anything-V2
mkdir -p Depth-Anything-V2/checkpoints
curl -L -o Depth-Anything-V2/checkpoints/depth_anything_v2_vits.pth \
  "https://huggingface.co/depth-anything/Depth-Anything-V2-Small/resolve/main/depth_anything_v2_vits.pth?download=true"
# Only needed for avoid.py --depth metric:
curl -L -o Depth-Anything-V2/checkpoints/depth_anything_v2_metric_hypersim_vits.pth \
  "https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-Hypersim-Small/resolve/main/depth_anything_v2_metric_hypersim_vits.pth?download=true"
```

## Obstacle-avoidance MVP (`avoid.py`)

The webcam stands in for the drone's forward camera. Every depth frame is turned into 3D points. The drone's default path is **straight ahead**, and it is only changed when the drone's safety tube would hit something within the reaction distance:

- **CLEAR** (green): nothing in the way; the path goes straight.
- **AVOIDING LEFT/RIGHT/UP/DOWN** (orange): the path bends to the clear direction that needs the smallest turn. It sticks with a side instead of flip-flopping, and eases back to straight once the way is clear.
- **STOP** (red): no direction in the steering range gets past (e.g. a wall right in front).

The window shows the camera view with the blocking obstacle tinted red, the safety tube's outline at the reaction distance, and the planned path. A side panel shows a top-down mini-map (grey = nearby points, red = in the way, lines = field of view and the tube), plus status.

### Depth models

| `--depth` | Model | Distances |
|---|---|---|
| `relative` (default) | General Depth Anything V2 | Relative depth turned into metres by calibration (`depth = k / disparity`). Needs `--calibrate` once. |
| `metric` | Indoor metric model (Hypersim, 0–20 m) | Outputs metres directly. Calibration is optional and rescales them (`depth = k × output`). |

On the laptop webcam the metric model reads close objects as much farther away than they are. Uncalibrated, a face about 0.6 m away read as about 2.5 m, so it said CLEAR. It's better suited to obstacles a few metres away. If you use it up close, calibrate it too. Each model keeps its own calibration in `models/calibration.json`.

### 1. Calibrate distances (once per camera)

The depth model only gives *relative* depth, so it needs one known distance to output metres. Measure 1 m from the laptop's webcam to a wall or door, then:

```bash
.venv/bin/python avoid.py --calibrate 1.0
```

Aim the yellow centre box at the wall and press `c`. The box should now read 1.00 m. Press `q`. The value is saved to `models/calibration.json` and used automatically. Until you calibrate, the relative model's distances are a rough guess and the status says **UNCALIBRATED**. Add `--depth metric` to calibrate the metric model instead.

### 2. Run

```bash
.venv/bin/python avoid.py                              # live webcam
.venv/bin/python avoid.py --record run.mp4             # ...and save the annotated view
.venv/bin/python avoid.py --video run_raw.mp4          # replay a recorded (un-annotated) clip
.venv/bin/python avoid.py --depth metric               # use the metric depth model
```

Keys: `q`/`Esc` quit, `space` pause, `h` status text on/off.

Useful flags (defaults in brackets): `--depth` [relative], `--react-dist` [1.5 m], `--drone-width` [0.3 m], `--clearance` [0.3 m], `--hfov` [70°, the webcam's horizontal field of view], `--input-size` [252], `--backend` [openvino], `--no-display`, `--duration`.

To test it: walk slowly toward a chair or door frame holding the laptop, keeping it at the drone's height. The path should stay green and straight until the object is within 1.5 m and in line with the camera centre, then bend around it.

### Tests

```bash
.venv/bin/python -m pytest tests
```

These check the planner on synthetic depth maps: it goes straight when clear, ignores close objects outside the tube, dodges in the right direction, stops at walls, and doesn't flicker.

### Code

| File | What it does |
|---|---|
| `eyeinthesky/depth.py` | Webcam/video sources, relative or metric depth model (OpenVINO, falls back to PyTorch), calibration |
| `eyeinthesky/planner.py` | Depth → 3D points → collision check for each candidate direction → `Plan` / `Command` |
| `eyeinthesky/viz.py` | Drawing on the camera view + mini-map side panel |
| `avoid.py` | Command line, threads, window, recording |

`Plan.command` gives a hardware-neutral `Command(yaw_deg, pitch_deg, speed, state)`. That's the hook for a real drone link later.

### Known limitations

- **Unseen space counts as free.** Anything outside the camera's view (to the sides, or too close to see) is assumed empty.
- **Distances are estimates** from one camera and one calibration constant. They may drift between very different scenes; recalibrate if they look off. `--hfov` also affects them.
- **No memory or goal.** "Straight ahead" always means wherever the camera points; obstacles that leave the view are forgotten.
- **Close obstacles.** With the default 0.9 m-wide safety tube and ±30° steering, an obstacle only about 1 m ahead often can't be dodged, so the planner shows STOP. That's intended.
