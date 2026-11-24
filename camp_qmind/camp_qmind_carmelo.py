import time

import cv2
import numpy as np
import torch
from PIL import Image
from transformers import pipeline

# Use the Apple GPU if available, otherwise fall back to CPU
DEVICE = "mps" if torch.backends.mps.is_available() else "cpu"
MODEL = "depth-anything/Depth-Anything-V2-Small-hf"  # swap Small for Base/Large to compare
WIDTH, HEIGHT = 640, 360  # smaller = faster, but thin objects may disappear

ROWS, COLS = 10, 10  # 100 sample points in a grid
PATCH = 10  # average a small square around each point instead of one noisy pixel
CENTER_RADIUS = HEIGHT // 2  # centre zone: only react to obstacles inside this circle


def rank_points(depth):
    """Return (x, y, value) for each grid point, sorted closest first."""
    h, w = depth.shape
    points = []
    for r in range(ROWS):
        for c in range(COLS):
            x = int((c + 0.5) * w / COLS)
            y = int((r + 0.5) * h / ROWS)
            patch = depth[y - PATCH:y + PATCH, x - PATCH:x + PATCH]
            points.append((x, y, patch.mean()))
    return sorted(points, key=lambda p: p[2], reverse=True)  # higher value = closer

print(f"Loading {MODEL} on {DEVICE}...")
pipe = pipeline("depth-estimation", model=MODEL, device=DEVICE)

cap = cv2.VideoCapture(0)  # 0 = built-in webcam
if not cap.isOpened():
    raise SystemExit("Could not open webcam. Check camera permissions in System Settings.")

prev = time.time()
while True:
    ok, frame = cap.read()
    if not ok:
        break

    frame = cv2.resize(frame, (WIDTH, HEIGHT))
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # OpenCV uses BGR, the model expects RGB

    depth = np.array(pipe(Image.fromarray(rgb))["depth"])
    depth_color = cv2.applyColorMap(depth, cv2.COLORMAP_INFERNO)  # bright/yellow = close

    # Rank all points closest first; dots fade from red (closest) to blue (furthest)
    ranked = rank_points(depth)
    n = len(ranked)
    for rank, (x, y, value) in enumerate(ranked, start=1):
        t = (rank - 1) / (n - 1)  # 0 for closest, 1 for furthest
        color = (int(255 * t), 0, int(255 * (1 - t)))  # OpenCV colors are BGR
        for img in (frame, depth_color):
            cv2.circle(img, (x, y), 4, color, -1)  # -1 = filled dot
            if rank in (1, n):  # only label the closest and furthest
                cv2.putText(img, str(rank), (x + 8, y + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Draw the centre zone, and only draw the arrow if the closest point is inside it
    closest, furthest = ranked[0], ranked[-1]
    cx, cy = WIDTH // 2, HEIGHT // 2
    in_center = (closest[0] - cx) ** 2 + (closest[1] - cy) ** 2 <= CENTER_RADIUS ** 2
    for img in (frame, depth_color):
        cv2.circle(img, (cx, cy), CENTER_RADIUS, (255, 255, 255), 1)
        if in_center:
            cv2.arrowedLine(img, closest[:2], furthest[:2], (0, 255, 0), 3, tipLength=0.08)

    now = time.time()
    fps = 1 / (now - prev)
    prev = now

    combined = np.hstack([frame, depth_color])
    cv2.putText(combined, f"{fps:.1f} FPS", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.imshow("Webcam | Depth  (press q to quit)", combined)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
