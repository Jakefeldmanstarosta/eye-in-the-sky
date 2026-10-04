"""Obstacle detection and path choice from a metric depth map.

Coordinates are camera-centred: X right, Y down, Z forward (metres). The default path is straight ahead (+Z).
It only changes when the drone's tube along that path would hit something within `react_dist`.
"""
from __future__ import annotations  # lets `float | None` work on Python 3.9 (macOS's built-in python3)

import math
from dataclasses import dataclass, field

import numpy as np

CLEAR, AVOIDING, STOP = 'CLEAR', 'AVOIDING', 'STOP'


@dataclass
class PlannerConfig:
    hfov: float = 70.0            # camera horizontal field of view, degrees
    drone_width: float = 0.3      # metres
    clearance: float = 0.3        # extra margin around the drone, metres
    react_dist: float = 1.5       # only obstacles closer than this along the path count, metres
    yaw_range: float = 30.0       # how far left/right the path may bend, degrees
    pitch_range: float = 20.0     # how far up/down the path may bend, degrees
    step: float = 2.5             # angular resolution of candidate directions, degrees
    min_points: int = 15          # depth samples needed inside the tube to call it blocked (noise rejection)
    debounce_frames: int = 2      # consecutive frames before switching between clear and avoiding
    stick_time: float = 0.5       # seconds the other side must be clearly better before switching dodge side
    switch_margin: float = 5.0    # degrees by which the other side must be better to start that timer
    ease_time: float = 0.5        # seconds for the path to settle on a new direction
    pool: int = 4                 # depth map is min-pooled by this factor before back-projection

    @property
    def radius(self):
        return self.drone_width / 2 + self.clearance


@dataclass
class Command:
    """Hardware-neutral steering output for a future drone link."""
    yaw_deg: float     # + = right
    pitch_deg: float   # + = up
    speed: float       # 0-1 fraction of cruise speed
    state: str


@dataclass
class Plan:
    state: str
    label: str                    # '', 'LEFT', 'RIGHT', 'UP', 'DOWN'
    yaw: float                    # eased direction the path currently points, degrees
    pitch: float
    target_yaw: float
    target_pitch: float
    nearest: float | None         # distance to the closest obstacle in the straight-ahead tube, metres
    speed: float
    points: np.ndarray = field(repr=False)        # (N, 3) back-projected points near the drone
    pixels: np.ndarray = field(repr=False)        # (N, 2) their (u, v) in depth-map pixels
    in_corridor: np.ndarray = field(repr=False)   # (N,) bool, inside the straight-ahead tube within react_dist
    depth_shape: tuple = (0, 0)

    @property
    def command(self):
        return Command(self.yaw, self.pitch, self.speed, self.state)


def direction(yaw_deg, pitch_deg):
    """Unit vector(s) for yaw (+right) / pitch (+up) in camera coordinates."""
    y, p = np.radians(yaw_deg), np.radians(pitch_deg)
    return np.stack([np.sin(y) * np.cos(p), -np.sin(p), np.cos(y) * np.cos(p)], axis=-1)


def focal_px(width, hfov):
    return (width / 2) / math.tan(math.radians(hfov) / 2)


def depth_to_points(depth, hfov, pool=4, max_range=None):
    """Min-pool the depth map (nearest surface wins), then back-project to 3D with a pinhole model."""
    h, w = depth.shape
    hp, wp = h // pool, w // pool
    d = depth[:hp * pool, :wp * pool].reshape(hp, pool, wp, pool).min(axis=(1, 3))
    f = focal_px(w, hfov)
    v, u = np.mgrid[0:hp, 0:wp]
    u = (u + 0.5) * pool
    v = (v + 0.5) * pool
    z = d
    keep = np.isfinite(z) & (z > 0)
    if max_range is not None:
        keep &= z < max_range
    u, v, z = u[keep], v[keep], z[keep]
    pts = np.stack([(u - w / 2) / f * z, (v - h / 2) / f * z, z], axis=1).astype(np.float32)
    return pts, np.stack([u, v], axis=1).astype(np.float32)


def side_of(yaw, pitch):
    if abs(yaw) < 1e-6 and abs(pitch) < 1e-6:
        return ''
    if abs(yaw) >= abs(pitch):
        return 'RIGHT' if yaw > 0 else 'LEFT'
    return 'UP' if pitch > 0 else 'DOWN'


class Planner:
    def __init__(self, cfg: PlannerConfig = PlannerConfig()):
        self.cfg = cfg
        n_yaw = int(round(cfg.yaw_range / cfg.step))
        n_pitch = int(round(cfg.pitch_range / cfg.step))
        yaws = np.arange(-n_yaw, n_yaw + 1) * cfg.step
        pitches = np.arange(-n_pitch, n_pitch + 1) * cfg.step
        py, yy = np.meshgrid(pitches, yaws, indexing='ij')
        self.cand_yaw, self.cand_pitch = yy.ravel(), py.ravel()
        self.cand_dir = direction(self.cand_yaw, self.cand_pitch).astype(np.float32)
        self.cand_cost = np.hypot(self.cand_yaw, self.cand_pitch)  # smallest turn from straight ahead wins
        self.cand_side = np.array([side_of(y, p) for y, p in zip(self.cand_yaw, self.cand_pitch)])
        self.straight = int(np.argmin(self.cand_cost))
        self.reset()

    def reset(self):
        self.state, self.side = CLEAR, ''
        self.yaw = self.pitch = 0.0
        self.blocked_streak = self.clear_streak = 0
        self.switch_timer = 0.0

    def check(self, points):
        """For every candidate direction: is the drone's tube blocked, and how far is the nearest hit."""
        cfg = self.cfg
        if len(points) == 0:
            return np.zeros(len(self.cand_dir), bool), np.full(len(self.cand_dir), np.inf)
        t = points @ self.cand_dir.T                                   # distance along each direction
        r2 = (points * points).sum(1, keepdims=True) - t * t           # squared distance from its axis
        hit = (t > 0) & (t < cfg.react_dist) & (r2 < cfg.radius ** 2)
        blocked = hit.sum(0) >= cfg.min_points
        nearest = np.where(hit, t, np.inf).min(0)
        return blocked, nearest

    def update(self, depth, dt):
        cfg = self.cfg
        points, pixels = depth_to_points(depth, cfg.hfov, cfg.pool, max_range=cfg.react_dist + cfg.radius)
        blocked, nearest = self.check(points)
        clear = ~blocked

        # Debounce: one noisy frame shouldn't make the path swerve (or snap back)
        if blocked[self.straight]:
            self.blocked_streak, self.clear_streak = self.blocked_streak + 1, 0
        else:
            self.blocked_streak, self.clear_streak = 0, self.clear_streak + 1
        if self.state == CLEAR and self.blocked_streak >= cfg.debounce_frames:
            self.state = AVOIDING
        elif self.state != CLEAR and self.clear_streak >= cfg.debounce_frames:
            self.state, self.side, self.switch_timer = CLEAR, '', 0.0

        target_yaw = target_pitch = 0.0
        if self.state != CLEAR:
            if not clear.any():
                self.state, self.side = STOP, ''
            else:
                self.state = AVOIDING
                cost = np.where(clear, self.cand_cost, np.inf)
                best = int(np.argmin(cost))
                in_side = clear & (self.cand_side == self.side)
                if not self.side or not in_side.any():
                    self.side, self.switch_timer = self.cand_side[best], 0.0  # current side is gone: switch now
                else:
                    best_side = int(np.argmin(np.where(in_side, self.cand_cost, np.inf)))
                    if self.cand_side[best] != self.side and cost[best] < cost[best_side] - cfg.switch_margin:
                        self.switch_timer += dt
                        if self.switch_timer >= cfg.stick_time:
                            self.side, self.switch_timer = self.cand_side[best], 0.0
                    else:
                        self.switch_timer = 0.0
                pick = int(np.argmin(np.where(clear & (self.cand_side == self.side), self.cand_cost, np.inf)))
                target_yaw, target_pitch = float(self.cand_yaw[pick]), float(self.cand_pitch[pick])

        # Ease towards the target (~95% of the way after ease_time)
        a = 1 - math.exp(-3 * dt / cfg.ease_time) if cfg.ease_time > 0 else 1.0
        self.yaw += a * (target_yaw - self.yaw)
        self.pitch += a * (target_pitch - self.pitch)

        straight_nearest = float(nearest[self.straight]) if np.isfinite(nearest[self.straight]) else None
        if self.state == STOP:
            speed = 0.0
        elif self.state == AVOIDING:
            speed = float(np.clip((straight_nearest or cfg.react_dist) / cfg.react_dist, 0.2, 1.0))
        else:
            speed = 1.0

        # Points inside the straight-ahead tube: what the display highlights as the obstacle
        if len(points):
            fwd = self.cand_dir[self.straight]
            t = points @ fwd
            r2 = (points * points).sum(1) - t * t
            in_corridor = (t > 0) & (t < cfg.react_dist) & (r2 < cfg.radius ** 2)
        else:
            in_corridor = np.zeros(0, bool)

        return Plan(self.state, self.side if self.state == AVOIDING else '', self.yaw, self.pitch,
                    target_yaw, target_pitch, straight_nearest, speed, points, pixels, in_corridor, depth.shape)
