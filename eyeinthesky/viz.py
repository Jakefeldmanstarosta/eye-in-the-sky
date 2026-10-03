"""Drawing: obstacle tint, flight corridor and planned path on the camera view, plus a top-down mini-map."""
import math

import cv2
import numpy as np

from .planner import AVOIDING, CLEAR, STOP, Plan, PlannerConfig, focal_px

STATE_COLORS = {CLEAR: (80, 200, 80), AVOIDING: (0, 165, 255), STOP: (40, 40, 230)}  # BGR
OBSTACLE_COLOR = (40, 40, 230)
PANEL_W = 240  # side panel with the mini-map and status, right of the camera view


def aim_point(yaw, pitch, w, h, hfov):
    """Where a direction (degrees) lands on the image (its vanishing point)."""
    f = focal_px(w, hfov)
    y, p = math.radians(yaw), math.radians(pitch)
    return (w / 2 + f * math.tan(y), h / 2 - f * math.tan(p) / math.cos(y))


def bezier(p0, p1, p2, n=24):
    t = np.linspace(0, 1, n)[:, None]
    return ((1 - t) ** 2 * np.array(p0) + 2 * (1 - t) * t * np.array(p1) + t ** 2 * np.array(p2)).astype(np.int32)


def draw_obstacles(img, plan: Plan, cfg: PlannerConfig):
    """Tint the parts of the image that block the straight-ahead path."""
    if not plan.in_corridor.any():
        return
    dh, dw = plan.depth_shape
    mask = np.zeros((dh // cfg.pool, dw // cfg.pool), np.uint8)
    uv = (plan.pixels[plan.in_corridor] / cfg.pool).astype(int)
    mask[uv[:, 1], uv[:, 0]] = 255
    mask = cv2.resize(mask, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
    tint = img.copy()
    tint[mask > 0] = OBSTACLE_COLOR
    cv2.addWeighted(tint, 0.45, img, 0.55, 0, dst=img)


def draw_corridor(img, cfg: PlannerConfig):
    """Cross-section of the straight-ahead safety tube at the reaction distance."""
    h, w = img.shape[:2]
    r = int(focal_px(w, cfg.hfov) * cfg.radius / cfg.react_dist)
    cv2.circle(img, (w // 2, h // 2), r, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(img, f'{cfg.react_dist:g} m', (w // 2 + r + 4, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                (255, 255, 255), 1, cv2.LINE_AA)


def draw_path(img, plan: Plan, cfg: PlannerConfig):
    """Path from the drone (bottom centre) heading forward, then bending to the planned direction."""
    h, w = img.shape[:2]
    color = STATE_COLORS[plan.state]
    if plan.state == STOP:
        cx, cy = w // 2, h // 2
        pts = [(cx + int(40 * math.cos(math.radians(22.5 + 45 * i))), cy + int(40 * math.sin(math.radians(22.5 + 45 * i))))
               for i in range(8)]
        cv2.fillPoly(img, [np.array(pts)], color, cv2.LINE_AA)
        (tw, th), _ = cv2.getTextSize('STOP', cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
        cv2.putText(img, 'STOP', (cx - tw // 2, cy + th // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2,
                    cv2.LINE_AA)
        return
    start = (w / 2, h - 1)
    end = aim_point(plan.yaw, plan.pitch, w, h, cfg.hfov)
    ctrl = (w / 2, h * 0.62)  # path leaves the drone heading straight forward
    curve = bezier(start, ctrl, end)
    cv2.polylines(img, [curve], False, (0, 0, 0), 7, cv2.LINE_AA)
    cv2.polylines(img, [curve], False, color, 4, cv2.LINE_AA)
    draw_arrowhead(img, curve[-4], curve[-1], color, 18)


def draw_arrowhead(img, p_from, p_to, color, size):
    d = np.array(p_to, float) - np.array(p_from, float)
    d /= max(np.linalg.norm(d), 1e-6)
    n = np.array([-d[1], d[0]])
    tip = np.array(p_to, float)
    head = np.array([tip + d * size * 0.3, tip - d * size + n * size * 0.6, tip - d * size - n * size * 0.6])
    cv2.fillPoly(img, [head.astype(np.int32)], (0, 0, 0), cv2.LINE_AA)
    cv2.fillPoly(img, [((head - tip) * 0.75 + tip).astype(np.int32)], color, cv2.LINE_AA)


def minimap(plan: Plan, cfg: PlannerConfig, size=240):
    """Top-down view (X right, Z forward) of nearby obstacles, the safety tube and the planned path."""
    z_max = 2 * cfg.react_dist
    scale = (size - 30) / z_max
    m = np.full((size, size, 3), 30, np.uint8)
    ox, oy = size // 2, size - 15  # drone position on the map

    def to_map(x, z):
        return int(ox + x * scale), int(oy - z * scale)

    # Field of view: everything outside these lines is unseen (assumed free)
    for s in (-1, 1):
        a = math.radians(cfg.hfov / 2)
        cv2.line(m, (ox, oy), to_map(s * z_max * math.tan(a), z_max), (70, 70, 70), 1, cv2.LINE_AA)
    # Straight-ahead tube and reaction distance
    for s in (-1, 1):
        cv2.line(m, to_map(s * cfg.radius, 0), to_map(s * cfg.radius, cfg.react_dist), (120, 120, 120), 1)
    cv2.line(m, to_map(-cfg.radius, cfg.react_dist), to_map(cfg.radius, cfg.react_dist), (120, 120, 120), 1)

    # Obstacles at the drone's height (within the tube's vertical extent)
    if len(plan.points):
        band = np.abs(plan.points[:, 1]) < cfg.radius
        for pts, color in ((plan.points[band & ~plan.in_corridor], (150, 150, 150)),
                           (plan.points[band & plan.in_corridor], OBSTACLE_COLOR)):
            px = (ox + pts[:, 0] * scale).astype(int)
            py = (oy - pts[:, 2] * scale).astype(int)
            ok = (px >= 1) & (px < size - 1) & (py >= 1) & (py < size - 1)
            for dx, dy in ((0, 0), (1, 0), (0, 1), (1, 1)):  # 2x2 dots
                m[py[ok] + dy, px[ok] + dx] = color

    color = STATE_COLORS[plan.state]
    if plan.state == STOP:
        cv2.putText(m, 'STOP', (ox - 22, oy - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
    else:
        end_z = 1.3 * cfg.react_dist
        end = to_map(end_z * math.tan(math.radians(plan.yaw)), end_z)
        curve = bezier((ox, oy), to_map(0, end_z * 0.45), end)
        cv2.polylines(m, [curve], False, color, 2, cv2.LINE_AA)
        draw_arrowhead(m, curve[-4], curve[-1], color, 10)
        if abs(plan.pitch) > 2:  # the top-down view can't show height, so mark climb/descend
            tip = (end[0], end[1] - 8) if plan.pitch > 0 else (end[0], end[1] + 8)
            base = end[1] + 4 if plan.pitch > 0 else end[1] - 4
            cv2.fillPoly(m, [np.array([tip, (end[0] - 6, base), (end[0] + 6, base)])], color, cv2.LINE_AA)
    cv2.fillPoly(m, [np.array([(ox, oy - 8), (ox - 6, oy + 6), (ox + 6, oy + 6)])], (255, 255, 255), cv2.LINE_AA)
    cv2.putText(m, f'top-down, {z_max:g} m ahead', (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1,
                cv2.LINE_AA)
    return m


def side_panel(plan: Plan, cfg: PlannerConfig, height, lines, width=PANEL_W):
    """Mini-map on top, status lines below; sits to the right of the camera view."""
    panel = np.full((height, width, 3), 20, np.uint8)
    m = minimap(plan, cfg, width)
    panel[:width] = m
    y = width + 28
    title = plan.state + (f' {plan.label}' if plan.label else '')
    cv2.putText(panel, title, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, STATE_COLORS[plan.state], 2, cv2.LINE_AA)
    y += 30
    for line in lines:
        cv2.putText(panel, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1, cv2.LINE_AA)
        y += 22
    return panel


def draw_hud(img, text):
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    cv2.rectangle(img, (5, 8), (15 + tw, 16 + th), (0, 0, 0), -1)
    cv2.putText(img, text, (10, 12 + th), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)


def render(frame, plan: Plan, cfg: PlannerConfig, lines=()):
    """Camera view with obstacles/corridor/path, plus the side panel (mini-map + status lines)."""
    img = frame.copy()
    draw_obstacles(img, plan, cfg)
    draw_corridor(img, cfg)
    draw_path(img, plan, cfg)
    return np.hstack([img, side_panel(plan, cfg, img.shape[0], lines)])
