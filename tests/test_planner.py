import numpy as np
import pytest

from eyeinthesky.planner import AVOIDING, CLEAR, STOP, Planner, PlannerConfig, focal_px

H, W = 252, 336
DT = 1 / 15


def scene(*boxes, far=10.0):
    """Depth map of a far wall plus boxes: (x0, x1, y0, y1, z) in metres at depth z (y down)."""
    depth = np.full((H, W), far, np.float32)
    f = focal_px(W, PlannerConfig().hfov)
    for x0, x1, y0, y1, z in boxes:
        u0, u1 = int(max(0, W / 2 + x0 / z * f)), int(min(W, W / 2 + x1 / z * f))
        v0, v1 = int(max(0, H / 2 + y0 / z * f)), int(min(H, H / 2 + y1 / z * f))
        depth[v0:v1, u0:u1] = np.minimum(depth[v0:v1, u0:u1], z)
    return depth


def run(planner, depth, frames=10):
    for _ in range(frames):
        plan = planner.update(depth, DT)
    return plan


def test_open_space_flies_straight():
    plan = run(Planner(), scene())
    assert plan.state == CLEAR and plan.yaw == 0 and plan.pitch == 0 and plan.speed == 1.0


def test_obstacle_beyond_react_dist_is_ignored():
    plan = run(Planner(), scene((-0.5, 0.5, -0.5, 0.5, 2.5)))
    assert plan.state == CLEAR and plan.yaw == 0


def test_close_object_off_the_corridor_does_not_change_path():
    # Pole 0.7 m to the left at 1 m: close, but outside the 0.45 m tube
    plan = run(Planner(), scene((-0.9, -0.65, -2, 2, 1.0)))
    assert plan.state == CLEAR and plan.yaw == 0 and plan.pitch == 0


def test_centre_box_triggers_avoidance():
    # Obstacles are placed just inside react_dist (1.5 m), where a real drone would first see them
    plan = run(Planner(), scene((-0.2, 0.2, -0.2, 0.2, 1.4)))
    assert plan.state == AVOIDING
    assert plan.label in ('LEFT', 'RIGHT')  # the box is square and pitch range is smaller, so sideways is cheaper
    assert plan.nearest == pytest.approx(1.4, abs=0.1)
    assert 0 < plan.speed < 1


def test_box_covering_left_and_centre_dodges_right():
    # Full-height box from far left to slightly right of centre, so up/down is not an option
    plan = run(Planner(), scene((-3, 0.2, -3, 3, 1.4)))
    assert plan.state == AVOIDING and plan.label == 'RIGHT' and plan.target_yaw > 0


def test_wide_low_obstacle_climbs():
    # Wide barrier from the centre line down, open above: the smallest turn is upwards
    plan = run(Planner(), scene((-3, 3, 0.0, 3, 1.4)))
    assert plan.state == AVOIDING and plan.label == 'UP' and plan.target_pitch > 0


def test_wall_everywhere_stops():
    plan = run(Planner(), scene((-5, 5, -5, 5, 0.8)))
    assert plan.state == STOP and plan.speed == 0


def test_object_too_close_to_dodge_stops():
    # At 0.7 m no direction within the steering range gets the 0.45 m tube past a 0.4 m box
    plan = run(Planner(), scene((-0.2, 0.2, -0.2, 0.2, 0.7)))
    assert plan.state == STOP and plan.speed == 0


def test_single_noisy_frame_does_not_swerve():
    planner = Planner()
    run(planner, scene())
    plan = planner.update(scene((-0.2, 0.2, -0.2, 0.2, 1.4)), DT)  # one blocked frame
    assert plan.state == CLEAR
    plan = planner.update(scene(), DT)
    assert plan.state == CLEAR and plan.yaw == 0


def test_returns_to_straight_after_obstacle_clears():
    planner = Planner()
    assert run(planner, scene((-0.2, 0.2, -0.2, 0.2, 1.4))).state == AVOIDING
    plan = run(planner, scene(), frames=int(1.0 / DT))
    assert plan.state == CLEAR and abs(plan.yaw) < 1 and abs(plan.pitch) < 1


def test_symmetric_obstacle_with_noise_does_not_flip_flop():
    rng = np.random.default_rng(0)
    planner = Planner()
    sides = []
    base = scene((-0.15, 0.15, -3, 3, 1.4))  # tall pole dead ahead: left and right are equally good
    for _ in range(60):
        noisy = base * rng.normal(1.0, 0.03, base.shape).astype(np.float32)
        sides.append(planner.update(noisy, DT).label)
    sides = [s for s in sides if s]
    changes = sum(a != b for a, b in zip(sides, sides[1:]))
    assert sides and changes == 0
