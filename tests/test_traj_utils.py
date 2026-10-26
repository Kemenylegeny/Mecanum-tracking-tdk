"""Tests for traj_utils (no Isaac Lab needed): python -m pytest tests/  or  python tests/test_traj_utils.py"""

import math
import os
import sys
import tempfile

import numpy as np
import torch

_TASK = os.path.join("source", "mecanum_tracking_ws", "mecanum_tracking_ws", "tasks", "manager_based", "mecanum_tracking")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", _TASK, "mdp"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts", "tools"))
from traj_utils import TH_COL, VX_COL, X_COL, Y_COL, anchor_to_world, load_trajectories, sample_table, yaw_from_quat_wxyz  # noqa: E402
import gen_reference_trajectories as gen  # noqa: E402


def _write_refs(tmp):
    files = []
    for name, path in {"circle": gen.circle(1.0, 2000), "eight": gen.figure_eight(1.5, 2000)}.items():
        t, p = gen.time_parametrize(path, 0.5, 0.01)
        fn = os.path.join(tmp, f"{name}.csv")
        np.savetxt(fn, gen.build_table(t, p, "tangent"), delimiter=",", header="t,X,Y,theta,vx,vy,omega", comments="")
        files.append(fn)
    return files


def test_exact_samples_and_interpolation():
    with tempfile.TemporaryDirectory() as tmp:
        traj, traj_len, dt = load_trajectories(_write_refs(tmp), "cpu")
        assert abs(dt - 0.01) < 1e-6
        idx = torch.tensor([0, 1])
        # on a sample point the table row comes back exactly
        tq = torch.tensor([[0.5], [0.5]])
        rows = sample_table(traj, traj_len, dt, idx, tq, loop=True)[:, 0]
        assert torch.allclose(rows[0, X_COL], traj[0, 50, X_COL], atol=1e-5)
        # halfway between two samples = mean of the two rows (x, y)
        tq = torch.tensor([[0.505], [0.505]])
        rows = sample_table(traj, traj_len, dt, idx, tq, loop=True)[:, 0]
        mid = 0.5 * (traj[0, 50, X_COL] + traj[0, 51, X_COL])
        assert torch.allclose(rows[0, X_COL], mid, atol=1e-5)


def test_heading_wraps_across_pi():
    # two samples: theta = 3.1 -> -3.1 (a 0.083 rad turn through +pi, NOT a 6.2 rad turn back)
    traj = torch.zeros(1, 2, 7)
    traj[0, :, 0] = torch.tensor([0.0, 1.0])
    traj[0, 0, TH_COL] = 3.1
    traj[0, 1, TH_COL] = -3.1
    rows = sample_table(traj, torch.tensor([2]), 1.0, torch.tensor([0]), torch.tensor([[0.5]]), loop=False)
    th = rows[0, 0, TH_COL].item()
    assert abs(abs(th) - math.pi) < 1e-3, th  # halfway is +-pi, not 0


def test_loop_and_clamp():
    with tempfile.TemporaryDirectory() as tmp:
        traj, traj_len, dt = load_trajectories(_write_refs(tmp), "cpu")
        dur = (traj_len[0].item() - 1) * dt
        idx = torch.tensor([0])
        a = sample_table(traj, traj_len, dt, idx, torch.tensor([[0.3]]), loop=True)
        b = sample_table(traj, traj_len, dt, idx, torch.tensor([[0.3 + dur]]), loop=True)
        assert torch.allclose(a[..., X_COL], b[..., X_COL], atol=1e-4)  # one period later = same point
        c = sample_table(traj, traj_len, dt, idx, torch.tensor([[dur + 5.0]]), loop=False)
        last = traj[0, traj_len[0] - 1]
        assert torch.allclose(c[0, 0, X_COL], last[X_COL], atol=1e-5)  # held at the end


def test_anchor_puts_start_on_robot():
    # trajectory frame rotated by 90 deg and shifted to (2, 1): the first point lands on (2, 1)
    rows = torch.zeros(1, 2, 7)
    rows[0, 1, X_COL] = 1.0  # second point 1 m along the trajectory's x
    rows[0, 1, VX_COL] = 0.5
    anchor = torch.tensor([[2.0, 1.0, math.pi / 2]])
    w = anchor_to_world(rows, anchor, rotate_velocity=False)
    assert torch.allclose(w[0, 0, [X_COL, Y_COL]], torch.tensor([2.0, 1.0]), atol=1e-6)
    assert torch.allclose(w[0, 1, [X_COL, Y_COL]], torch.tensor([2.0, 2.0]), atol=1e-6)  # +x -> +y
    assert abs(w[0, 1, VX_COL].item() - 0.5) < 1e-6  # body-frame velocity unchanged


def test_body_velocity_consistent_with_path():
    # integrating the body-frame velocities with the heading must reproduce the path
    path = gen.figure_eight(1.5, 2000)
    t, p = gen.time_parametrize(path, 0.5, 0.01)
    tab = gen.build_table(t, p, "tangent")
    th, vx, vy = tab[:, TH_COL], tab[:, VX_COL], tab[:, 5]
    vxw = np.cos(th) * vx - np.sin(th) * vy
    vyw = np.sin(th) * vx + np.cos(th) * vy
    x = np.concatenate([[0.0], np.cumsum(0.5 * (vxw[1:] + vxw[:-1]) * 0.01)])
    assert np.max(np.abs(x - tab[:, X_COL])) < 0.02


def test_yaw_from_quat():
    yaw = 2.0
    q = torch.tensor([[math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]])
    assert abs(yaw_from_quat_wxyz(q).item() - yaw) < 1e-6


def test_preview_to_body_invariance():
    # the same relative situation, rotated + shifted in the world, must give the SAME body-frame observation
    from traj_utils import preview_to_body

    torch.manual_seed(1)
    rows = torch.randn(1, 3, 7)
    rows[..., TH_COL] = torch.remainder(rows[..., TH_COL] + math.pi, 2 * math.pi) - math.pi
    robot_xy = torch.tensor([[0.3, -0.2]])
    robot_yaw = torch.tensor([0.4])
    a = preview_to_body(rows, robot_xy, robot_yaw)
    # rotate the whole world by phi around the origin and shift by (5, 7)
    phi = 1.3
    anchor = torch.tensor([[5.0, 7.0, phi]])
    rows2 = anchor_to_world(rows, anchor, rotate_velocity=False)
    c, s = math.cos(phi), math.sin(phi)
    xy2 = torch.tensor([[5.0 + c * 0.3 - s * -0.2, 7.0 + s * 0.3 + c * -0.2]])
    b = preview_to_body(rows2, xy2, robot_yaw + phi)
    assert torch.allclose(a, b, atol=1e-5)


def test_preview_to_body_simple():
    from traj_utils import preview_to_body

    # robot at origin facing +y (yaw 90 deg); reference 1 m ahead in world +y, same heading, moving forward
    rows = torch.zeros(1, 1, 7)
    rows[0, 0, Y_COL] = 1.0
    rows[0, 0, TH_COL] = math.pi / 2
    rows[0, 0, VX_COL] = 0.5
    o = preview_to_body(rows, torch.zeros(1, 2), torch.tensor([math.pi / 2]))[0, 0]
    assert torch.allclose(o[:2], torch.tensor([1.0, 0.0]), atol=1e-6)  # straight ahead in the body frame
    assert torch.allclose(o[2:4], torch.tensor([0.0, 1.0]), atol=1e-6)  # sin, cos of zero heading error
    assert torch.allclose(o[4:6], torch.tensor([0.5, 0.0]), atol=1e-6)  # forward reference velocity


def test_heading_free_reference_velocity():
    # heading-free files: theta column = 0, (vx, vy) in the trajectory frame. Whatever the robot's heading is,
    # the observed reference velocity must be the reference's WORLD velocity expressed in the robot body frame.
    from traj_utils import VY_COL, preview_to_body

    torch.manual_seed(0)
    n = 200
    rows = torch.zeros(n, 1, 7)
    rows[..., VX_COL] = torch.randn(n, 1)
    rows[..., VY_COL] = torch.randn(n, 1)
    anchor = torch.stack([torch.randn(n), torch.randn(n), (torch.rand(n) * 2 - 1) * math.pi], 1)
    w = anchor_to_world(rows, anchor, rotate_velocity=False)
    c, s = torch.cos(anchor[:, 2:3]), torch.sin(anchor[:, 2:3])
    vxw = c * rows[..., VX_COL] - s * rows[..., VY_COL]
    vyw = s * rows[..., VX_COL] + c * rows[..., VY_COL]
    yaw = (torch.rand(n) * 2 - 1) * math.pi
    o = preview_to_body(w, torch.zeros(n, 2), yaw)
    cy, sy = torch.cos(yaw)[:, None], torch.sin(yaw)[:, None]
    assert torch.allclose(o[..., 4], cy * vxw + sy * vyw, atol=1e-5)
    assert torch.allclose(o[..., 5], -sy * vxw + cy * vyw, atol=1e-5)


def test_tracking_errors():
    # a robot exactly on the reference, moving with the reference's world velocity -> zero errors,
    # whatever the reference pose's heading is (body velocity rotated to world inside)
    from traj_utils import VY_COL, tracking_errors

    ref = torch.zeros(3, 7)
    ref[:, X_COL] = torch.tensor([1.0, 2.0, 3.0])
    ref[:, TH_COL] = torch.tensor([0.0, math.pi / 2, -2.0])
    ref[:, VX_COL] = 0.5  # forward in the reference pose's frame
    ref[:, VY_COL] = 0.1
    th = ref[:, TH_COL]
    v_w = torch.stack([torch.cos(th) * 0.5 - torch.sin(th) * 0.1, torch.sin(th) * 0.5 + torch.cos(th) * 0.1], 1)
    p, v = tracking_errors(ref, ref[:, [X_COL, Y_COL]], v_w)
    assert torch.allclose(p, torch.zeros(3)) and torch.allclose(v, torch.zeros(3), atol=1e-6)
    # 0.3 m off in y, standing still -> pos err 0.09, vel err = |v_ref|^2 = 0.26
    p, v = tracking_errors(ref[:1], torch.tensor([[1.0, 0.3]]), torch.zeros(1, 2))
    assert abs(p.item() - 0.09) < 1e-6 and abs(v.item() - 0.26) < 1e-6


def test_time_scale():
    # with time scale s, sampling at reference time s*t must give s times the stored velocities, and the
    # positions reached after real time t must be the ones the unscaled reference reaches after s*t
    from traj_utils import scale_velocities

    with tempfile.TemporaryDirectory() as tmp:
        traj, traj_len, dt = load_trajectories(_write_refs(tmp), "cpu")
        idx = torch.tensor([0, 0])
        s = torch.tensor([[1.0], [2.0]])
        t_real = 1.0
        rows = scale_velocities(sample_table(traj, traj_len, dt, idx, s * t_real, loop=True), s)
        ref_fast = sample_table(traj, traj_len, dt, idx[:1], torch.tensor([[2.0]]), loop=True)
        assert torch.allclose(rows[1, 0, X_COL], ref_fast[0, 0, X_COL], atol=1e-5)
        sp = torch.linalg.norm(rows[:, 0, VX_COL:VX_COL + 2], dim=1)
        assert abs(sp[1].item() / sp[0].item() - 2.0) < 1e-3


def test_long_trajectory_clock_stays_on_samples():
    # a 300 s lap (as the train_own ones), the clock advanced like TrajectoryCommand._update_command (20 Hz, s = 1):
    # in float64 every query still lands on a table sample; the float32 clock drifted ~0.8 sample by 200 s
    with tempfile.TemporaryDirectory() as tmp:
        t = np.arange(30001) * 0.01
        table = np.zeros((t.size, 7))
        table[:, 0], table[:, X_COL] = t, np.sin(t)  # curved, so a shifted query gives a different value
        fn = os.path.join(tmp, "long.csv")
        np.savetxt(fn, table, delimiter=",", header="t,X,Y,theta,vx,vy,omega", comments="")
        traj, traj_len, dt = load_trajectories([fn], "cpu")
        assert dt == 0.01  # from the float64 time column, not the float32 table
        idx = torch.tensor([0])
        clock = torch.zeros(1, dtype=torch.float64)
        step_dt, scale = 18 / 360, torch.ones(1).double()
        for k in range(1, 6001):
            clock += step_dt * scale
            if k % 1000 == 0:
                rows = sample_table(traj, traj_len, dt, idx, clock.unsqueeze(1), loop=False)[0, 0]
                assert rows.dtype == traj.dtype
                assert abs(rows[X_COL].item() - traj[0, 5 * k, X_COL].item()) < 1e-6, k


def test_training_trajectories_feasible():
    import gen_training_trajectories as gt

    rng = np.random.default_rng(1)
    for _ in range(3):
        a = gt.make_trajectory(rng, 0.01, 0.5, 0.5, 0.5)
        assert np.allclose(a[0, 1:3], a[-1, 1:3], atol=1e-6)  # closed
        p, h = a[:, 1:3], 5
        dp = (np.roll(p[:-1], -h, 0) - np.roll(p[:-1], h, 0)) / (2 * h * 0.01)
        sp = np.linalg.norm(dp, axis=1)
        at = (np.roll(sp, -h) - np.roll(sp, h)) / (2 * h * 0.01)
        assert sp.max() < 0.51 and np.abs(at).max() < 0.55  # within the limits (+ numerical margin)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
