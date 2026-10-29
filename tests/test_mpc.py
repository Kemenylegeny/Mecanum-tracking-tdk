"""Tests of the MPC side of the comparison (mecanum_tracking_ws/mpc), no Isaac Lab needed:
python tests/test_mpc.py  (or python -m pytest tests/)"""

import os
import sys

import numpy as np

_ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(_ROOT, "source", "mecanum_tracking_ws", "mecanum_tracking_ws"))
from mpc import dynamics, params, tables, wheel_dynamics  # noqa: E402
from mpc.lmpc import LinearMPCv2, solve_dare  # noqa: E402
from mpc.reference import Reference  # noqa: E402

URDF = os.path.join(_ROOT, "assets", "robots", "own_mecanum", "robot_measured.urdf")
P = params.from_urdf(URDF)
Q = np.diag([40000, 40000, 3283, 2500, 2500, 100.0])
R = np.diag([400.0] * 4)


def _plant(x, u, T, substeps=10):
    for _ in range(substeps):
        x = dynamics.rk4_step(x, u, T / substeps, P)
    return x


def test_measured_model():
    # the weighed robot (MPC side's URDF): m = 2.134 kg, geometry of the Isaac robot
    assert abs(P.m_s - 2.134) < 1e-3 and abs(P.R - 0.03028) < 1e-4
    assert abs(P.rho - 0.064) < 1e-9 and abs(P.l - 0.1526) < 1e-9 and np.all(P.wheel_sign == 1.0)


def test_dare_solves_riccati():
    ref = Reference(P, "circle", None, 0.1, "tangent")
    ref.anchor((0.0, 0.0, 0.0))
    x_r, u_r = ref.horizon(10.0, 0.05, 10)
    A, B = dynamics.rk4_jacobians(x_r[9], u_r[9], 0.05, P)
    Pd, it = solve_dare(A, B, Q, R)
    K = np.linalg.solve(R + B.T @ Pd @ B, B.T @ Pd @ A)
    assert it < 200
    assert np.abs(Q + A.T @ Pd @ (A - B @ K) - Pd).max() < 1e-6 * np.abs(Pd).max()
    assert np.all(np.linalg.eigvalsh(Pd) > 0) and np.abs(np.linalg.eigvals(A - B @ K)).max() < 1.0
    # warm start: the same problem converges at once
    assert solve_dare(A, B, Q, R, Pd)[1] <= 2


def test_torque_change_penalty():
    ref = Reference(P, "square", None, 0.1, "tangent")
    ref.anchor((0.0, 0.0, 0.0))
    x_r, u_r = ref.horizon(5.0, 0.05, 10)
    x = x_r[0] + np.array([0.02, -0.01, 0.1, 0.0, 0.0, 0.0])
    u_applied = np.zeros(4)
    jumps = []
    for S in (None, [1e4] * 4):
        u, _ = LinearMPCv2(P, terminal="dare", S=S).step(x, x_r, u_r, u_applied)
        jumps.append(np.linalg.norm(u - u_applied))
    assert jumps[1] < 0.5 * jumps[0]


def test_path_frame_weight():
    ref = Reference(P, "circle", None, 0.1, "fixed")
    ref.anchor((0.0, 0.0, 0.0))
    x_r, u_r = ref.horizon(12.0, 0.05, 10)
    x = x_r[0] + np.array([0.01, -0.02, 0.05, 0.0, 0.0, 0.0])
    for terminal in ("scaled", "dare"):
        # qa = qc = 40000 is the plain Q
        u0, _ = LinearMPCv2(P, terminal=terminal, S=[100.0] * 4).step(x, x_r, u_r, u_r[0])
        u1, _ = LinearMPCv2(P, terminal=terminal, S=[100.0] * 4, Q_path=(4e4, 4e4)).step(x, x_r, u_r, u_r[0])
        assert np.allclose(u0, u1, atol=1e-9)
    # the weight is along the direction of travel (not the fixed heading): t^T M t = qa, n^T M n = qc
    M = LinearMPCv2(P, Q_path=(9.0, 1.0)).path_weight(np.array([0.0, 0.2]))
    assert np.allclose(M, np.diag([1.0, 9.0]))
    # a larger along-track weight shrinks the predicted along-track error more than the cross-track one
    ctrl = LinearMPCv2(P, terminal="dare", S=[100.0] * 4, Q_path=(4e6, 4e4))
    _, info = ctrl.step(x, x_r, u_r, u_r[0])
    _, info0 = LinearMPCv2(P, terminal="dare", S=[100.0] * 4).step(x, x_r, u_r, u_r[0])
    def along_cross(info):
        e = info["predicted"][-1, :2] - x_r[-1, :2]
        t = x_r[-1, 3:5] / np.linalg.norm(x_r[-1, 3:5])
        return abs(t @ e), abs(t[0] * e[1] - t[1] * e[0])
    a1, _ = along_cross(info)
    a0, _ = along_cross(info0)
    assert a1 < a0


def _interp(tab, tq):
    n = len(tab)
    s = np.clip(tq / (tab[1, 0] - tab[0, 0]), 0, n - 1)
    i0 = np.minimum(np.floor(s).astype(int), n - 1)
    i1 = np.minimum(i0 + 1, n - 1)
    return tab[i0] + (s - i0)[:, None] * (tab[i1] - tab[i0])


def test_closed_loop_on_exported_table():
    # the eval script's path: table rows -> horizon_from_rows -> MPC; nominal plant -> sub-mm tracking
    ref = Reference(P, "figure8", None, 0.1, "tangent")
    tab = tables.reference_table(ref, 0.01, 0.05)
    mpc = LinearMPCv2(P, terminal="dare", S=[100.0] * 4)
    x = tables.horizon_from_rows(_interp(tab, np.zeros(1)), 0)[0][0]
    u_applied, err = np.zeros(4), []
    for k in range(400):
        x_r, u_r = tables.horizon_from_rows(_interp(tab, 0.05 * (k + np.arange(11))), 10)
        u, info = mpc.step(x, x_r, u_r, u_applied)
        assert info["status"] == 0
        err.append(np.hypot(*(x[:2] - x_r[0, :2])))
        u_applied, x = u, _plant(x, u, 0.05)
    assert max(err) < 1e-4


def test_tables_consistent():
    # the table torques of a sampled path (finite differences) = the analytic Reference's own u_r
    ref = Reference(P, "circle", None, 0.1, "fixed")
    tab = tables.reference_table(ref, 0.01, 0.05)
    tau = tables.path_torques(tab[:, 0], tab[:, 1:3], tab[:, 3], P, 0.05, periodic=False)
    assert np.abs(tau - tab[:, 7:]).max() < 1e-5
    # body-frame velocities and the wheel speeds of the check_limits table
    assert abs(np.abs(tables.table_wheel_speeds(tab, P)).max() - ref.check_limits(0.05, 1.95, 1.95, 8.06)[0]["omega"]) < 1e-2
    # periodic closed loop at constant speed with a tangent heading: psi_dot = v / r, tau_r = const
    r, v, dt = 0.5, 0.1, 0.01
    t = np.arange(int(round(2 * np.pi * r / v / dt)) + 1) * dt
    t = t * (2 * np.pi * r / v) / t[-1]
    a = v * t / r
    xy = np.c_[r * np.sin(a), r * (1 - np.cos(a))]
    tab = tables.path_table(t, xy, a, P, 0.05, periodic=True)
    assert np.allclose(tab[:, 6], v / r, atol=1e-6) and np.allclose(tab[:, 4], v, atol=1e-4)
    assert np.allclose(tab[:, 7:], tab[0, 7:], atol=1e-6)


def test_wheel_state_model():
    # the wheel-speed state model is dynamics.py in other coordinates: z_dot = d from_world(x) / dt, the same motion
    rng = np.random.default_rng(0)
    for _ in range(50):
        x = np.r_[rng.normal(0, 1, 2), rng.uniform(-np.pi, np.pi), rng.normal(0, 0.1, 2), rng.normal(0, 0.5)]
        u = rng.normal(0, 0.02, 4)
        z = wheel_dynamics.from_world(x, P)
        assert np.allclose(wheel_dynamics.to_world(z, P), x)
        xd, h = dynamics.f(x, u, P), 1e-7
        zd = (wheel_dynamics.from_world(x + h * xd, P) - wheel_dynamics.from_world(x - h * xd, P)) / (2 * h)
        assert np.abs(wheel_dynamics.f(z, u, P) - zd).max() < 1e-5
        # 1 s: both RK4s converge to the same solution (at T = 0.05 their truncation errors differ by up to ~0.02)
        xa, za = x.copy(), z.copy()
        for _ in range(200):
            xa, za = dynamics.rk4_step(xa, u, 0.005, P), wheel_dynamics.rk4_step(za, u, 0.005, P)
        assert np.abs(wheel_dynamics.to_world(za, P) - xa).max() < 1e-5
        # exact Jacobians (continuous and RK4) against finite differences
        A, B = wheel_dynamics.jacobians(z, u, P)
        An = np.stack([(wheel_dynamics.f(z + 1e-6 * e, u, P) - wheel_dynamics.f(z - 1e-6 * e, u, P)) / 2e-6
                       for e in np.eye(6)], 1)
        assert np.abs(A - An).max() < 1e-5
        Ad, Bd = wheel_dynamics.rk4_jacobians(z, u, 0.05, P)
        Bn = np.stack([(wheel_dynamics.rk4_step(z, u + 1e-6 * e, 0.05, P) - wheel_dynamics.rk4_step(z, u - 1e-6 * e, 0.05, P))
                       / 2e-6 for e in np.eye(4)], 1)
        assert np.abs(Bd - Bn).max() < 1e-5
    # wheel 4 follows from the rolling constraints; the encoders' slip part (1, 1, -1, -1) is dropped
    w = wheel_dynamics.wheel_speeds(z, P)
    assert np.isclose(w[3], w[0] + w[1] - w[2])
    assert np.allclose(wheel_dynamics.from_encoders(x, w + 0.5 * np.array([1, 1, -1, -1]), P), z)


def test_wheel_state_controller():
    # the same tracking problem in both states: nearly the same torques (the velocity error is measured in other
    # coordinates and the model linearised in them), and sub-mm closed-loop tracking on the nominal plant
    ref = Reference(P, "square", None, 0.1, "tangent")
    ref.anchor((0.0, 0.0, 0.0))
    x_r, u_r = ref.horizon(5.0, 0.05, 10)
    x = x_r[0] + np.array([0.02, -0.01, 0.1, 0.01, 0.0, 0.05])
    for limit in (False, True):
        u0, _ = LinearMPCv2(P, terminal="dare", wheel_speed_limit=limit).step(x, x_r, u_r, u_r[0])
        u1, info = LinearMPCv2(P, terminal="dare", wheel_speed_limit=limit, state="wheel").step(
            wheel_dynamics.from_world(x, P), wheel_dynamics.from_world(x_r, P), u_r, u_r[0])
        assert info["status"] == 0 and np.abs(u1 - u0).max() < 0.1 * np.abs(u0).max()
    tab = tables.reference_table(Reference(P, "figure8", None, 0.1, "tangent"), 0.01, 0.05)
    mpc = LinearMPCv2(P, terminal="dare", state="wheel")
    z = wheel_dynamics.from_world(tables.horizon_from_rows(_interp(tab, np.zeros(1)), 0)[0][0], P)
    u_applied, err = np.zeros(4), []
    for k in range(400):
        x_r, u_r = tables.horizon_from_rows(_interp(tab, 0.05 * (k + np.arange(11))), 10)
        u, info = mpc.step(z, wheel_dynamics.from_world(x_r, P), u_r, u_applied)
        assert info["status"] == 0
        err.append(np.hypot(*(z[:2] - x_r[0, :2])))
        u_applied = u
        for _ in range(10):
            z = wheel_dynamics.rk4_step(z, u, 0.005, P)
    assert max(err) < 1e-4


def test_horizon_unwraps_heading():
    rows = np.zeros((11, 11))
    rows[:, 3] = np.pi - 0.05 + 0.02 * np.arange(11)
    rows[:, 3] = (rows[:, 3] + np.pi) % (2 * np.pi) - np.pi  # wrapped in the middle of the horizon
    x_r, u_r = tables.horizon_from_rows(rows, 10)
    assert np.allclose(np.diff(x_r[:, 2]), 0.02) and u_r.shape == (10, 4)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
