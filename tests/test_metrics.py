"""Tests for tools/metrics.py on synthetic logs: python tests/test_metrics.py"""

import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts", "tools"))
import metrics  # noqa: E402


def _log(offset_lateral=0.0, lag=0.0, n=500):
    # reference: straight line along +x at 0.5 m/s; the robot is `lag` behind and `offset_lateral` to the left
    t = np.arange(n) * 0.02
    x_ref = 0.5 * t
    return {
        "t": t, "x": x_ref - lag, "y": np.full(n, offset_lateral), "yaw": np.zeros(n),
        "vx": np.full(n, 0.5), "vy": np.zeros(n), "wz": np.zeros(n),
        "x_ref": x_ref, "y_ref": np.zeros(n), "vx_ref": np.full(n, 0.5), "vy_ref": np.zeros(n),
        "u_0": np.full(n, 0.5), "u_1": np.zeros(n), "u_2": np.zeros(n),
        "compute_ms": np.full(n, 1.0), "failed": np.zeros(n),
    }


def test_contour_lag_split():
    m = metrics.run_metrics(_log(offset_lateral=0.05, lag=0.10))
    assert abs(m["contour_rmse_m"] - 0.05) < 1e-9  # lateral offset -> contouring error only
    assert abs(m["lag_mean_m"] - 0.10) < 1e-9  # behind -> positive lag
    assert abs(m["pos_rmse_m"] - np.hypot(0.05, 0.10)) < 1e-9
    assert m["strafe_share"] < 1e-9 and m["action_rate_mean"] == 0.0


def test_csv_roundtrip_and_groups():
    with tempfile.TemporaryDirectory() as tmp:
        files = []
        for i, off in enumerate([0.01, 0.02, 0.03]):
            log = _log(offset_lateral=off)
            fn = os.path.join(tmp, f"seed{i}.csv")
            np.savetxt(fn, np.stack(list(log.values()), 1), delimiter=",", header=",".join(log), comments="")
            files.append(fn)
        m = metrics.run_metrics(metrics.load_log(files[1]))
        assert abs(m["pos_rmse_m"] - 0.02) < 1e-6
        lo, hi = metrics.bootstrap_ci(np.array([0.01, 0.02, 0.03]))
        assert lo <= 0.02 <= hi


def test_heading_wheel_speed_and_window():
    log = _log()
    n = len(log["t"])
    log["yaw_ref"] = np.zeros(n)
    log["yaw"] = np.full(n, 0.1)
    log["wz_ref"] = np.zeros(n)
    for j in range(4):
        log[f"w_{j}"] = np.full(n, 5.0)
    log["w_2"][10:20] = 9.06  # 1 rad/s above the XM430 limit for 10 of 500 steps
    log["vx_ref"][400:] = 0.0  # the reference stops at t = 8 s: the rest is outside the window
    log["x"][450:] += 1.0
    m = metrics.run_metrics(log)
    assert abs(m["heading_rmse_rad"] - 0.1) < 1e-9 and abs(m["heading_max_rad"] - 0.1) < 1e-9
    assert abs(m["omega_exceed_max"] - 1.0) < 1e-9 and abs(m["omega_exceed_share"] - 10 / 400) < 1e-9
    assert m["pos_max_m"] < 1e-9


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
