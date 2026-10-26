"""The LTV-MPC of the RL comparison, and everything both controllers must agree on.

Copied from the MPC side's repository, Pvaradi05/Mecanum-Robot-MPC-TDK @ fe01f88 (2026-10-21), so that the MPC runs in
the same Isaac Lab env as the policy (scripts/rsl_rl/eval_tracking.py --controller mpc):

    params.py, dynamics.py, motor.py,          mecanum_mpc_common/*.py, unchanged
    wheel_speed.py, reference.py, conventions.py
    lmpc.py                                     mecanum_lmpc_v2/lmpc.py; imports made relative, plus the DARE terminal
                                                weight and the torque-change penalty S (both off by default)
    tables.py                                   new: references <-> the RL's CSV tables (t, X, Y, theta, vx, vy,
                                                omega, tau_1..4), and reference torques of any tabulated path

Pure numpy (+ quadprog for the QP): no ROS and no Isaac Lab, so the tools and tests import it without the simulator
(``sys.path`` -> this folder's parent, ``import mpc``). When the MPC side changes its code, copy the files again and
re-run tests/test_mpc.py.
"""
