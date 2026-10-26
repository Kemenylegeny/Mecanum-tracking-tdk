"""Plant check of the comparison task: constant wheel torques in Isaac Sim vs. the MPC's model (docs/OSSZEHANGOLAS.md,
2.1 point 4 and 5. step 1). Checks the wheel numbering and torque signs of the MPC (mpc/params.py: S_V, S_R) against
the Isaac wheel joints, and how close the roller physics is to the Chaplygin model the MPC predicts with.

Three envs, from rest (after settling on the ground with zero torque), the MPC's wheel order (FL, FR, RL, RR):
    forward   u = +a (1, 1, 1, 1)          model: X_ddot   = 4 a / R / m_e
    sideways  u = +a (-1, 1, 1, -1) = S_V   model: Y_ddot   = 4 a / R / m_e   (to the left)
    turn      u = +a (-1, 1, -1, 1) = S_R   model: psi_ddot = 4 a L / R / J_e (counter-clockwise)

    python scripts/torque_check.py --headless [--torque 0.01] [--steps 4]
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", default="Mecanum-Tracking-Torque-Play-v0")
parser.add_argument("--torque", type=float, default=0.01, help="a [Nm] per wheel")
parser.add_argument("--steps", type=int, default=4, help="control steps (50 ms) with the torque on")
parser.add_argument("--settle", type=int, default=20, help="control steps with zero torque first")
parser.add_argument("--vel_iters", type=int, default=None, help="diagnosis: PhysX velocity iterations of the robot")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
simulation_app = AppLauncher(args_cli).app

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402

import mecanum_tracking_ws.tasks  # noqa: E402, F401
from mecanum_tracking_ws.mpc import dynamics, params  # noqa: E402
from mecanum_tracking_ws.mpc.params import S_R, S_V  # noqa: E402
from mecanum_tracking_ws.robots import own_mecanum  # noqa: E402
from mecanum_tracking_ws.tasks.manager_based.mecanum_tracking.mdp.traj_utils import yaw_from_quat_wxyz  # noqa: E402
from mecanum_tracking_ws.tasks.manager_based.mecanum_tracking.tracking_env_cfg import TORQUE_ACTION_SCALE  # noqa: E402


def state(robot):
    """MPC state x = [X, Y, psi, X_dot, Y_dot, psi_dot] of the base_link origin (n, 6)."""
    d = robot.data
    return torch.cat([d.root_link_pos_w[:, :2], yaw_from_quat_wxyz(d.root_link_quat_w)[:, None],
                      d.root_link_lin_vel_w[:, :2], d.root_link_ang_vel_w[:, 2:3]], 1).cpu().numpy()


def main():
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=3)
    env_cfg.events.reset_base.params["pose_range"] = {"x": (0.0, 0.0), "y": (0.0, 0.0), "yaw": (0.0, 0.0)}
    env_cfg.episode_length_s = 1000.0
    env_cfg.terminations.deviation = None
    if args_cli.vel_iters is not None:
        env_cfg.scene.robot.spawn.articulation_props.solver_velocity_iteration_count = args_cli.vel_iters
    env = gym.make(args_cli.task, cfg=env_cfg)
    uenv = env.unwrapped
    robot = uenv.scene["robot"]
    p = params.from_urdf(str(own_mecanum.URDF_PATH))
    a = args_cli.torque
    u = np.array([np.ones(4), S_V, S_R]) * a  # MPC wheel order and sign convention
    joint_u = u * p.wheel_sign  # MPC torque -> joint effort (all wheel signs are +1 for this URDF)
    env.reset()
    zero = torch.zeros(3, 4, device=uenv.device)
    for _ in range(args_cli.settle):
        env.step(zero)
    x0 = state(robot)
    act = torch.tensor(joint_u / TORQUE_ACTION_SCALE, dtype=torch.float32, device=uenv.device)
    for _ in range(args_cli.steps):
        env.step(act)
    x1 = state(robot)
    T = args_cli.steps * uenv.step_dt
    applied = robot.data.applied_torque[:, :].cpu().numpy()
    wheel_ids, _ = robot.find_joints(own_mecanum.WHEEL_JOINT_NAMES, preserve_order=True)
    print(f"\nmodel: m_e {p.m_e:.4f} kg, J_e {p.J_e:.5f} kg m^2, R {p.R:.5f} m, L {p.L:.4f} m; "
          f"a = {a} Nm per wheel, {T:.2f} s from rest (after {args_cli.settle * uenv.step_dt:.1f} s settling)")
    for i, name in enumerate(["forward", "sideways", "turn"]):
        # model prediction: the MPC's RK4 step from the measured start state
        xm = x0[i].copy()
        for _ in range(args_cli.steps):
            xm = dynamics.rk4_step(xm, u[i], uenv.step_dt, p)
        c, s = np.cos(x0[i, 2]), np.sin(x0[i, 2])
        dv = x1[i, 3:5] - x0[i, 3:5]
        dvm = xm[3:5] - x0[i, 3:5]
        body = lambda v: np.array([c * v[0] + s * v[1], -s * v[0] + c * v[1]])  # noqa: E731
        print(f"  {name:9s} u = {np.round(u[i], 4).tolist()}  applied joint torques "
              f"{np.round(applied[i, wheel_ids], 4).tolist()}")
        print(f"            body velocity change  Isaac {np.round(body(dv), 4).tolist()} m/s,  model "
              f"{np.round(body(dvm), 4).tolist()} m/s;  psi_dot change Isaac {x1[i, 5] - x0[i, 5]:+.4f}, model "
              f"{xm[5] - x0[i, 5]:+.4f} rad/s")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
