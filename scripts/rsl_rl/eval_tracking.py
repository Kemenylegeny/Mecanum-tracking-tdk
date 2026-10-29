"""Evaluate a controller on a fixed test set and write the COMMON log format (tools/metrics.py).

Both controllers of the comparison run HERE, in the same Isaac Lab env (docs/OSSZEHANGOLAS.md, 2.1): the same plant,
references, start states, resets, termination and log. Only the controller differs:
    --controller policy   the trained policy (exported to TorchScript, evaluated on the CPU, one env at a time)
    --controller mpc      the LTV-MPC (mecanum_tracking_ws/mpc/lmpc.py = Mecanum-Robot-MPC-TDK's LinearMPCv2 + DARE
                          terminal weight; S and the soft wheel-speed limit off by default), CPU, one env at a time
One env per test trajectory (fixed_assignment), one lap each (the reference stops at its end: loop=False), then
--hold seconds at the end pose; no noise / DR / curriculum.

The MPC (inputs, options) lives in mpc_controller.py, shared with play_shapes.py.

    R=source/mecanum_tracking_ws/mecanum_tracking_ws/tasks/manager_based/mecanum_tracking/refs
    python scripts/rsl_rl/eval_tracking.py --controller mpc --out logs/eval/mpc/test_fixed --traj_dir $R/test_fixed --headless
    python scripts/rsl_rl/eval_tracking.py --controller policy --checkpoint <model.pt> --out logs/eval/rl_seed0/test_fixed \
        --traj_dir $R/test_fixed --headless
    # checkpoints trained until 2026-10-27 (last-torque observation): --task Mecanum-Tracking-Torque-Offset-Eval-v0
    # scenario 2: 30 cm / 30 deg initial error (both controllers):  --start_offset 0.3 0.0 0.52

Then:  python scripts/tools/metrics.py --group RL logs/eval/rl_seed*/test_fixed/circle.csv --group MPC logs/eval/mpc/test_fixed/circle.csv
"""

import argparse
import glob
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", default="Mecanum-Tracking-Torque-Eval-v0")
parser.add_argument("--controller", choices=["policy", "mpc"], default="policy")
parser.add_argument("--out", required=True)
parser.add_argument("--traj_dir", required=True, help="folder of the TEST csv files (never the training ones)")
parser.add_argument("--time_scale", type=float, default=1.0)
parser.add_argument("--start_offset", type=float, nargs=3, default=[0.0, 0.0, 0.0], metavar=("DX", "DY", "DPSI"),
                    help="initial error: robot pose relative to the start pose [m, m, rad] (scenario 2: 0.3 0 0.52)")
parser.add_argument("--hold", type=float, default=1.0, help="seconds recorded after the end of the reference")
parser.add_argument("--loop", action="store_true", help="closed trajectories without ramps (legacy refs/test)")
parser.add_argument("--seed", type=int, default=0)
sys.path.insert(0, os.path.dirname(__file__))  # next to play.py / cli_args.py (scripts/rsl_rl/)
import cli_args  # isort: skip  # noqa: E402
from mpc_controller import MPC_EXTRA, MPCController, add_mpc_args, robot_state  # isort: skip  # noqa: E402

add_mpc_args(parser)
cli_args.add_rsl_rl_args(parser)  # adds --checkpoint (the model.pt to evaluate)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
if args_cli.controller == "policy" and not args_cli.checkpoint:
    parser.error("--checkpoint <model.pt> is required for --controller policy")
simulation_app = AppLauncher(args_cli).app

import importlib.metadata as metadata  # noqa: E402
import tempfile  # noqa: E402
import time  # noqa: E402

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry  # noqa: E402

import mecanum_tracking_ws.tasks  # noqa: E402, F401
from mecanum_tracking_ws.tasks.manager_based.mecanum_tracking.tracking_env_cfg import robot_module  # noqa: E402

def main():
    files = sorted(glob.glob(os.path.join(args_cli.traj_dir, "*.csv")))
    names = [os.path.splitext(os.path.basename(f))[0] for f in files]
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=len(files))
    # own folder for the URDF -> USD conversion: the default /tmp/IsaacLab/usd_<second>_<random> repeats across
    # processes started in the same second (the env seeds `random` before the scene is built), and parallel runs
    # then overwrite each other's USD ("Failed to find an articulation")
    env_cfg.scene.robot.spawn.usd_dir = f"/tmp/IsaacLab/usd_eval_{os.getpid()}"
    # --- the evaluation protocol: identical for both controllers ---
    cmd = env_cfg.commands.ref_traj
    cmd.traj_files, cmd.fixed_assignment, cmd.random_start = files, True, False
    cmd.initial_time_scale = args_cli.time_scale
    cmd.loop = args_cli.loop
    cmd.start_offset = tuple(args_cli.start_offset)
    env_cfg.curriculum.speed = None  # fixed time scale (already off in the Eval task)
    env_cfg.observations.policy.enable_corruption = False  # noise-free state, like the MPC
    env_cfg.events.physics_material = None  # nominal friction
    env_cfg.events.base_mass = None  # nominal mass (= the MPC's model)
    env_cfg.events.reset_base.params["pose_range"] = {"x": (0.0, 0.0), "y": (0.0, 0.0), "yaw": (0.0, 0.0)}
    env_cfg.seed = args_cli.seed
    rb = robot_module(env_cfg.robot_name)
    # one lap per trajectory (in sim time) + hold; the episode outlasts the longest one, so no env times out
    ends = [np.loadtxt(f, delimiter=",", skiprows=1, usecols=0)[-1] / args_cli.time_scale for f in files]
    env_cfg.episode_length_s = max(ends) + args_cli.hold + 1.0
    n_cols = len(open(files[0]).readline().split(","))

    if args_cli.controller == "mpc":
        if n_cols < 11:
            raise SystemExit("the MPC needs the reference torques: tau_1..4 columns (tools/export_mpc_references.py)")
        if args_cli.time_scale != 1.0:
            raise SystemExit("the reference torques hold only at time scale 1")
        env = RslRlVecEnvWrapper(gym.make(args_cli.task, cfg=env_cfg), clip_actions=None)
    else:
        agent_cfg = load_cfg_from_registry(args_cli.task, "rsl_rl_cfg_entry_point")
        agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, metadata.version("rsl-rl-lib"))
        env = RslRlVecEnvWrapper(gym.make(args_cli.task, cfg=env_cfg), clip_actions=agent_cfg.clip_actions)

    uenv = env.unwrapped
    robot = uenv.scene["robot"]
    term = uenv.command_manager.get_term("ref_traj")
    wheel_ids, _ = robot.find_joints(rb.WHEEL_JOINT_NAMES, preserve_order=True)
    wheel_sign = torch.tensor(rb.WHEEL_JOINT_SIGNS, device=uenv.device)
    torque_term = uenv.action_manager.get_term("wheel_torque") if "wheel_torque" in uenv.action_manager.active_terms \
        else None
    origins = uenv.scene.env_origins[:, :2].cpu().numpy()

    if args_cli.controller == "mpc":
        if torque_term is None:
            raise SystemExit("the MPC commands wheel torques: use a Torque task")
        mpc = MPCController(len(files), rb, env_cfg, uenv.step_dt, args_cli)
    else:
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
        runner.load(args_cli.checkpoint)
        gpu_policy = runner.get_inference_policy(device=uenv.device)
        # the deployed form: TorchScript on the CPU, one observation at a time (timed like the MPC)
        jit_dir = tempfile.mkdtemp()
        runner.export_policy_to_jit(path=jit_dir, filename="policy.pt")
        cpu_policy = torch.jit.load(os.path.join(jit_dir, "policy.pt"), map_location="cpu").eval()
        torch.set_num_threads(1)

    n_steps = int(np.ceil((max(ends) + args_cli.hold) / uenv.step_dt)) + 1
    ends_t = torch.tensor(ends, device=uenv.device)
    rows = [[] for _ in files]
    failed = torch.zeros(len(files), dtype=torch.bool, device=uenv.device)
    done_any = torch.zeros_like(failed)

    obs = env.get_observations()
    for k in range(n_steps):
        with torch.inference_mode():
            # state and reference BEFORE the step, at the same instant (the observed state, the current reference
            # point). Copies: env.step overwrites the command's buffers in place.
            ref = term.ref_now.clone()
            preview = term.preview_w.double().cpu().numpy()
            x = robot_state(robot)
            w = (robot.data.joint_vel[:, wheel_ids] * wheel_sign).cpu().numpy()  # MPC sign convention
            extra = None
            if args_cli.controller == "mpc":
                u, infos = mpc(x, preview)
                compute_ms = [info["step_ms"] for info in infos]
                extra = mpc.extra(infos)
                joint_u = torch.tensor(u, dtype=torch.float32, device=uenv.device) * wheel_sign
                actions = joint_u / torque_term.cfg.scale
            else:
                obs_cpu = obs["policy"].cpu()
                acts, compute_ms = [], []
                for i in range(len(files)):
                    t0 = time.perf_counter()
                    acts.append(cpu_policy(obs_cpu[i : i + 1]))
                    compute_ms.append((time.perf_counter() - t0) * 1e3)
                actions = torch.cat(acts).to(uenv.device)
                if k == 0:
                    diff = (gpu_policy(obs) - actions).abs().max().item()
                    print(f"[eval] exported CPU policy vs. GPU inference policy: max |diff| = {diff:.2e}")
            th = ref[:, 3]
            vx_ref = torch.cos(th) * ref[:, 4] - torch.sin(th) * ref[:, 5]
            vy_ref = torch.sin(th) * ref[:, 4] + torch.cos(th) * ref[:, 5]
            ref_np = torch.stack([ref[:, 1], ref[:, 2], th, vx_ref, vy_ref, ref[:, 6]], 1).double().cpu().numpy()
            obs, _, dones, _ = env.step(actions)
            # the commanded torques of this step (after the clip) and what the motor model applied
            if torque_term is not None:
                u_cmd = (torque_term.processed_actions * wheel_sign).cpu().numpy()
            else:
                u_cmd = actions.cpu().numpy()
            tau = (robot.data.applied_torque[:, wheel_ids] * wheel_sign).cpu().numpy()
            t = k * uenv.step_dt
            in_lap = t <= ends_t + args_cli.hold
            lost = uenv.termination_manager.get_term("deviation")
            failed |= lost & ~done_any & in_lap
            for i in range(len(files)):
                if done_any[i] or not in_lap[i]:
                    continue
                o = origins[i]
                rows[i].append([t, x[i, 0] - o[0], x[i, 1] - o[1], *x[i, 2:], ref_np[i, 0] - o[0], ref_np[i, 1] - o[1],
                                *ref_np[i, 2:], *u_cmd[i], *tau[i], *w[i], compute_ms[i], float(failed[i].item())]
                               + (extra[i] if extra is not None else []))
            done_any |= dones.bool()
        if bool((done_any | ~in_lap).all()):
            break

    os.makedirs(args_cli.out, exist_ok=True)
    n_u = u_cmd.shape[1]  # 4 torques (twist ablation: 3 raw actions)
    header = (["t", "x", "y", "yaw", "vx", "vy", "wz", "x_ref", "y_ref", "yaw_ref", "vx_ref", "vy_ref", "wz_ref"]
              + [f"u_{j}" for j in range(n_u)] + [f"tau_{j}" for j in range(4)] + [f"w_{j}" for j in range(4)]
              + ["compute_ms", "failed"] + (MPC_EXTRA if args_cli.controller == "mpc" else []))
    for name, r in zip(names, rows):
        np.savetxt(os.path.join(args_cli.out, f"{name}.csv"), np.array(r), delimiter=",", header=",".join(header),
                   comments="", fmt="%.9g")
    with open(os.path.join(args_cli.out, "eval_args.txt"), "w") as f:
        f.write(" ".join(sys.argv) + "\n")
    print(f"[eval] {args_cli.controller}: {len(files)} trajectories -> {args_cli.out}; "
          f"lost: {[n for n, f in zip(names, failed) if f]}")
    # compute_ms: the MPC's step time / the policy's single-observation CPU forward pass, same machine
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
