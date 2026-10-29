"""Demo video: the policy (or the LTV-MPC, --controller mpc) drives a circle, a figure eight and a rounded square one
after the other, with the reference path painted on the ground (green) and the driven path drawn behind the robot
(orange).

Same protocol as eval_tracking.py (one env per file of --traj_dir in sorted order, fixed assignment, s = 1, no noise /
DR / curriculum, start at the env origin with heading psi_r(0), one lap with loop=False unless --loop, the same
--start_offset). Every shape is filmed from a fixed oblique camera above it for one lap (+ --extra_s, = eval_tracking's
--hold); all envs are reset (wheels stopped too) at the start of every segment. The first segment of a run is the very
run of eval_tracking.py (bit for bit with the MPC); the later ones start from a reset, not a fresh simulation, and may
differ slightly, so for the analysis videos run one shape per process (--shapes <one>). With --duration the whole video is fast-forwarded uniformly (frames are dropped, the
simulation still runs at s = 1 and the overlay shows sim time) so that it lasts that many seconds.

    # FUJI tasks (legacy closed shapes of refs/test)
    python scripts/rsl_rl/play_shapes.py --task Mecanum-Tracking-Eval-v0 --checkpoint <model.pt> --loop \
        --out logs/videos/shapes.mp4 --headless --enable_cameras [--duration 30]
    # comparison with the MPC: the MPC side's shapes, exactly as eval_tracking.py runs them
    python scripts/rsl_rl/play_shapes.py --task Mecanum-Tracking-Torque-Eval-v0 --checkpoint <model.pt> \
        --traj_dir $R/test_fixed --out logs/videos/torque_test_fixed.mp4 --headless --enable_cameras [--duration 60]
    # the MPC (mpc_controller.py, eval_tracking.py's controller and options) instead of a policy
    python scripts/rsl_rl/play_shapes.py --task Mecanum-Tracking-Torque-Eval-v0 --controller mpc \
        --traj_dir $R/test_fixed --out logs/videos/mpc_test_fixed.mp4 --headless --enable_cameras
    # analysis mode: raw frames + logs per shape, then tools/compose_dynamic_video.py draws the panels
    python scripts/rsl_rl/play_shapes.py --task Mecanum-Tracking-Torque-Eval-v0 --controller mpc \
        --traj_dir $R/test_dynamic --raw_dir logs/videos/raw/mpc_test_dynamic --headless --enable_cameras
"""

import argparse
import os
import sys

from isaaclab.app import AppLauncher

REF_TEST = os.path.join(
    os.path.dirname(__file__), "..", "..", "source", "mecanum_tracking_ws", "mecanum_tracking_ws", "tasks",
    "manager_based", "mecanum_tracking", "refs", "test",
)
LABELS = {
    "circle": "Kör", "figure_eight": "Nyolcas", "rounded_square": "Lekerekített négyzet", "sharp_square": "Sarkos négyzet",
    "ellipse": "Ellipszis", "figure8": "Nyolcas", "square": "Lekerekített négyzet",  # the MPC side's shapes
    "circle_small": "Kis kör", "slalom": "Szlalom", "rose": "Háromszirmú rózsa",  # refs/test_dynamic
    "stop_go": "Egyenes: indulás és megállás",
}
LEGACY_SHAPES = ["circle", "figure_eight", "rounded_square"]

parser = argparse.ArgumentParser()
parser.add_argument("--task", default="Mecanum-Tracking-Eval-v0")
parser.add_argument("--controller", choices=["policy", "mpc"], default="policy")
parser.add_argument("--out", default=None, help="output .mp4")
parser.add_argument(
    "--raw_dir", default=None,
    help="analysis mode instead of --out: per shape <raw_dir>/<shape>.mp4 (no overlay or title frame, one frame per "
    "control step from the reset: frame j shows row j of the log) and <raw_dir>/<shape>.csv (eval_tracking.py's log "
    "format), for tools/compose_dynamic_video.py",
)
parser.add_argument("--traj_dir", default=REF_TEST, help="folder of the test csv files (default: refs/test)")
parser.add_argument(
    "--shapes", nargs="+", default=None,
    help="file names (without .csv) of --traj_dir, filmed in this order; default: every file (refs/test: "
    + ", ".join(LEGACY_SHAPES) + ")",
)
parser.add_argument("--loop", action="store_true", help="closed trajectories without ramps (legacy refs/test)")
parser.add_argument("--start_offset", type=float, nargs=3, default=[0.0, 0.0, 0.0], metavar=("DX", "DY", "DPSI"),
                    help="initial error, as in eval_tracking.py (scenario 2: 0.3 0 0.52)")
parser.add_argument("--time_scale", type=float, default=1.0)
parser.add_argument("--extra_s", type=float, default=1.0, help="keep filming this long after the lap [s]")
parser.add_argument("--title_s", type=float, default=1.0, help="still title frame before each shape [s]")
parser.add_argument("--duration", type=float, default=None, help="total video length [s] (fast-forward); default: real time")
parser.add_argument("--fps", type=float, default=30.0, help="output fps when --duration is set")
parser.add_argument("--max_steps", type=int, default=None, help="debug: cap the steps per shape")
parser.add_argument("--width", type=int, default=1920)
parser.add_argument("--height", type=int, default=1080)
parser.add_argument("--cam_zoom", type=float, default=1.0, help="camera distance divided by this")
parser.add_argument("--cam_elev", type=float, default=51.7, help="camera elevation above the ground [deg]")
parser.add_argument(
    "--cam_window", type=float, default=None,
    help="frame only the reference of the first CAM_WINDOW seconds and the robot's start pose (initial-error clips); "
    "default: the whole path",
)
parser.add_argument("--line_radius", type=float, default=0.03, help="radius of the painted path [m] (trail: 0.6x)")
parser.add_argument("--marker_scale", type=float, default=1.0, help="size of the reference point markers")
parser.add_argument("--seed", type=int, default=0)
sys.path.insert(0, os.path.dirname(__file__))
import cli_args  # isort: skip  # noqa: E402
from mpc_controller import MPC_EXTRA, MPCController, add_mpc_args, robot_state  # isort: skip  # noqa: E402

add_mpc_args(parser)
cli_args.add_rsl_rl_args(parser)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
if args_cli.shapes is None:
    if os.path.samefile(args_cli.traj_dir, REF_TEST):
        args_cli.shapes = LEGACY_SHAPES
    else:
        args_cli.shapes = sorted(os.path.splitext(f)[0] for f in os.listdir(args_cli.traj_dir) if f.endswith(".csv"))
SHAPES = [(name, LABELS.get(name, name)) for name in args_cli.shapes]
if (args_cli.out is None) == (args_cli.raw_dir is None):
    parser.error("give exactly one of --out and --raw_dir")
if args_cli.raw_dir and args_cli.duration:
    parser.error("--raw_dir records in real time (no --duration)")
if args_cli.controller == "policy" and not args_cli.checkpoint:
    parser.error("--checkpoint <model.pt> is required for --controller policy")
args_cli.enable_cameras = True
simulation_app = AppLauncher(args_cli).app

import importlib.metadata as metadata  # noqa: E402
import tempfile  # noqa: E402
import time  # noqa: E402

import gymnasium as gym  # noqa: E402
import imageio  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg  # noqa: E402

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry  # noqa: E402

import mecanum_tracking_ws.tasks  # noqa: E402, F401
from mecanum_tracking_ws.tasks.manager_based.mecanum_tracking.tracking_env_cfg import robot_module  # noqa: E402

FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"


def overlay(frame: np.ndarray, title: str, lines: list[str]) -> np.ndarray:
    img = Image.fromarray(frame)
    draw = ImageDraw.Draw(img, "RGBA")
    h = img.height
    big, small = ImageFont.truetype(FONT_BOLD, h // 22), ImageFont.truetype(FONT, h // 40)
    pad = h // 60
    box_h = pad * 3 + big.size + len(lines) * (small.size + pad // 2)
    box_w = max([draw.textlength(title, font=big)] + [draw.textlength(s, font=small) for s in lines]) + 2 * pad
    draw.rounded_rectangle((pad, pad, pad + box_w, pad + box_h), radius=pad, fill=(0, 0, 0, 140))
    draw.text((2 * pad, 2 * pad), title, font=big, fill=(255, 255, 255))
    y = 2 * pad + big.size + pad
    for s in lines:
        draw.text((2 * pad, y), s, font=small, fill=(235, 235, 235))
        y += small.size + pad // 2
    # legend, bottom left
    leg = [((30, 215, 65), "referencia pálya"), ((255, 90, 0), "megtett út"), ((230, 25, 25), "aktuális ref. pont")]
    y = h - pad - len(leg) * (small.size + pad // 2) - pad
    draw.rounded_rectangle(
        (pad, y - pad, pad + 2 * pad + small.size + max(draw.textlength(t, font=small) for _, t in leg) + 2 * pad,
         h - pad), radius=pad, fill=(0, 0, 0, 140))
    for color, text in leg:
        draw.rectangle((2 * pad, y + small.size // 4, 2 * pad + small.size // 2 * 2, y + small.size // 4 * 3), fill=color)
        draw.text((3 * pad + small.size, y), text, font=small, fill=(235, 235, 235))
        y += small.size + pad // 2
    return np.asarray(img)


def main():
    # eval_tracking.py's layout: env j follows file j of the sorted folder, filmed or not
    files = sorted(os.path.abspath(os.path.join(args_cli.traj_dir, f)) for f in os.listdir(args_cli.traj_dir)
                   if f.endswith(".csv"))
    env_of = {os.path.splitext(os.path.basename(f))[0]: j for j, f in enumerate(files)}
    tables = [np.loadtxt(f, delimiter=",", skiprows=1) for f in files]
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=len(files))
    env_cfg.scene.robot.spawn.usd_dir = f"/tmp/IsaacLab/usd_play_{os.getpid()}"  # parallel runs: see eval_tracking.py
    cmd = env_cfg.commands.ref_traj
    cmd.traj_files, cmd.fixed_assignment, cmd.random_start = files, True, False
    cmd.initial_time_scale = args_cli.time_scale
    cmd.loop = args_cli.loop
    cmd.start_offset = tuple(args_cli.start_offset)
    # the reference path painted on the ground: dense green discs -> a continuous line
    cmd.path_vis_num_envs = len(files)
    cmd.path_vis_points = 2500
    cmd.path_visualizer_cfg.markers["path"] = sim_utils.CylinderCfg(
        radius=args_cli.line_radius, height=0.004, visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.1, 0.85, 0.25))
    )
    for marker in cmd.ref_visualizer_cfg.markers.values():
        marker.radius *= args_cli.marker_scale
    env_cfg.curriculum.speed = None
    env_cfg.observations.policy.enable_corruption = False
    env_cfg.events.physics_material = None
    env_cfg.events.base_mass = None
    env_cfg.events.reset_base.params["pose_range"] = {"x": (0.0, 0.0), "y": (0.0, 0.0), "yaw": (0.0, 0.0)}
    env_cfg.seed = args_cli.seed
    laps = [tab[-1, 0] / args_cli.time_scale for tab in tables]
    env_cfg.episode_length_s = max(laps) + args_cli.extra_s + 5.0  # no time-out during a segment
    env_cfg.viewer.origin_type = "world"  # static camera, placed per shape below
    env_cfg.viewer.resolution = (args_cli.width, args_cli.height)

    if args_cli.controller == "mpc":
        env = RslRlVecEnvWrapper(gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array"), clip_actions=None)
    else:
        agent_cfg = load_cfg_from_registry(args_cli.task, "rsl_rl_cfg_entry_point")
        agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, metadata.version("rsl-rl-lib"))
        env = RslRlVecEnvWrapper(
            gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array"), clip_actions=agent_cfg.clip_actions
        )
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
        runner.load(args_cli.checkpoint)
        # the deployed form, as in eval_tracking.py: TorchScript on the CPU, one observation at a time (the batched GPU
        # policy differs by ~1e-8, which the contacts grow to ~1 mm within seconds: the clip would not be the eval run)
        jit_dir = tempfile.mkdtemp()
        runner.export_policy_to_jit(path=jit_dir, filename="policy.pt")
        cpu_policy = torch.jit.load(os.path.join(jit_dir, "policy.pt"), map_location="cpu").eval()
        torch.set_num_threads(1)

    uenv = env.unwrapped
    robot = uenv.scene["robot"]
    term = uenv.command_manager.get_term("ref_traj")
    dt = uenv.step_dt
    # the common log of eval_tracking.py (MPC wheel order and signs)
    rb = robot_module(env_cfg.robot_name)
    wheel_ids, _ = robot.find_joints(rb.WHEEL_JOINT_NAMES, preserve_order=True)
    wheel_sign = torch.tensor(rb.WHEEL_JOINT_SIGNS, device=uenv.device)
    torque_term = uenv.action_manager.get_term("wheel_torque") if "wheel_torque" in uenv.action_manager.active_terms \
        else None
    if args_cli.controller == "mpc" and torque_term is None:
        raise SystemExit("the MPC commands wheel torques: use a Torque task")
    origins = uenv.scene.env_origins[:, :2].cpu().numpy()
    fps = 1.0 / dt
    trail_vis = VisualizationMarkers(
        VisualizationMarkersCfg(
            prim_path="/Visuals/robot_trail",
            markers={
                "trail": sim_utils.CylinderCfg(
                    radius=0.6 * args_cli.line_radius, height=0.004,
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.25, 0.0)),
                )
            },
        )
    )

    steps = [int((lap + args_cli.extra_s) / dt) for lap in laps]
    if args_cli.max_steps:
        steps = [min(n, args_cli.max_steps) for n in steps]
    if args_cli.duration:
        out_fps = args_cli.fps
        filmed_s = sum(steps[env_of[name]] for name, _ in SHAPES) * dt
        speed = filmed_s / (args_cli.duration - len(SHAPES) * args_cli.title_s)
    else:
        out_fps, speed = fps, 1.0
    if args_cli.raw_dir:
        os.makedirs(args_cli.raw_dir, exist_ok=True)
    else:
        os.makedirs(os.path.dirname(os.path.abspath(args_cli.out)), exist_ok=True)
        writer = imageio.get_writer(args_cli.out, fps=out_fps, codec="libx264", quality=8, macro_block_size=8)
    if args_cli.controller == "mpc":
        S = "off" if args_cli.mpc_S <= 0 else f"{args_cli.mpc_S:g}"
        who = f"LTV-MPC: {args_cli.mpc_terminal} terminal, S = {S}" + (
            f", Q_path {args_cli.mpc_Q_path[0]:g} / {args_cli.mpc_Q_path[1]:g}" if args_cli.mpc_Q_path else "") + (
            ", soft wheel-speed limit" if args_cli.mpc_wheel_limit else "")
    else:
        who = f"checkpoint: {os.path.relpath(args_cli.checkpoint)}"
    summary = []

    for seg, (name, label) in enumerate(SHAPES):
        i = env_of[name]
        with torch.inference_mode():  # the buffers were created as inference tensors in the previous segment
            if seg > 0:  # the reset events only place the base: stop the wheels of the previous segment as well
                robot.write_joint_state_to_sim(robot.data.default_joint_pos, robot.data.default_joint_vel)
            env.reset()
            obs = env.get_observations()
        # camera: oblique, from -y, framing the painted path of env i
        ref = tables[i][:, 1:3]
        a = term.anchor[i].cpu().numpy()  # yaw 0 without a start_offset
        c, s = np.cos(a[2]), np.sin(a[2])
        if args_cli.cam_window:  # the start of the path and the robot's start pose (in the path frame)
            rp = robot.data.root_pos_w[i, :2].cpu().numpy() - a[:2]
            ref = np.vstack([ref[tables[i][:, 0] <= args_cli.cam_window], [c * rp[0] + s * rp[1], -s * rp[0] + c * rp[1]]])
        lo, hi = ref.min(0), ref.max(0)
        mid = (lo + hi) / 2
        center = np.array([a[0] + c * mid[0] - s * mid[1], a[1] + s * mid[0] + c * mid[1], 0.0])
        size = max(hi[0] - lo[0], 1.6 * (hi[1] - lo[1]))
        dist = 1.21 * (0.95 * size + 1.3) / args_cli.cam_zoom  # eye to path centre
        elev = np.radians(args_cli.cam_elev)
        uenv.viewport_camera_controller.update_view_location(
            eye=center + dist * np.array([0.0, -np.cos(elev), np.sin(elev)]),
            lookat=center + np.array([0.0, 0.15 / args_cli.cam_zoom, 0.0]),
        )
        trail = []
        trail_vis.visualize(translations=torch.tensor([[0.0, 0.0, -10.0]], device=uenv.device))  # hide
        # warm-up renders so the new camera pose and the painted path are in the frame
        # (env.render, not sim.render: the rgb annotator only exists after the first env.render, and its first
        # frames come out black); in a fresh process the ground texture also streams in for a while (wall time)
        t_warm = time.perf_counter()
        for n_warm in range(1000):
            uenv.render()
            if n_warm >= (10 if seg == 0 else 3) and (seg > 0 or time.perf_counter() - t_warm > 3.0):
                break
        frame = uenv.render()
        n_steps = steps[i]
        if args_cli.raw_dir:
            writer = imageio.get_writer(os.path.join(args_cli.raw_dir, f"{name}.mp4"), fps=fps, codec="libx264",
                                        quality=10, macro_block_size=8)
            writer.append_data(frame)  # frame 0: the state of log row 0
        if args_cli.controller == "mpc":
            mpc = MPCController(len(files), rb, env_cfg, dt, args_cli)  # fresh (u_applied, warm starts) per shape
        v_ref = args_cli.time_scale * np.hypot(tables[i][:, 4], tables[i][:, 5]).max()
        heading = "fix irány" if np.ptp(tables[i][:, 3]) < 1e-6 else "tangens irány"
        info = [f"{n_steps * dt:.1f} s, v_ref = {v_ref:.2f} m/s, {heading}", who]
        if any(args_cli.start_offset):
            dx, dy, dpsi = args_cli.start_offset
            info.append(f"kezdeti hiba: {100 * np.hypot(dx, dy):.0f} cm, {np.degrees(dpsi):.0f}°")
        if speed != 1.0:
            info.append(f"lejátszás: {speed:.2f}× gyorsítva")
        for _ in range(0 if args_cli.raw_dir else round(args_cli.title_s * out_fps)):
            writer.append_data(overlay(frame, label, info))

        errs, lost, ended, rows = [], False, False, []
        n_written = 0  # frames of this segment; frame j shows sim time j * speed / out_fps
        for k in range(n_steps):
            with torch.inference_mode():
                # state and reference BEFORE the step, as eval_tracking.py logs them
                ref = term.ref_now[i].clone()
                x = robot_state(robot)
                w = (robot.data.joint_vel[i, wheel_ids] * wheel_sign).cpu().numpy()
                extra = []
                if args_cli.controller == "mpc":
                    u, infos = mpc(x, term.preview_w.double().cpu().numpy())
                    compute_ms, extra = infos[i]["step_ms"], mpc.extra(infos)[i]
                    actions = torch.tensor(u, dtype=torch.float32, device=uenv.device) * wheel_sign
                    actions /= torque_term.cfg.scale
                else:
                    obs_cpu = obs["policy"].cpu()
                    acts = []
                    for e in range(obs_cpu.shape[0]):  # not k: the step counter of this loop
                        t0 = time.perf_counter()
                        acts.append(cpu_policy(obs_cpu[e : e + 1]))
                        if e == i:
                            compute_ms = (time.perf_counter() - t0) * 1e3  # env i's forward pass, as eval_tracking.py
                    actions = torch.cat(acts).to(uenv.device)
                obs, _, dones, _ = env.step(actions)
                if torque_term is not None:
                    u_cmd = (torque_term.processed_actions[i] * wheel_sign).cpu().numpy()
                else:
                    u_cmd = actions[i].cpu().numpy()
                tau = (robot.data.applied_torque[i, wheel_ids] * wheel_sign).cpu().numpy()
            if bool(dones[i]):
                lost = lost or bool(uenv.termination_manager.get_term("deviation")[i])
                trail = []
            if not ended:  # as eval_tracking.py: no rows after the env's first reset, failed = 1 on its last row
                th, o = float(ref[3]), origins[i]
                c, s = np.cos(th), np.sin(th)
                ref_v = [c * float(ref[4]) - s * float(ref[5]), s * float(ref[4]) + c * float(ref[5])]
                rows.append([k * dt, x[i, 0] - o[0], x[i, 1] - o[1], *x[i, 2:], float(ref[1]) - o[0],
                             float(ref[2]) - o[1], th, *ref_v, float(ref[6]), *u_cmd, *tau, *w, compute_ms,
                             float(lost)] + extra)
            ended = ended or bool(dones[i])
            # after the step the clock and the robot state refer to the same instant
            pos = robot.data.root_pos_w[i].cpu().numpy()
            r = term.ref_now[i].cpu().numpy()
            err = float(np.hypot(r[1] - pos[0], r[2] - pos[1]))
            if k * dt <= laps[i]:
                errs.append(err)
            trail.append([pos[0], pos[1], 0.016])
            if args_cli.raw_dir:
                if k + 1 < n_steps:  # frame k + 1 = the state of row k + 1
                    trail_vis.visualize(translations=torch.tensor(trail, device=uenv.device))
                    writer.append_data(uenv.render())
                continue
            if (k + 1) * dt < n_written * speed / out_fps:
                continue  # fast-forward: this step gets no frame
            n_written += 1
            trail_vis.visualize(translations=torch.tensor(trail, device=uenv.device))
            frame = uenv.render()
            lines = [
                f"t = {(k + 1) * dt:5.1f} s   pozícióhiba: {1000 * err:5.1f} mm",
                f"átlag: {1000 * np.mean(errs):5.1f} mm   max: {1000 * np.max(errs):5.1f} mm",
            ]
            if lost:
                lines.append("ELVESZTETTE A PÁLYÁT (eltérés > 1 m, reset)")
            writer.append_data(overlay(frame, label, lines))
        summary.append(f"{name}: mean {1000 * np.mean(errs):.2f} mm, max {1000 * np.max(errs):.2f} mm, lost={lost}")
        print(f"[shapes] {summary[-1]}", flush=True)
        if args_cli.raw_dir:
            writer.close()
            n_u = len(rows[0]) - 23 - len(extra)  # 4 torques (twist ablation: 3 raw actions)
            header = (["t", "x", "y", "yaw", "vx", "vy", "wz", "x_ref", "y_ref", "yaw_ref", "vx_ref", "vy_ref",
                       "wz_ref"] + [f"u_{j}" for j in range(n_u)] + [f"tau_{j}" for j in range(4)]
                      + [f"w_{j}" for j in range(4)] + ["compute_ms", "failed"]
                      + (MPC_EXTRA if args_cli.controller == "mpc" else []))
            np.savetxt(os.path.join(args_cli.raw_dir, f"{name}.csv"), np.array(rows), delimiter=",",
                       header=",".join(header), comments="", fmt="%.9g")

    if args_cli.raw_dir:
        with open(os.path.join(args_cli.raw_dir, "play_args.txt"), "a") as f:  # one line per run
            f.write(" ".join(sys.argv) + "\n")
        print(f"[shapes] raw videos and logs -> {args_cli.raw_dir} ({fps:.2f} fps)", flush=True)
    else:
        writer.close()
        print(f"[shapes] video -> {args_cli.out} ({out_fps:.2f} fps, {speed:.2f}x)", flush=True)
    print("[shapes] " + " | ".join(summary), flush=True)
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
