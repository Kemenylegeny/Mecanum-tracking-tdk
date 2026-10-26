"""Mecanum trajectory tracking: the complete environment configuration (nodes 1-7).

The robot, the simulation settings and the action configs are those of the mecanum_ws navigation task (copied into
this project: robots/, mdp/actions.py, ActionsCfg / IkActionsCfg below); the scene (flat ground, no lidar), commands,
observations, rewards, terminations, curriculum and events are the tracking-specific parts.
Every choice is explained in docs/MAGYARAZAT.md (section numbers in the comments).

The comparison with the LTV-MPC (docs/OSSZEHANGOLAS.md, since 2026-10-23) is the ``Mecanum-Tracking-Torque-*`` task family
(:class:`MecanumTrackingTorqueEnvCfg`): our own robot with the weighed masses and XM430 torque motors, the policy
commands the four wheel torques at 20 Hz, observes the MPC's state and reference horizon (N = 10, T = 0.05 s), tracks
the heading too, and follows the MPC side's references (0.1 m/s); the training starts from random initial errors
(up to 0.4 m / 45 deg, no deviation termination; since 2026-10-26). The FUJI tasks keep their original settings
(51 Hz, K = 20, heading-free) so that their runs stay reproducible.
"""

import glob
import math
import os

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

from mecanum_tracking_ws.robots import mecanum, own_mecanum

from . import mdp

##
# Constants
##

REF_DIR = os.path.join(os.path.dirname(__file__), "refs")
"""Reference trajectories: refs/train (tools/gen_training_trajectories.py), refs/test (standard shapes),
refs/test_unseen (random curves with another seed, seen by neither controller). README_EPITES.md, step 2."""

MPC_N = 20
"""FUJI tasks (legacy): number of look-ahead points K, set before the MPC side decided N (MAGYARAZAT 4.3)."""
MPC_T = 0.05
"""FUJI tasks (legacy): spacing of the look-ahead points [s]."""

COMPARE_N = 10
"""Comparison tasks: the MPC horizon N (Mecanum-Robot-MPC-TDK D4: N = 10 since 2026-10-21) = look-ahead points K."""
COMPARE_T = 0.05
"""Comparison tasks: the MPC sampling time T [s] = spacing of the look-ahead points = the policy period."""
COMPARE_DECIMATION = 18
"""Comparison tasks: 18 physics steps of 1/360 s = 0.05 s = T, the policy holds its torques in between (ZOH), as the
MPC does."""
COMPARE_CURRICULUM_TOP = 1.2
"""Top of the speed curriculum of the comparison tasks: the training references (refs/train_own, 0.1 m/s at s = 1)
are generated with wheel speeds <= 8.06 / 1.2 rad/s, so even the fastest one stays feasible
(tools/gen_training_trajectories.py --own)."""
TORQUE_ACTION_SCALE = 0.25
"""Comparison tasks: wheel torque per policy output unit [Nm]. Only the parametrization of the policy output: the
torque is clipped to +-u_max = +-1.95 Nm, the same box as the MPC's. The reference torques are <= 0.01 Nm and the
traction limit is ~0.14 Nm per wheel (mu 0.9, 2.134 kg), so a unit of 1.95 Nm would make the initial exploration
noise (std 0.5 units) spin every wheel."""

COMPARE_START_OFFSET_PROB = (0.5, 0.5)
"""Comparison tasks (training): probability of a random position / heading offset at a reset, independent of each
other: a quarter of the starts on the reference, a quarter off it, a quarter turned, a quarter both."""
COMPARE_START_OFFSET_MAX = (0.4, math.pi / 4)
"""Comparison tasks (training): largest random start offset, 0.4 m and 45 deg (covers the evaluation scenario 2,
30 cm / 30 deg)."""

EPISODE_LENGTH_S = 20.0
"""Training episode [s]; a truncation (time_out=True), tracking has no natural end (MAGYARAZAT 5.2)."""

ENV_SPACING = 8.0
"""Distance between the env origins [m]. The training curves reach 6 m (median) to 11 m from their start, so the
paths of neighbouring envs overlap; harmless, the scene filters the collisions between envs
(``InteractiveSceneCfg.filter_collisions``), only the videos show other robots passing by."""

FOLLOW_CAMERA_EYE = {"fuji": (-3.0, -3.0, 3.5), "own": (-2.0, -2.0, 2.5)}
"""Viewer / video camera offset from the followed robot [m], per robot (the own robot is ~0.3 m, FUJI ~0.66 m)."""


def _refs(sub: str) -> list[str]:
    files = sorted(glob.glob(os.path.join(REF_DIR, sub, "*.csv")))
    if not files:
        raise FileNotFoundError(f"no reference files in {os.path.join(REF_DIR, sub)} -- see README_EPITES.md, step 2")
    return files


##
# Scene: flat ground with the roller contact material, one robot per env, no lidar
##


@configclass
class MecanumTrackingSceneCfg(InteractiveSceneCfg):
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        # carries the compliant roller contact (only effective on the ground side), as in your navigation scene
        physics_material=mecanum.ground_material(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.45, 0.45, 0.5)),
        debug_vis=False,
    )
    robot: ArticulationCfg = mecanum.MECANUM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=750.0),
    )


##
# MDP
##


@configclass
class ActionsCfg:
    """The policy commands the four wheel speed setpoints (as in the mecanum_ws navigation task).

    The setpoints go through the torque-limited DC motor model of the wheels, so the policy faces the roller physics
    directly: the wheels slip whenever the torque demand exceeds the traction.
    """

    wheel_vel = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=mecanum.WHEEL_JOINT_NAMES,
        scale=mecanum.MAX_WHEEL_SPEED,
        use_default_offset=False,
        preserve_order=True,
        clip={".*": (-mecanum.MAX_WHEEL_SPEED, mecanum.MAX_WHEEL_SPEED)},
    )


@configclass
class IkActionsCfg:
    """The policy commands a base twist, mapped to wheels by mecanum IK (as in the mecanum_ws navigation task)."""

    base_vel = mdp.MecanumBaseVelocityActionCfg(
        asset_name="robot",
        wheel_joint_names=mecanum.WHEEL_JOINT_NAMES,
        wheel_joint_signs=mecanum.WHEEL_JOINT_SIGNS,
        wheel_radius=mecanum.WHEEL_RADIUS,
        wheel_base_half_length=mecanum.WHEEL_BASE_HALF_LENGTH,
        track_half_width=mecanum.TRACK_HALF_WIDTH,
        scale=(1.0, 1.0, 1.5),
        max_wheel_speed=mecanum.MAX_WHEEL_SPEED,
    )


@configclass
class TorqueActionsCfg:
    """Comparison tasks: the policy commands the four wheel torques, the MPC's interface (OSSZEHANGOLAS 3.2)."""

    wheel_torque = mdp.WheelTorqueActionCfg(
        asset_name="robot",
        joint_names=own_mecanum.WHEEL_JOINT_NAMES,
        scale=TORQUE_ACTION_SCALE,
        preserve_order=True,
        clip={".*": (-own_mecanum.WHEEL_EFFORT_LIMIT, own_mecanum.WHEEL_EFFORT_LIMIT)},
    )


@configclass
class CommandsCfg:
    """One reference trajectory per env, fixed for the episode (MAGYARAZAT 3.2, 4)."""

    ref_traj = mdp.TrajectoryCommandCfg(
        asset_name="robot",
        traj_files=[],  # filled in __post_init__ (needs the generated files)
        preview_steps=MPC_N,
        preview_dt=MPC_T,
        loop=True,
        random_start=True,  # training: start anywhere on the curve (eval: False)
        initial_time_scale=0.5,  # speed curriculum starts slow (MAGYARAZAT 8.2)
        velocity_in_traj_frame=False,  # heading-free files, theta = 0 (MAGYARAZAT 6.3)
        resampling_time_range=(1.0e9, 1.0e9),  # only resampled on reset (MAGYARAZAT 3.2)
        debug_vis=True,
    )


@configclass
class CompareCommandsCfg:
    """Comparison tasks: the MPC's horizon as look-ahead (N = 10, T = 0.05 s), and training from random initial errors
    (COMPARE_START_OFFSET_*; the PLAY / EVAL variants switch them off, eval_tracking.py --start_offset sets a fixed one).
    The references (refs/train_own) are filled in by apply_comparison."""

    ref_traj = mdp.TrajectoryCommandCfg(
        asset_name="robot",
        traj_files=[],  # filled in apply_comparison (needs the generated files)
        preview_steps=COMPARE_N,
        preview_dt=COMPARE_T,
        loop=True,
        random_start=True,
        initial_time_scale=0.5,
        velocity_in_traj_frame=False,
        resampling_time_range=(1.0e9, 1.0e9),
        start_offset_prob=COMPARE_START_OFFSET_PROB,
        start_offset_max=COMPARE_START_OFFSET_MAX,
        debug_vis=True,
    )


@configclass
class ObservationsCfg:
    """MAGYARAZAT 5. NO remaining-time term: time-unlimited task (Pardo et al. 2018)."""

    @configclass
    class PolicyCfg(ObsGroup):
        ref_preview = ObsTerm(
            func=mdp.ref_preview_b,
            # heading-FREE task: no sin/cos(dtheta). n_extra=4 if the MPC files contain tau_ref (not with time scaling)
            params={"command_name": "ref_traj", "include_current": True, "include_heading": False, "n_extra": 0},
            noise=Unoise(n_min=-0.02, n_max=0.02),
        )
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, noise=Unoise(n_min=-0.05, n_max=0.05))
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.1, n_max=0.1))
        wheel_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=mecanum.WHEEL_JOINT_NAMES, preserve_order=True)},
            scale=1.0 / mecanum.MAX_WHEEL_SPEED,
            noise=Unoise(n_min=-0.02, n_max=0.02),
        )
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self) -> None:
            self.enable_corruption = True  # training: noise on; evaluation: off (MAGYARAZAT 5.3)
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class CompareObservationsCfg:
    """Comparison tasks: the policy sees what the MPC's step(x_meas, x_r, u_r, u_applied) gets, expressed in the
    robot's frame (the dynamics are invariant to planar rigid motions, so the world pose itself carries no
    information):
    * ``ref_preview``: x_r,0..N relative to x: (dX, dY) and (X_dot_r, Y_dot_r) in the robot's heading frame,
      sin/cos(psi_r - psi), psi_dot_r; 11 points (N = 10, T = 0.05 s);
    * ``base_vel``: the velocity part of x: (X_dot, Y_dot) in the heading frame, psi_dot;
    * ``last_torque``: u_applied, the torques of the previous step.
    u_r (the MPC's feedforward) is left out in the main case; ablation: ``ref_preview`` n_extra = 4 (the tau columns of
    the files), only with the speed curriculum off (the torques do not scale with the time scale)."""

    @configclass
    class PolicyCfg(ObsGroup):
        ref_preview = ObsTerm(
            func=mdp.ref_preview_b,
            params={"command_name": "ref_traj", "include_current": True, "include_heading": True, "n_extra": 0},
            noise=Unoise(n_min=-0.005, n_max=0.005),
        )
        base_vel = ObsTerm(func=mdp.base_planar_vel_b, noise=Unoise(n_min=-0.01, n_max=0.01))
        last_torque = ObsTerm(
            func=mdp.last_wheel_torque, params={"action_name": "wheel_torque", "scale": 1.0 / TORQUE_ACTION_SCALE}
        )

        def __post_init__(self) -> None:
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class CompareRewardsCfg:
    """Comparison tasks: the kernels scaled to the 0.1 m/s / millimetre task, plus heading tracking (the MPC tracks
    psi_r); the torque-rate penalty is the RL counterpart of the MPC's S, the wheel-speed penalty that of its soft
    wheel-speed limit. ~3.5 per step at perfect tracking.

    Position and heading have a coarse and a fine kernel each: a Gaussian kernel is only steep within ~its std, and
    both errors span a wide range (the training starts up to 0.4 m / 45 deg off, COMPARE_START_OFFSET_*). The coarse
    one brings the robot back (std / largest offset ~1.6 for both), the fine one gives the precision."""

    track_pos_coarse = RewTerm(func=mdp.track_pos_exp, weight=1.0, params={"command_name": "ref_traj", "std": 0.25})
    track_pos_fine = RewTerm(func=mdp.track_pos_exp, weight=1.0, params={"command_name": "ref_traj", "std": 0.05})
    track_vel = RewTerm(func=mdp.track_vel_exp, weight=0.5, params={"command_name": "ref_traj", "std": 0.1})
    track_heading_coarse = RewTerm(
        func=mdp.track_heading_exp, weight=0.5, params={"command_name": "ref_traj", "std": 0.5}
    )
    track_heading_fine = RewTerm(
        func=mdp.track_heading_exp, weight=0.5, params={"command_name": "ref_traj", "std": 0.2}
    )
    """Until 2026-10-26 named track_heading (runs before torque_offset_seed0 log it under that name)."""
    torque_rate = RewTerm(func=mdp.torque_rate_l2, weight=-20.0, params={"action_name": "wheel_torque"})
    """-20: with -1 the first policies chattered (torque RMS 5x the MPC's, du up to 0.15 Nm per step; 2026-10-23)."""
    wheel_speed = RewTerm(
        func=mdp.wheel_speed_excess_l2,
        weight=-0.1,
        params={
            "omega_max": own_mecanum.MOTOR_NO_LOAD_SPEED,
            "asset_cfg": SceneEntityCfg("robot", joint_names=own_mecanum.WHEEL_JOINT_NAMES, preserve_order=True),
        },
    )
    """The MPC's soft wheel-speed limit, |w| <= 8.06 rad/s (the chattering policies spun the wheels up to 13 rad/s)."""
    yaw_rate_error = RewTerm(func=mdp.yaw_rate_error_l2, weight=-0.01, params={"command_name": "ref_traj"})
    torque = RewTerm(func=mdp.torque_l2, weight=0.0, params={"action_name": "wheel_torque"})
    """Off (0): the RL counterpart of the MPC's R, to switch on if the torques get noisy."""


@configclass
class RewardsCfg:
    """MAGYARAZAT 7. Starting values: ~2.5 per step at perfect tracking; penalties a small fraction of it so the total
    stays positive. Tune with your penalty tools (e.g. staged_penalties.py, the calibration you used for
    PILLAR_PENALTIES)."""

    track_pos_coarse = RewTerm(func=mdp.track_pos_exp, weight=1.0, params={"command_name": "ref_traj", "std": 0.5})
    track_pos_fine = RewTerm(func=mdp.track_pos_exp, weight=1.0, params={"command_name": "ref_traj", "std": 0.1})
    track_vel = RewTerm(func=mdp.track_vel_exp, weight=0.5, params={"command_name": "ref_traj", "std": 0.3})
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.01)
    yaw_rate = RewTerm(func=mdp.yaw_rate_l2, weight=-0.01)


@configclass
class TerminationsCfg:
    """MAGYARAZAT 7.3: the time limit is a truncation (bootstrap), losing the trajectory is a real failure."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    deviation = DoneTerm(func=mdp.tracking_deviation, params={"command_name": "ref_traj", "max_error": 1.0})


@configclass
class CompareTerminationsCfg:
    """Comparison tasks (training): only the time limit. The robot also starts off the reference
    (COMPARE_START_OFFSET_*) and has to come back from wherever it gets to, so losing the trajectory does not end the
    episode (the speed curriculum then only moves up). The PLAY / EVAL variants keep TerminationsCfg: > 1 m = lost."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@configclass
class CurriculumCfg:
    """MAGYARAZAT 8.2: per-env reference speed (time scale s), like your terrain_levels_goal."""

    speed = CurrTerm(
        func=mdp.speed_levels,
        params={"command_name": "ref_traj", "up_error": 0.1, "scale_range": (0.5, 2.0), "step": 0.1},
    )


@configclass
class EventCfg:
    """MAGYARAZAT 8.3: domain randomization is a TRAINING tool; the evaluation uses fixed, identical values."""

    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.5, 1.1),
            "dynamic_friction_range": (0.4, 1.0),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 64,
            "make_consistent": True,  # dynamic <= static friction in every bucket (physically required)
        },
    )
    base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=mecanum.BASE_LINK_NAME),
            "mass_distribution_params": (0.8, 1.2),
            "operation": "scale",
        },
    )
    # random start pose around the env origin; the TrajectoryCommand anchors the trajectory to it (MAGYARAZAT 4.4)
    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={"pose_range": {"x": (-0.3, 0.3), "y": (-0.3, 0.3), "yaw": (-3.14159, 3.14159)}, "velocity_range": {}},
    )


##
# Viewer and robot selection
##


def follow_camera(cfg: ManagerBasedRLEnvCfg, eye: tuple[float, float, float], env_index: int = 0) -> None:
    """Point the viewer (and the recorded videos) at the robot of ``env_index``, from above-behind, so that the robot
    and its reference markers (red: current point, blue: look-ahead) stay in view."""
    cfg.viewer.origin_type = "asset_root"
    cfg.viewer.asset_name = "robot"
    cfg.viewer.env_index = env_index
    cfg.viewer.eye = eye
    cfg.viewer.lookat = (0.0, 0.0, 0.0)


def robot_module(name: str):
    """Robot constants module of ``robot_name``: ``fuji`` (:mod:`mecanum_tracking_ws.robots.mecanum`, the default)
    or ``own`` (:mod:`mecanum_tracking_ws.robots.own_mecanum`)."""
    return own_mecanum if name == "own" else mecanum


def apply_robot(cfg: "MecanumTrackingEnvCfg", name: str) -> None:
    """Swap every robot-dependent setting of a tracking task (robot, materials, action / observation scaling, the body
    of the mass randomization, camera), like ``apply_robot`` of the mecanum_ws navigation task."""
    rb = robot_module(name)
    cfg.robot_name = name
    # torque actions drive the motors directly; setpoint actions need the robot with the wheel velocity loop
    robot = rb.MECANUM_CFG if hasattr(cfg.actions, "wheel_torque") else getattr(rb, "MECANUM_VEL_CFG", rb.MECANUM_CFG)
    cfg.scene.robot = robot.replace(prim_path="{ENV_REGEX_NS}/Robot")
    cfg.scene.terrain.physics_material = rb.ground_material()
    cfg.sim.physics_material = rb.ROBOT_MATERIAL
    if getattr(cfg.observations.policy, "wheel_vel", None) is not None:
        cfg.observations.policy.wheel_vel.scale = 1.0 / rb.MAX_WHEEL_SPEED
    if cfg.events.base_mass is not None:
        cfg.events.base_mass.params["asset_cfg"] = SceneEntityCfg("robot", body_names=rb.BASE_LINK_NAME)
    if hasattr(cfg.actions, "wheel_torque"):
        pass  # names, scale and clip are set in TorqueActionsCfg (own robot only)
    elif hasattr(cfg.actions, "base_vel"):
        cfg.actions.base_vel.wheel_joint_signs = rb.WHEEL_JOINT_SIGNS
        cfg.actions.base_vel.wheel_radius = rb.WHEEL_RADIUS
        cfg.actions.base_vel.wheel_base_half_length = rb.WHEEL_BASE_HALF_LENGTH
        cfg.actions.base_vel.track_half_width = rb.TRACK_HALF_WIDTH
        cfg.actions.base_vel.max_wheel_speed = rb.MAX_WHEEL_SPEED
    else:
        cfg.actions.wheel_vel.scale = rb.MAX_WHEEL_SPEED
        cfg.actions.wheel_vel.clip = {".*": (-rb.MAX_WHEEL_SPEED, rb.MAX_WHEEL_SPEED)}
    follow_camera(cfg, FOLLOW_CAMERA_EYE[name])


def apply_comparison(cfg: "MecanumTrackingEnvCfg") -> None:
    """The settings shared with the LTV-MPC (docs/OSSZEHANGOLAS.md): our own robot (weighed masses, XM430), 20 Hz
    control, the training references at the MPC's speed (0.1 m/s), a curriculum that keeps them feasible (the
    look-ahead and the initial errors are in CompareCommandsCfg)."""
    apply_robot(cfg, "own")
    cfg.decimation = COMPARE_DECIMATION
    cfg.sim.render_interval = cfg.decimation
    cfg.commands.ref_traj.traj_files = _refs("train_own")
    if cfg.curriculum.speed is not None:
        cfg.curriculum.speed.params["scale_range"] = (0.5, COMPARE_CURRICULUM_TOP)
        cfg.curriculum.speed.params["up_error"] = 0.05


def to_play(cfg: "MecanumTrackingEnvCfg") -> None:
    """Watch the policy: a few envs, noise and DR off, nominal speed."""
    cfg.scene.num_envs = 16
    cfg.observations.policy.enable_corruption = False
    cfg.events.physics_material = None
    cfg.events.base_mass = None
    cfg.curriculum.speed = None
    cfg.commands.ref_traj.initial_time_scale = 1.0
    cfg.commands.ref_traj.start_offset_prob = (0.0, 0.0)  # no random initial error (a fixed one: start_offset)


def to_eval(cfg: "MecanumTrackingEnvCfg", refs: str) -> None:
    """The evaluation protocol (MAGYARAZAT 9.3), on top of :func:`to_play`: env i follows test trajectory i of
    refs/<refs>, from its start, at the env origin with heading 0 (= psi_r(0) after anchoring). The MPC runs in the
    very same env (scripts/rsl_rl/eval_tracking.py --controller mpc)."""
    cfg.commands.ref_traj.traj_files = _refs(refs)
    cfg.commands.ref_traj.fixed_assignment = True
    cfg.commands.ref_traj.random_start = False
    cfg.scene.num_envs = len(cfg.commands.ref_traj.traj_files)
    cfg.events.reset_base.params["pose_range"] = {"x": (0.0, 0.0), "y": (0.0, 0.0), "yaw": (0.0, 0.0)}


##
# Environment configs
##


@configclass
class MecanumTrackingEnvCfg(ManagerBasedRLEnvCfg):
    """Trajectory tracking; the policy outputs a base twist (vx, vy, wz) -> mecanum IK -> wheel setpoints through
    the torque-limited motor model. The action space is still an OPEN decision (MAGYARAZAT 2, 4.):
    see MecanumTrackingWheelEnvCfg for the wheel-level alternative."""

    scene: MecanumTrackingSceneCfg = MecanumTrackingSceneCfg(num_envs=4096, env_spacing=ENV_SPACING)
    robot_name: str = "fuji"
    """Robot of the task: ``fuji`` or ``own`` (see :func:`apply_robot`); scripts/rsl_rl/eval_tracking.py reads it."""
    observations: ObservationsCfg = ObservationsCfg()
    actions: IkActionsCfg = IkActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self) -> None:
        # same simulation settings as the navigation task: 360 Hz physics, 51 Hz policy
        self.decimation = 7
        self.episode_length_s = EPISODE_LENGTH_S
        self.sim.dt = 1.0 / 360.0
        self.sim.render_interval = self.decimation
        self.sim.physics_material = mecanum.ROBOT_MATERIAL
        self.sim.physx.gpu_max_rigid_patch_count = 2**19
        self.commands.ref_traj.traj_files = _refs("train")
        follow_camera(self, FOLLOW_CAMERA_EYE[self.robot_name])


@configclass
class MecanumTrackingWheelEnvCfg(MecanumTrackingEnvCfg):
    """Ablation A4: the policy outputs the four wheel speed setpoints directly (no IK)."""

    actions: ActionsCfg = ActionsCfg()


@configclass
class MecanumTrackingEnvCfg_PLAY(MecanumTrackingEnvCfg):
    """Watch the policy: a few envs, noise and DR off, nominal speed."""

    def __post_init__(self) -> None:
        super().__post_init__()
        to_play(self)


@configclass
class MecanumTrackingEnvCfg_EVAL(MecanumTrackingEnvCfg_PLAY):
    """The evaluation protocol (MAGYARAZAT 9.3): env i follows test trajectory i, one lap, from its start.
    scripts/eval_tracking.py sets traj_files / num_envs / episode length itself; the MPC runs use the same settings."""

    def __post_init__(self) -> None:
        super().__post_init__()
        to_eval(self, "test")


##
# Comparison with the LTV-MPC: our own robot, wheel torques (assets/robots/own_mecanum/robot_measured.urdf)
##


@configclass
class MecanumTrackingTorqueEnvCfg(MecanumTrackingEnvCfg):
    """THE comparison task (docs/OSSZEHANGOLAS.md): the policy outputs the four wheel torques at 20 Hz, the same
    plant, references, horizon and limits as the LTV-MPC. Trained from random initial errors (CompareCommandsCfg,
    since 2026-10-26; the runs before torque_offset_seed0 started on the reference and ended at 1 m deviation, see
    their params/env.yaml)."""

    actions: TorqueActionsCfg = TorqueActionsCfg()
    commands: CompareCommandsCfg = CompareCommandsCfg()
    observations: CompareObservationsCfg = CompareObservationsCfg()
    rewards: CompareRewardsCfg = CompareRewardsCfg()
    terminations: CompareTerminationsCfg = CompareTerminationsCfg()

    def __post_init__(self) -> None:
        super().__post_init__()
        apply_comparison(self)


@configclass
class MecanumTrackingTorqueEnvCfg_PLAY(MecanumTrackingTorqueEnvCfg):
    """Watching / evaluation: no random initial error, and losing the trajectory (> 1 m) ends the run (eval_tracking.py
    and play_shapes.py report it as lost)."""

    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self) -> None:
        super().__post_init__()
        to_play(self)


@configclass
class MecanumTrackingTorqueEnvCfg_EVAL(MecanumTrackingTorqueEnvCfg_PLAY):
    """Evaluation, default set refs/test_fixed (eval_tracking.py --traj_dir selects test_tangent / test_unseen_own);
    the reference stops at its end (loop=False: one lap with cosine ramps)."""

    def __post_init__(self) -> None:
        super().__post_init__()
        to_eval(self, "test_fixed")
        self.commands.ref_traj.loop = False


@configclass
class MecanumTrackingOwnEnvCfg(MecanumTrackingTorqueEnvCfg):
    """Action-space ablation of the comparison: the same task, but the policy outputs a base twist -> mecanum IK ->
    wheel speed setpoints of a velocity loop (MECANUM_VEL_CFG: needs an armature, so a slightly different plant)."""

    actions: IkActionsCfg = IkActionsCfg()

    def __post_init__(self) -> None:
        super().__post_init__()
        # the robot's top speed is 8.06 rad/s * 0.0303 m = 0.24 m/s
        self.actions.base_vel.scale = (0.25, 0.25, 1.0)
        self.observations.policy.last_torque = ObsTerm(func=mdp.last_action)
        self.rewards.torque_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.01)
        self.rewards.torque = None


@configclass
class MecanumTrackingOwnEnvCfg_PLAY(MecanumTrackingOwnEnvCfg):
    terminations: TerminationsCfg = TerminationsCfg()  # as MecanumTrackingTorqueEnvCfg_PLAY

    def __post_init__(self) -> None:
        super().__post_init__()
        to_play(self)


@configclass
class MecanumTrackingOwnEnvCfg_EVAL(MecanumTrackingOwnEnvCfg_PLAY):
    def __post_init__(self) -> None:
        super().__post_init__()
        to_eval(self, "test_fixed")
        self.commands.ref_traj.loop = False
