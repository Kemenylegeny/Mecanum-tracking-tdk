"""Mecanum trajectory tracking (RL, to be compared with an LTV-MPC). See docs/MAGYARAZAT.md of the project."""

import gymnasium as gym

from . import agents

_TASKS = {
    # policy outputs a base twist (vx, vy, wz) -> mecanum IK -> wheel setpoints
    "Mecanum-Tracking-v0": ("MecanumTrackingEnvCfg", "PPORunnerTrackingCfg"),
    "Mecanum-Tracking-Play-v0": ("MecanumTrackingEnvCfg_PLAY", "PPORunnerTrackingCfg"),
    "Mecanum-Tracking-Eval-v0": ("MecanumTrackingEnvCfg_EVAL", "PPORunnerTrackingCfg"),
    # ablation A4: policy outputs the four wheel speed setpoints
    "Mecanum-Tracking-Wheel-v0": ("MecanumTrackingWheelEnvCfg", "PPORunnerTrackingWheelCfg"),
    # THE comparison with the LTV-MPC (docs/OSSZEHANGOLAS.md): own robot (weighed masses, XM430), 4 wheel torques,
    # 20 Hz, N = 10, heading tracking, the MPC side's references; eval_tracking.py --controller mpc runs the MPC in it
    "Mecanum-Tracking-Torque-v0": ("MecanumTrackingTorqueEnvCfg", "PPORunnerTrackingTorqueCfg"),
    "Mecanum-Tracking-Torque-Play-v0": ("MecanumTrackingTorqueEnvCfg_PLAY", "PPORunnerTrackingTorqueCfg"),
    "Mecanum-Tracking-Torque-Eval-v0": ("MecanumTrackingTorqueEnvCfg_EVAL", "PPORunnerTrackingTorqueCfg"),
    # + a 7 mm position kernel (torque_precise_seed0, 2026-10-30); evaluate with -Torque-Eval-v0
    "Mecanum-Tracking-Torque-Precise-v0": ("MecanumTrackingTorquePreciseEnvCfg", "PPORunnerTrackingTorqueCfg"),
    # the ID torque_offset_seed0 was trained under: the comparison task until 2026-10-27, with the torque-rate and
    # wheel-speed penalties and the last-torque observation (model_311.pt, model_600.pt: evaluate with -Offset-Eval-v0)
    "Mecanum-Tracking-Torque-Offset-v0": ("MecanumTrackingTorqueOffsetEnvCfg", "PPORunnerTrackingTorqueCfg"),
    "Mecanum-Tracking-Torque-Offset-Play-v0": ("MecanumTrackingTorqueOffsetEnvCfg_PLAY", "PPORunnerTrackingTorqueCfg"),
    "Mecanum-Tracking-Torque-Offset-Eval-v0": ("MecanumTrackingTorqueOffsetEnvCfg_EVAL", "PPORunnerTrackingTorqueCfg"),
    # its action-space ablation: twist + IK -> wheel speed setpoints (velocity loop), otherwise the same task
    "Mecanum-Tracking-Own-v0": ("MecanumTrackingOwnEnvCfg", "PPORunnerTrackingOwnCfg"),
    "Mecanum-Tracking-Own-Play-v0": ("MecanumTrackingOwnEnvCfg_PLAY", "PPORunnerTrackingOwnCfg"),
    "Mecanum-Tracking-Own-Eval-v0": ("MecanumTrackingOwnEnvCfg_EVAL", "PPORunnerTrackingOwnCfg"),
}

for task_id, (env_cfg, runner_cfg) in _TASKS.items():
    gym.register(
        id=task_id,
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.tracking_env_cfg:{env_cfg}",
            "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:{runner_cfg}",
        },
    )
