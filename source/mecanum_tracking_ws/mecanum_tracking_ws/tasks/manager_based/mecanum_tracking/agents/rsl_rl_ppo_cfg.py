"""PPO settings for trajectory tracking: the PPO config of the mecanum_ws navigation task (PPORunnerFlatCfg, copied
here so the project does not depend on mecanum_ws), adapted to a DENSE reward."""

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg


@configclass
class PPORunnerTrackingCfg(RslRlOnPolicyRunnerCfg):
    experiment_name = "mecanum_tracking"
    # 48 steps per env was chosen in the navigation task against its SPARSE end-of-episode reward; the tracking
    # reward is dense, so 24 (as in the mecanum_velocity task) is enough and halves the time per iteration
    num_steps_per_env = 24
    max_iterations = 1500
    save_interval = 100
    # gamma = 0.99 -> horizon ~1/(1-gamma) = 100 steps ~ 2 s at 51 Hz (MAGYARAZAT 1)
    seed = 0  # train several seeds for the comparison: --seed 0 1 2 (3), 4 (MAGYARAZAT 9.1)
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.5,
        actor_obs_normalization=True,
        critic_obs_normalization=True,
        actor_hidden_dims=[256, 128, 64],
        critic_hidden_dims=[256, 128, 64],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        # no entropy bonus, as in the navigation flat task: with clipped actions nothing counteracts it (there the
        # action std grew from 0.5 to 4.3 in 500 iterations with entropy_coef = 0.005)
        entropy_coef=0.0,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )


@configclass
class PPORunnerTrackingWheelCfg(PPORunnerTrackingCfg):
    experiment_name = "mecanum_tracking_wheel"


@configclass
class PPORunnerTrackingOwnCfg(PPORunnerTrackingCfg):
    experiment_name = "mecanum_tracking_own"


@configclass
class PPORunnerTrackingTorqueCfg(PPORunnerTrackingCfg):
    """The comparison task (wheel torques, 20 Hz). gamma = 0.99 is now a ~5 s horizon (100 steps of 50 ms); 24 steps
    per env = 1.2 s of experience per iteration."""

    experiment_name = "mecanum_tracking_torque"
