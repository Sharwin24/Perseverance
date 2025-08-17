
import os

import numpy as np
import gymnasium as gym

from gymnasium.wrappers import TimeLimit
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnRewardThreshold
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.vec_env import VecMonitor, VecNormalize


from rover_nav_gym import RoverNavEnv


def make_env(render_mode=None):
    def _thunk():
        env = RoverNavEnv(render_mode=render_mode)
        env = TimeLimit(env, max_episode_steps=400)
        return env
    return _thunk


if __name__ == "__main__":
    # Single env check
    check_env(make_env()())

    # Vectorized envs
    n_envs = 8
    vec_env = make_vec_env(make_env(), n_envs=n_envs, monitor_dir="./logs")
    vec_env = VecMonitor(vec_env)  # record ep reward/len
    vec_env = VecNormalize(vec_env, norm_obs=True,
                           norm_reward=True, clip_obs=10.0, gamma=0.995)

    policy_kwargs = dict(net_arch=[128, 128])

    model = PPO(
        "MlpPolicy",
        vec_env,
        n_steps=1024,               # per env -> total rollout = n_steps * n_envs
        batch_size=256,
        gamma=0.995,
        gae_lambda=0.98,
        learning_rate=3e-4,
        ent_coef=0.01,
        clip_range=0.2,
        vf_coef=0.5,
        max_grad_norm=0.5,
        policy_kwargs=policy_kwargs,
        verbose=1,
        tensorboard_log="./tb",
        seed=42,
    )

    # ---- before training (for EvalCallback) ----
    eval_env = make_vec_env(make_env(), n_envs=1)
    eval_env = VecMonitor(eval_env)
    # use obs normalization for eval, not reward normalization
    eval_env = VecNormalize(eval_env, training=False,
                            norm_obs=True, norm_reward=False)

    callback_on_best = StopTrainingOnRewardThreshold(
        reward_threshold=4.5, verbose=1)
    eval_callback = EvalCallback(
        eval_env,
        callback_on_new_best=callback_on_best,
        best_model_save_path="./checkpoints",
        log_path="./eval_logs",
        eval_freq=5000,
        deterministic=True,
        render=False,
    )

    total_timesteps = 300_000
    model.learn(total_timesteps=total_timesteps, callback=eval_callback)

    # Save model + VecNormalize stats
    os.makedirs("./checkpoints", exist_ok=True)
    model.save("./checkpoints/ppo_rover_nav")
    vec_env.save("./checkpoints/vecnormalize.pkl")
    print("Training complete. Saved model and normalization stats.")

    # -------- Evaluation --------
    eval_env = make_vec_env(make_env(), n_envs=1)
    eval_env = VecMonitor(eval_env)
    eval_env = VecNormalize.load("./checkpoints/vecnormalize.pkl", eval_env)
    eval_env.training = False
    eval_env.norm_reward = False

    model = PPO.load("./checkpoints/ppo_rover_nav", eval_env)

    obs = eval_env.reset()
    for _ in range(5000):
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, dones, infos = eval_env.step(action)
        if dones[0]:
            obs = eval_env.reset()
