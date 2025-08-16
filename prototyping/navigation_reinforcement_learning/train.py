import argparse
import os
import time
from typing import List
import torch
import numpy as np

from nav_env import RoverNavEnv
from ppo import PPOAgent


def train(
        total_updates: int = 100,
        rollout_steps: int = 2048,
        max_ep_len: int = 400,
        lr_actor: float = 3e-4,
        lr_critic: float = 1e-3,
        gamma: float = 0.995,
        epsilon_clip: float = 0.2,
        K_epochs: int = 10,
        entropy_coef: float = 0.01,
        gae_lambda: float = 0.98,
        seed: int = 42,
        save_dir: str = "./checkpoints",
    save_every: int = 10,
    render: bool = False,
    render_fps: int = 60,
):
    os.makedirs(save_dir, exist_ok=True)

    # Environment
    env = RoverNavEnv(seed=seed, normalized_actions=True)
    obs = env.reset()

    # Agent setup (normalized actions in [-1, 1])
    state_dim = obs.shape[0]
    action_dim = 2
    action_low = np.array([-1.0, -1.0], dtype=np.float32)
    action_high = np.array([1.0, 1.0], dtype=np.float32)

    agent = PPOAgent(
        state_dim=state_dim,
        action_dim=action_dim,
        lr_actor=lr_actor,
        lr_critic=lr_critic,
        gamma=gamma,
        epsilon_clip=epsilon_clip,
        K_epochs=K_epochs,
        entropy_coef=entropy_coef,
        gae_lambda=gae_lambda,
        action_space_low=action_low,
        action_space_high=action_high,
    )

    ep_returns: List[float] = []
    ep_lens: List[int] = []
    ep_return = 0.0
    ep_len = 0

    start_time = time.time()
    for update in range(1, total_updates + 1):
        steps_collected = 0
        while steps_collected < rollout_steps:
            action, log_prob = agent.select_action(obs)
            next_obs, reward, done, _ = env.step(action)

            agent.buffer.store(obs, action, reward, next_obs, done, log_prob)

            obs = next_obs
            ep_return += reward
            ep_len += 1
            steps_collected += 1

            # Optional real-time visualization
            if render:
                try:
                    env.render(fps=render_fps)
                except Exception:
                    # If rendering fails (e.g., headless), continue training
                    render = False

            # Episode termination (success or timeout)
            if done or ep_len >= max_ep_len:
                ep_returns.append(ep_return)
                ep_lens.append(ep_len)
                obs = env.reset()
                ep_return = 0.0
                ep_len = 0

        # Policy update
        agent.update()

        # Logging
        mean_ret = float(np.mean(ep_returns[-10:])) if ep_returns else 0.0
        mean_len = float(np.mean(ep_lens[-10:])) if ep_lens else 0.0
        elapsed = time.time() - start_time
        print(
            f"Update {update:04d}/{total_updates} | Steps {rollout_steps} | "
            f"AvgReturn(10) {mean_ret:.3f} | AvgLen(10) {mean_len:.1f} | Elapsed {elapsed:.1f}s"
        )

        # Checkpointing
        if save_every and (update % save_every == 0):
            ckpt_path = os.path.join(
                save_dir, f"ppo_rover_update_{update:04d}.pt")
            # Save both actor and critic

            torch.save(
                {
                    "actor": agent.actor.state_dict(),
                    "critic": agent.critic.state_dict(),
                    "config": {
                        "state_dim": state_dim,
                        "action_dim": action_dim,
                        "lr_actor": lr_actor,
                        "lr_critic": lr_critic,
                        "gamma": gamma,
                        "epsilon_clip": epsilon_clip,
                        "K_epochs": K_epochs,
                        "entropy_coef": entropy_coef,
                        "gae_lambda": gae_lambda,
                        "action_low": action_low,
                        "action_high": action_high,
                    },
                },
                ckpt_path,
            )
            print(f"Saved checkpoint to {ckpt_path}")

    env.close()


def parse_args():
    p = argparse.ArgumentParser(description="Train PPOAgent on RoverNavEnv")
    p.add_argument("--total-updates", type=int, default=50)
    p.add_argument("--rollout-steps", type=int, default=2048)
    p.add_argument("--max-ep-len", type=int, default=400)
    p.add_argument("--lr-actor", type=float, default=3e-4)
    p.add_argument("--lr-critic", type=float, default=1e-3)
    p.add_argument("--gamma", type=float, default=0.995)
    p.add_argument("--epsilon-clip", type=float, default=0.2)
    p.add_argument("--K-epochs", type=int, default=10)
    p.add_argument("--entropy-coef", type=float, default=0.01)
    p.add_argument("--gae-lambda", type=float, default=0.98)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--save-dir", type=str, default="./checkpoints")
    p.add_argument("--save-every", type=int, default=10)
    p.add_argument("--render", action="store_true",
                   help="Render pygame", default=False)
    p.add_argument("--render-fps", type=int, default=60)
    # Evaluation options
    p.add_argument("--eval-ckpt", type=str, default="",
                   help="Path to a .pt checkpoint or a directory containing checkpoints to evaluate")
    p.add_argument("--eval-episodes", type=int, default=5)
    p.add_argument("--deterministic", action="store_true",
                   help="Use mean action (no sampling) at eval")
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()

    # Evaluation mode if a checkpoint path or directory is provided
    if args.eval_ckpt:
        def _find_latest(path: str) -> str:
            if os.path.isdir(path):
                cks = [f for f in os.listdir(path) if f.endswith(".pt")]
                if not cks:
                    raise FileNotFoundError(
                        f"No .pt checkpoints found in {path}")
                cks.sort()  # names are zero-padded; last is latest
                return os.path.join(path, cks[-1])
            return path

        ckpt_path = _find_latest(args.eval_ckpt)
        print(f"Evaluating checkpoint: {ckpt_path}")

        # Load checkpoint
        ckpt = torch.load(ckpt_path, map_location="cpu")
        cfg = ckpt.get("config", {})
        state_dim = cfg.get("state_dim", 3)
        action_dim = cfg.get("action_dim", 2)
        action_low = cfg.get("action_low", np.array(
            [-1.0, -1.0], dtype=np.float32))
        action_high = cfg.get("action_high", np.array(
            [1.0, 1.0], dtype=np.float32))

        agent = PPOAgent(
            state_dim=state_dim,
            action_dim=action_dim,
            lr_actor=cfg.get("lr_actor", 3e-4),
            lr_critic=cfg.get("lr_critic", 1e-3),
            gamma=cfg.get("gamma", 0.995),
            epsilon_clip=cfg.get("epsilon_clip", 0.2),
            K_epochs=cfg.get("K_epochs", 10),
            entropy_coef=cfg.get("entropy_coef", 0.01),
            gae_lambda=cfg.get("gae_lambda", 0.98),
            action_space_low=action_low,
            action_space_high=action_high,
        )
        agent.actor.load_state_dict(ckpt["actor"])  # type: ignore[index]
        agent.critic.load_state_dict(ckpt["critic"])  # type: ignore[index]
        agent.actor.eval()
        agent.critic.eval()

        env = RoverNavEnv(seed=args.seed, normalized_actions=True)
        returns: List[float] = []
        for ep in range(args.eval_episodes):
            obs = env.reset()
            done = False
            steps = 0
            ep_ret = 0.0
            while not done and steps < args.max_ep_len:
                if args.deterministic:
                    with torch.no_grad():
                        st = torch.tensor(
                            obs, dtype=torch.float32, device=agent.device).unsqueeze(0)
                        mean, _std = agent.actor(st)
                        act = torch.clamp(
                            mean, agent.action_space_low, agent.action_space_high)
                        action = act.squeeze(0).cpu().numpy()
                else:
                    action, _ = agent.select_action(obs)

                obs, reward, done, _ = env.step(action)
                if args.render:
                    env.render(fps=args.render_fps)
                ep_ret += reward
                steps += 1
            returns.append(ep_ret)
            print(
                f"Episode {ep+1}/{args.eval_episodes} Return: {ep_ret:.3f} Steps: {steps}")
        print(
            f"Average Return over {args.eval_episodes} episodes: {float(np.mean(returns)):.3f}")
        env.close()
    else:
        train(
            total_updates=args.total_updates,
            rollout_steps=args.rollout_steps,
            max_ep_len=args.max_ep_len,
            lr_actor=args.lr_actor,
            lr_critic=args.lr_critic,
            gamma=args.gamma,
            epsilon_clip=args.epsilon_clip,
            K_epochs=args.K_epochs,
            entropy_coef=args.entropy_coef,
            gae_lambda=args.gae_lambda,
            seed=args.seed,
            save_dir=args.save_dir,
            save_every=args.save_every,
            render=args.render,
            render_fps=args.render_fps,
        )
