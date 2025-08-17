# play_sb3.py
from stable_baselines3 import PPO
from rover_nav_gym import RoverNavEnv
import pygame

# 1) Make a *single* non-vectorized env with a window
env = RoverNavEnv(render_mode="human")
obs, info = env.reset(seed=0)

# 2) Load your trained model
model = PPO.load("./checkpoints/ppo_rover_nav", device="cpu")

# 3) Rollout loop
while True:
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(action)
    env.render()  # pygame window
    if terminated or truncated:
        obs, info = env.reset()
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN and event.key == pygame.K_r:
            obs, info = env.reset()
        if event.type == pygame.KEYDOWN and (event.key == pygame.K_q or event.key == pygame.K_ESCAPE):
            pygame.quit()
            break
