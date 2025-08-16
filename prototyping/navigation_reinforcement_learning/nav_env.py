import math
import random
from dataclasses import dataclass
from typing import Tuple, Optional
import numpy as np
import pygame
from ppo import PPOAgent


# Simple 2D rover nav environment with pygame rendering
# - World units are meters; renderer scales to pixels
# - Robot is an orange triangle (front is the point)
# - Goal is a green triangle aligned to goal orientation
WHEEL_RADIUS = 0.05  # The wheel radius [m]
WHEEL_BASE = 1.0  # Wheel to Wheel distance along the Robot Y-axis [m]
# The back to front wheel distance along the Robot X-axis [m]
TRACK_WIDTH = 0.6


def rpm_to_rad_per_sec(rpm: float) -> float:
    return rpm * (2 * math.pi / 60)


@dataclass
class Pose:
    x: float
    y: float
    theta: float  # radians


class RewardFunctions:
    """Encapsulates reward shaping and success criteria for the rover nav task."""

    def __init__(self,
                 progress_weight: float = 1.5,
                 heading_penalty_weight: float = 0.05,
                 turn_penalty_weight: float = 0.01,
                 step_penalty: float = 0.01,
                 success_bonus: float = 5.0,
                 dist_threshold: float = 0.1,  # meters
                 angle_threshold_rad: float = math.radians(10)):
        self.progress_weight = progress_weight
        self.heading_penalty_weight = heading_penalty_weight
        self.turn_penalty_weight = turn_penalty_weight
        self.step_penalty = step_penalty
        self.success_bonus = success_bonus
        self.dist_threshold = dist_threshold
        self.angle_threshold_rad = angle_threshold_rad

    def is_success(self, dist: float, angle_error_abs: float) -> bool:
        return (dist < self.dist_threshold) and (angle_error_abs < self.angle_threshold_rad)

    def reward(self,
               prev_dist: float,
               dist: float,
               angle_error_abs: float,
               turn_rate: float,
               max_turn_rate: float,
               success: bool) -> float:
        progress = (prev_dist - dist)
        heading_pen = -self.heading_penalty_weight * \
            (angle_error_abs / math.pi)
        turn_pen = -self.turn_penalty_weight * \
            (abs(turn_rate) / max(max_turn_rate, 1e-6))
        r = self.progress_weight * progress + heading_pen + turn_pen - self.step_penalty
        if success:
            r += self.success_bonus
        return r


class RoverNavEnv:
    def __init__(self,
                 world_size: Tuple[float, float] = (10.0, 10.0),  # meters
                 dt: float = 0.05,
                 seed: Optional[int] = None,
                 pixels_per_meter: int = 80,
                 window_size: Tuple[int, int] = (800, 800),
                 normalized_actions: bool = True):
        self.world_w, self.world_h = world_size
        self.dt = dt
        self.rng = np.random.default_rng(seed)
        self.ppm = pixels_per_meter
        self.window_w, self.window_h = window_size
        self.normalized_actions = normalized_actions

        # Robot parameters
        self.max_v = 1.5  # m/s
        self.max_w = rpm_to_rad_per_sec(60)  # rad/s
        self.robot_len = TRACK_WIDTH  # m, triangle base length along heading
        self.robot_wid = WHEEL_BASE  # m, triangle base width

        # Initial states
        self.pose = Pose(1.0, 1.0, 0.0)
        self.goal = Pose(self.world_w - 1.0, self.world_h - 1.0, math.pi / 2)
        self._prev_goal_dist = self._goal_distance()

        # Rewards
        self.rewards = RewardFunctions()

        # Pygame
        self._pg_inited = False
        self._screen = None
        self._clock = None
        self._bg_color = (245, 245, 245)
        self._grid_color = (220, 220, 220)
        self._robot_color = (255, 140, 0)   # orange
        self._goal_color = (34, 139, 34)    # green

    # --- Env API ---
    def reset(self, pose: Optional[Pose] = None, goal: Optional[Pose] = None):
        self.pose = pose if pose is not None else Pose(1.0, 1.0, 0.0)
        if goal is None:
            gx = float(self.rng.uniform(1.0, self.world_w - 1.0))
            gy = float(self.rng.uniform(1.0, self.world_h - 1.0))
            gth = float(self.rng.uniform(-math.pi, math.pi))
            self.goal = Pose(gx, gy, gth)
        else:
            self.goal = goal
        self._prev_goal_dist = self._goal_distance()
        return self._get_obs()

    def step(self, action: Tuple[float, float]):
        # action = (v, w) in env units, or normalized in [-1,1] if enabled
        if self.normalized_actions:
            v = float(np.clip(action[0], -1.0, 1.0)) * self.max_v
            w = float(np.clip(action[1], -1.0, 1.0)) * self.max_w
        else:
            v = float(np.clip(action[0], -self.max_v, self.max_v))
            w = float(np.clip(action[1], -self.max_w, self.max_w))

        # Unicycle update
        x, y, th = self.pose.x, self.pose.y, self.pose.theta
        x += v * math.cos(th) * self.dt
        y += v * math.sin(th) * self.dt
        th += w * self.dt
        th = ((th + math.pi) % (2 * math.pi)) - math.pi

        # Keep inside bounds
        x = float(np.clip(x, 0.0, self.world_w))
        y = float(np.clip(y, 0.0, self.world_h))
        self.pose = Pose(x, y, th)

        # Reward via RewardFunctions
        dist = self._goal_distance()
        th_err = abs(self._angle_error())
        done = self.rewards.is_success(dist, th_err)
        reward = self.rewards.reward(
            self._prev_goal_dist, dist, th_err, w, self.max_w, done)

        self._prev_goal_dist = dist

        obs = self._get_obs()
        info = {}
        return obs, reward, done, info

    def _get_obs(self):
        # Simple observation: relative goal pose in robot frame
        dx = self.goal.x - self.pose.x
        dy = self.goal.y - self.pose.y
        c, s = math.cos(-self.pose.theta), math.sin(-self.pose.theta)
        x_r = c * dx - s * dy
        y_r = s * dx + c * dy
        th_err = self._angle_wrap(self.goal.theta - self.pose.theta)
        return np.array([x_r, y_r, th_err], dtype=np.float32)

    # --- Helpers ---
    def _goal_distance(self) -> float:
        return math.hypot(self.goal.x - self.pose.x, self.goal.y - self.pose.y)

    def _angle_error(self) -> float:
        return self._angle_wrap(self.goal.theta - self.pose.theta)

    @staticmethod
    def _angle_wrap(a: float) -> float:
        return ((a + math.pi) % (2 * math.pi)) - math.pi

    # --- Rendering ---
    def render(self, grid: bool = True, fps: int = 60):
        if not self._pg_inited:
            pygame.init()
            self._screen = pygame.display.set_mode(
                (self.window_w, self.window_h))
            pygame.display.set_caption("RoverNavEnv")
            self._clock = pygame.time.Clock()
            self._pg_inited = True

        # Handle quit events to close window gracefully
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                self._pg_inited = False
                return

        self._screen.fill(self._bg_color)
        if grid:
            self._draw_grid()

        # Draw goal and robot
        self._draw_triangle(self.goal, self._goal_color)
        self._draw_triangle(self.pose, self._robot_color)

        pygame.display.flip()
        if fps > 0:
            self._clock.tick(fps)

    def close(self):
        if self._pg_inited:
            pygame.quit()
            self._pg_inited = False

    # --- Drawing helpers ---
    def _world_to_screen(self, x: float, y: float) -> Tuple[int, int]:
        # World origin at bottom-left; Pygame origin at top-left
        sx = int(x * self.ppm)
        sy = int(self.window_h - y * self.ppm)
        return sx, sy

    def _draw_grid(self):
        # vertical lines
        for i in range(0, int(self.world_w) + 1):
            x = i * self.ppm
            pygame.draw.line(self._screen, self._grid_color,
                             (x, 0), (x, self.window_h), 1)
        # horizontal lines
        for j in range(0, int(self.world_h) + 1):
            y = self.window_h - j * self.ppm
            pygame.draw.line(self._screen, self._grid_color,
                             (0, y), (self.window_w, y), 1)

    def _triangle_points(self, pose: Pose, length: float, width: float):
        # Triangle in local frame: front tip at (L/2, 0), base corners at (-L/2, +/-W/2)
        L = length
        W = width
        pts_local = np.array([
            [L/2,   0.0],   # tip
            [-L/2,  W/2],    # rear-left
            [-L/2, -W/2],    # rear-right
        ], dtype=np.float32)
        c, s = math.cos(pose.theta), math.sin(pose.theta)
        R = np.array([[c, -s], [s, c]], dtype=np.float32)
        pts_world = (R @ pts_local.T).T
        pts_world[:, 0] += pose.x
        pts_world[:, 1] += pose.y
        pts_screen = [self._world_to_screen(px, py) for px, py in pts_world]
        return pts_screen

    def _draw_triangle(self, pose: Pose, color: Tuple[int, int, int]):
        pts = self._triangle_points(pose, self.robot_len, self.robot_wid)
        pygame.draw.polygon(self._screen, color, pts)
        # small heading line from center for clarity
        cx, cy = self._world_to_screen(pose.x, pose.y)
        hx = pose.x + 0.35 * self.robot_len * math.cos(pose.theta)
        hy = pose.y + 0.35 * self.robot_len * math.sin(pose.theta)
        hx, hy = self._world_to_screen(hx, hy)
        pygame.draw.line(self._screen, (30, 30, 30), (cx, cy), (hx, hy), 2)
