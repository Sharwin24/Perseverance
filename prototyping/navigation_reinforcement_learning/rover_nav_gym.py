
import math
from dataclasses import dataclass
from typing import Tuple, Optional

import numpy as np
import gymnasium as gym
from gymnasium.wrappers import TimeLimit
from gymnasium import spaces

try:
    import pygame
except ImportError:
    pygame = None


def rpm_to_rad_per_sec(rpm: float) -> float:
    return rpm * (2 * math.pi / 60)


def _angle_wrap(a: float) -> float:
    return ((a + math.pi) % (2 * math.pi)) - math.pi


@dataclass
class Pose:
    x: float
    y: float
    theta: float  # radians


class RewardFunctions:
    """Encapsulates reward shaping and success criteria for the rover nav task."""

    def __init__(
        self,
        progress_weight: float = 1.0,
        heading_penalty_weight: float = 0.01,
        turn_penalty_weight: float = 0.01,
        step_penalty: float = 0.01,
        success_bonus: float = 5.0,
        dist_threshold: float = 0.1,  # meters
        angle_threshold_rad: float = math.radians(5),
        max_w: float = math.radians(60)
    ):
        """Initializes the reward function parameters.

        Args:
            progress_weight (float, optional): Reward for progress towards the goal. Defaults to 1.5.
            heading_penalty_weight (float, optional): Penalty for heading error. Defaults to 0.05.
            turn_penalty_weight (float, optional): Penalty for turn error. Defaults to 0.01.
            step_penalty (float, optional): Penalty for each step taken. Defaults to 0.01.
            success_bonus (float, optional): Bonus for successfully reaching the goal. Defaults to 5.0.
            dist_threshold (float, optional): Distance threshold for success. Defaults to 0.1.
            max_w (float, optional): Maximum angular velocity. Defaults to math.radians(60).
        """
        self.k_progress = progress_weight
        self.k_bear = heading_penalty_weight
        self.k_head = turn_penalty_weight
        self.k_w = step_penalty
        self.k_v_slow = success_bonus
        self.step_penalty = step_penalty
        self.success_bonus = success_bonus
        self.dist_threshold = dist_threshold
        self.ang_threshold = angle_threshold_rad

        self.d_near = 0.6
        self.sig = 0.20               # meters for the sigmoid ramp
        self.max_w = max_w

        # optional: require dwell time at goal to mark success
        self.hold_steps = 10
        self._hold_counter = 0

    def _sigmoid(self, z):       # smooth weight ramp
        return 1.0 / (1.0 + math.exp(-z))

    def is_success(self, dist, ang_abs):
        return (dist < self.dist_threshold) and (ang_abs < self.ang_threshold)

    def reward(self, prev_dist, dist, bearing_err_abs, head_err_abs, v, w) -> float:
        # 1) progress toward goal (potential-like)
        r = self.k_progress * (prev_dist - dist)

        # 2) orientation shaping (blend)
        w_orient = self._sigmoid((self.d_near - dist) / max(self.sig, 1e-6))
        # Far => penalize bearing error; Near => penalize final heading error
        r -= (1 - w_orient) * self.k_bear * (bearing_err_abs / math.pi)
        r -= w_orient * self.k_head * (head_err_abs / math.pi)

        # 3) control smoothing (more strict near goal)
        turn_scale = 0.5 + 0.5 * w_orient
        r -= self.k_w * turn_scale * (abs(w) / self.max_w)

        # Slow down when very close, to stop overshoot & spin
        if dist < self.d_near:
            # discourage racing in the pocket
            r -= self.k_v_slow * max(0.0, abs(v) - 0.2)

        # 4) step penalty
        r -= self.step_penalty

        # 5) success bonus with dwell-time
        if self.is_success(dist, bearing_err_abs):
            self._hold_counter += 1
            if self._hold_counter >= self.hold_steps:
                r += self.success_bonus
        else:
            self._hold_counter = 0

        return float(r)


class RoverNavEnv(gym.Env):
    """
    Gymnasium-compatible environment for a simple unicycle robot navigating to a pose goal.
    Observation: [x_goal_in_robot, y_goal_in_robot, theta_error]
      - x,y in meters in the robot frame
      - theta_error in radians, wrapped to [-pi, pi]
    Action: [v, w] both normalized to [-1, 1]
      - v scaled to [-max_v, max_v]
      - w scaled to [-max_w, max_w]
    Episode ends when within distance & angle thresholds (terminated), or by TimeLimit (truncated).
    """

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 60}

    def __init__(
        self,
        world_size: Tuple[float, float] = (10.0, 10.0),
        dt: float = 0.05,
        seed: Optional[int] = None,
        pixels_per_meter: int = 80,
        render_mode: Optional[str] = None,
        normalized_actions: bool = True,
        max_v: float = 0.5,                # m/s
        max_w: float = math.radians(30),   # rad/s
    ):
        super().__init__()
        self.world_w, self.world_h = world_size
        self.dt = dt
        self.pixels_per_meter = pixels_per_meter
        self.normalized_actions = normalized_actions

        self.max_v = float(max_v)
        self.max_w = float(max_w)
        self.robot_len = 0.6  # m (triangle length along heading)
        self.robot_wid = 1.0  # m (triangle width)

        # RNG
        self.np_rng = np.random.default_rng(seed)

        # Spaces
        high_obs = np.array([max(self.world_w, self.world_h), max(
            self.world_w, self.world_h), math.pi], dtype=np.float32)
        self.observation_space = spaces.Box(
            low=-high_obs, high=high_obs, dtype=np.float32)

        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

        # State
        self.pose = Pose(1.0, 1.0, 0.0)
        self.goal = Pose(self.world_w - 1.0, self.world_h - 1.0, math.pi / 2)
        self._prev_goal_dist = self._goal_distance()

        # Rewards
        self.rewards = RewardFunctions(max_w=self.max_w)

        # Rendering
        self.render_mode = render_mode
        self._pg_inited = False
        self._screen = None
        self._clock = None
        self.window_w = int(self.world_w * self.pixels_per_meter)
        self.window_h = int(self.world_h * self.pixels_per_meter)
        self._bg_color = (245, 245, 245)
        self._grid_color = (220, 220, 220)
        self._robot_color = (255, 140, 0)   # orange
        self._goal_color = (34, 139, 34)    # green
        self._path_color = (0, 255, 0)      # green
        self.robot_path: list[Pose] = []  # Track the robot's path for drawing

    # ---- Core API ----
    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        if seed is not None:
            self.np_rng = np.random.default_rng(seed)

        self.pose = Pose(1.0, 1.0, 0.0)
        gx = float(self.np_rng.uniform(1.0, self.world_w - 1.0))
        gy = float(self.np_rng.uniform(1.0, self.world_h - 1.0))
        gth = float(self.np_rng.uniform(-math.pi, math.pi))
        self.goal = Pose(gx, gy, gth)
        self._prev_goal_dist = self._goal_distance()
        self.robot_path.clear()

        obs = self._get_obs()
        info = {}
        return obs, info

    def step(self, action: np.ndarray):
        action = np.asarray(action, dtype=np.float32)
        action = np.clip(action, -1.0, 1.0)

        v = float(action[0]) * self.max_v
        w = float(action[1]) * self.max_w

        # Unicycle update
        x, y, th = self.pose.x, self.pose.y, self.pose.theta
        x += v * math.cos(th) * self.dt
        y += v * math.sin(th) * self.dt
        th = _angle_wrap(th + w * self.dt)

        # Keep inside bounds
        x = float(np.clip(x, 0.0, self.world_w))
        y = float(np.clip(y, 0.0, self.world_h))
        self.pose = Pose(x, y, th)

        dist = self._goal_distance()
        dx, dy = self.goal.x - self.pose.x, self.goal.y - self.pose.y
        bearing_world = math.atan2(dy, dx)
        bearing_err = _angle_wrap(
            bearing_world - self.pose.theta)  # <- bearing-to-goal
        head_err = _angle_wrap(
            self.goal.theta - self.pose.theta)   # <- final heading
        bearing_abs, head_abs = abs(bearing_err), abs(head_err)
        reward = self.rewards.reward(
            self._prev_goal_dist, dist, bearing_abs, head_abs, v, w
        )
        self._prev_goal_dist = dist
        terminated = self.rewards.is_success(dist, bearing_abs)

        obs = self._get_obs()
        truncated = False  # handled by TimeLimit wrapper externally
        info = {}
        if self.render_mode == "human":
            self.robot_path.append(
                Pose(self.pose.x, self.pose.y, self.pose.theta))
            self.render()

        return obs, reward, terminated, truncated, info

    # ---- Helpers ----
    def _goal_distance(self) -> float:
        return float(math.hypot(self.goal.x - self.pose.x, self.goal.y - self.pose.y))

    def _get_obs(self) -> np.ndarray:
        # relative goal pose in robot frame
        dx = self.goal.x - self.pose.x
        dy = self.goal.y - self.pose.y
        c, s = math.cos(-self.pose.theta), math.sin(-self.pose.theta)
        x_r = c * dx - s * dy
        y_r = s * dx + c * dy
        th_err = _angle_wrap(self.goal.theta - self.pose.theta)
        return np.array([x_r, y_r, th_err], dtype=np.float32)

    # ---- Rendering ----
    def render(self):
        if self.render_mode is None:
            return
        if pygame is None:
            raise RuntimeError(
                "pygame is required for rendering: pip install pygame")

        if not self._pg_inited:
            pygame.init()
            self._screen = pygame.display.set_mode(
                (self.window_w, self.window_h))
            pygame.display.set_caption("RoverNavEnv (Gymnasium)")
            self._clock = pygame.time.Clock()
            self._pg_inited = True

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                self._pg_inited = False
                return

        self._screen.fill(self._bg_color)
        self._draw_grid()
        self._draw_triangle(self.goal, self._goal_color)
        self._draw_triangle(self.pose, self._robot_color)
        self._draw_path()
        pygame.display.flip()
        self._clock.tick(self.metadata.get("render_fps", 60))

    def close(self):
        if self._pg_inited and pygame is not None:
            pygame.quit()
            self._pg_inited = False

    # ---- Draw helpers ----
    def _world_to_screen(self, x: float, y: float):
        sx = int(x * self.pixels_per_meter)
        sy = int(self.window_h - y * self.pixels_per_meter)
        return sx, sy

    def _draw_grid(self):
        # vertical lines
        for i in range(0, int(self.world_w) + 1):
            x = i * self.pixels_per_meter
            pygame.draw.line(self._screen, self._grid_color,
                             (x, 0), (x, self.window_h), 1)
        # horizontal lines
        for j in range(0, int(self.world_h) + 1):
            y = self.window_h - j * self.pixels_per_meter
            pygame.draw.line(self._screen, self._grid_color,
                             (0, y), (self.window_w, y), 1)

    def _triangle_points(self, pose: Pose, length: float, width: float):
        L, W = length, width
        pts_local = np.array(
            [
                [L / 2, 0.0],  # tip
                [-L / 2, W / 2],
                [-L / 2, -W / 2],
            ],
            dtype=np.float32,
        )
        c, s = math.cos(pose.theta), math.sin(pose.theta)
        R = np.array([[c, -s], [s, c]], dtype=np.float32)
        pts_world = (R @ pts_local.T).T
        pts_world[:, 0] += pose.x
        pts_world[:, 1] += pose.y
        pts_screen = [self._world_to_screen(px, py) for px, py in pts_world]
        return pts_screen

    def _draw_triangle(self, pose: Pose, color):
        pts = self._triangle_points(pose, self.robot_len, self.robot_wid)
        pygame.draw.polygon(self._screen, color, pts)
        cx, cy = self._world_to_screen(pose.x, pose.y)
        hx = pose.x + 0.35 * self.robot_len * math.cos(pose.theta)
        hy = pose.y + 0.35 * self.robot_len * math.sin(pose.theta)
        hx, hy = self._world_to_screen(hx, hy)
        pygame.draw.line(self._screen, (30, 30, 30), (cx, cy), (hx, hy), 2)

    def _draw_path(self):
        if not self.robot_path or len(self.robot_path) < 2:
            return
        points = [self._world_to_screen(p.x, p.y) for p in self.robot_path]
        pygame.draw.lines(self._screen, self._path_color, False, points, 2)


if __name__ == "__main__":
    # Quick manual test
    env = RoverNavEnv(render_mode="human")

    env = TimeLimit(env, max_episode_steps=400)
    obs, info = env.reset()
    while True:
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        if terminated or truncated:
            obs, info = env.reset()
