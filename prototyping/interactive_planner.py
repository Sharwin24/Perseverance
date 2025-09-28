# interactive_planner.py
import math
import pygame
import pygame_gui
import numpy as np
from dataclasses import asdict

from steering_planner import AckermannPlanner, RoverLimits
from rover_constants import WHEEL_LOCATIONS, STEERABLE_WHEELS

# --- Constants ---
SCREEN_WIDTH, SCREEN_HEIGHT = 1280, 800
UI_PANEL_WIDTH = 280
SIM_WIDTH = SCREEN_WIDTH - UI_PANEL_WIDTH
BG_COLOR = (20, 20, 20)
GRID_COLOR = (40, 40, 40)
GRID_SPACING = 50  # pixels

# --- World to Screen Transformation ---
METERS_TO_PIXELS = 100.0
WORLD_OFFSET_X = SIM_WIDTH / 2
WORLD_OFFSET_Y = SCREEN_HEIGHT / 2


def world_to_screen(x_m, y_m):
    """Converts world coordinates (meters) to screen coordinates (pixels)."""
    x_pix = WORLD_OFFSET_X + x_m * METERS_TO_PIXELS
    y_pix = WORLD_OFFSET_Y - y_m * METERS_TO_PIXELS  # Y is inverted
    return int(x_pix), int(y_pix)


def screen_to_world(x_pix, y_pix):
    """Converts screen coordinates (pixels) to world coordinates (meters)."""
    x_m = (x_pix - WORLD_OFFSET_X) / METERS_TO_PIXELS
    y_m = (WORLD_OFFSET_Y - y_pix) / METERS_TO_PIXELS
    return x_m, y_m

# --- Drawing Functions ---


def draw_grid(surface):
    for x in range(0, SIM_WIDTH, GRID_SPACING):
        pygame.draw.line(surface, GRID_COLOR, (x, 0), (x, SCREEN_HEIGHT))
    for y in range(0, SCREEN_HEIGHT, GRID_SPACING):
        pygame.draw.line(surface, GRID_COLOR, (0, y), (SIM_WIDTH, y))
    # Draw origin axes
    origin_x, origin_y = world_to_screen(0, 0)
    pygame.draw.line(surface, (100, 20, 20), (origin_x, 0),
                     (origin_x, SCREEN_HEIGHT), 2)
    pygame.draw.line(surface, (20, 100, 20), (0, origin_y),
                     (SIM_WIDTH, origin_y), 2)


def draw_pose(surface, pose, color, size=15):
    x, y, theta = pose
    px, py = world_to_screen(x, y)
    pygame.draw.circle(surface, color, (px, py), size, 3)
    end_x = px + size * 1.5 * math.cos(theta)
    end_y = py - size * 1.5 * math.sin(theta)  # Y inverted
    pygame.draw.line(surface, color, (px, py), (end_x, end_y), 3)


def draw_trajectory(surface, traj):
    if not traj or len(traj['x']) < 2:
        return
    points = [world_to_screen(x, y) for x, y in zip(traj['x'], traj['y'])]
    pygame.draw.aalines(surface, (200, 200, 255), False, points, 1)

# --- Main Application Class ---


class InteractivePlannerApp:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("Interactive Ackermann Planner")
        self.clock = pygame.time.Clock()
        self.is_running = True

        self.ui_manager = pygame_gui.UIManager(
            (SCREEN_WIDTH, SCREEN_HEIGHT), 'theme.json'
        )
        self.sim_surface = pygame.Surface((SIM_WIDTH, SCREEN_HEIGHT))

        # --- Planner and State ---
        self.limits = RoverLimits(
            wheel_diameter=0.15,
            wheel_speed_max=60,
            steer_angle_max=math.radians(30.0),
            accel_max=0.8,
            decel_max=1.2,
        )
        self.planner = None
        self.start_pose = (0.0, -1.0, math.pi / 2)
        self.goal_pose = (3.0, 2.0, 0.0)
        self.trajectory = None
        self.needs_replan = True

        # --- UI and Interaction State ---
        self.editing_mode = 'start'  # 'start' or 'goal'
        self.is_dragging = False
        self.drag_start_pos = None

        self.value_labels = {}
        self._create_ui()
        self._reinitialize_planner()

    def _create_ui(self):
        self.toggle_button = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect((SIM_WIDTH + 20, 20), (240, 40)),
            text='Editing: START',
            manager=self.ui_manager
        )

        self.sliders = {}
        y_offset = 80
        limit_ranges = {
            'wheel_speed_max': (30, 100),
            'steer_angle_max': (5.0, 60.0),  # Degrees
            'accel_max': (0.1, 5.0),
            'decel_max': (0.1, 5.0),
        }
        limit_units = {
            'wheel_speed_max': 'rpm',
            'steer_angle_max': 'deg',
            'accel_max': 'm/s²',
            'decel_max': 'm/s²',
        }

        for key, (min_val, max_val) in limit_ranges.items():
            unit = limit_units.get(key, '')
            pygame_gui.elements.UILabel(
                relative_rect=pygame.Rect(
                    (SIM_WIDTH + 20, y_offset), (180, 20)),
                text=f"{key.replace('_', ' ').title()} ({unit})",
                manager=self.ui_manager
            )

            current_val = getattr(self.limits, key)
            if 'angle' in key:
                current_val = math.degrees(current_val)

            value_label = pygame_gui.elements.UILabel(
                relative_rect=pygame.Rect(
                    (SIM_WIDTH + 200, y_offset), (60, 20)),
                text=f"{current_val:.2f}",
                manager=self.ui_manager
            )
            self.value_labels[key] = value_label
            y_offset += 25

            slider = pygame_gui.elements.UIHorizontalSlider(
                relative_rect=pygame.Rect(
                    (SIM_WIDTH + 20, y_offset), (240, 20)),
                start_value=current_val,
                value_range=(min_val, max_val),
                manager=self.ui_manager
            )
            self.sliders[key] = slider
            y_offset += 30

    def _reinitialize_planner(self):
        # Convert wheel locations from mm to m
        wheel_locations_m = {name: (x/1000.0, y/1000.0)
                             for name, (x, y) in WHEEL_LOCATIONS.items()}
        try:
            self.planner = AckermannPlanner(
                wheel_locations_m, STEERABLE_WHEELS, self.limits)
            self.needs_replan = True
        except ValueError as e:
            print(f"Error initializing planner: {e}")
            self.planner = None

    def _run_planner(self):
        if not self.planner or not self.needs_replan:
            return
        try:
            self.trajectory = self.planner.plan(
                self.start_pose, self.goal_pose)
            self.needs_replan = False
        except (RuntimeError, ValueError) as e:
            print(f"Could not plan path: {e}")
            self.trajectory = None

    def _handle_events(self):
        time_delta = self.clock.tick(60) / 1000.0
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                self.is_running = False

            # --- UI Events ---
            if event.type == pygame_gui.UI_BUTTON_PRESSED:
                if event.ui_element == self.toggle_button:
                    self.editing_mode = 'goal' if self.editing_mode == 'start' else 'start'
                    self.toggle_button.set_text(
                        f'Editing: {self.editing_mode.upper()}')

            if event.type == pygame_gui.UI_HORIZONTAL_SLIDER_MOVED:
                for key, slider in self.sliders.items():
                    if event.ui_element == slider:
                        new_val = slider.get_current_value()
                        self.value_labels[key].set_text(f"{new_val:.2f}")
                        if 'angle' in key:
                            setattr(self.limits, key, math.radians(new_val))
                        else:
                            setattr(self.limits, key, new_val)
                        self._reinitialize_planner()

            self.ui_manager.process_events(event)

            # --- Mouse Events for Pose Setting ---
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if event.pos[0] < SIM_WIDTH:  # Click is in the simulation area
                    self.is_dragging = True
                    self.drag_start_pos = event.pos
                    x_m, y_m = screen_to_world(*event.pos)
                    if self.editing_mode == 'start':
                        _, _, theta = self.start_pose
                        self.start_pose = (x_m, y_m, theta)
                    else:
                        _, _, theta = self.goal_pose
                        self.goal_pose = (x_m, y_m, theta)
                    self.needs_replan = True

            if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                self.is_dragging = False
                self.drag_start_pos = None

            if event.type == pygame.MOUSEMOTION and self.is_dragging:
                x_m, y_m = screen_to_world(*self.drag_start_pos)
                current_x_m, current_y_m = screen_to_world(*event.pos)
                theta = math.atan2(current_y_m - y_m, current_x_m - x_m)

                if self.editing_mode == 'start':
                    self.start_pose = (x_m, y_m, theta)
                else:
                    self.goal_pose = (x_m, y_m, theta)
                self.needs_replan = True

    def _update(self, time_delta):
        self.ui_manager.update(time_delta)
        self._run_planner()

    def _render(self):
        self.screen.fill((50, 50, 50))
        self.sim_surface.fill(BG_COLOR)

        draw_grid(self.sim_surface)
        draw_pose(self.sim_surface, self.start_pose, (100, 255, 100))
        draw_pose(self.sim_surface, self.goal_pose, (255, 100, 100))
        if self.trajectory:
            draw_trajectory(self.sim_surface, self.trajectory)

        self.screen.blit(self.sim_surface, (0, 0))
        self.ui_manager.draw_ui(self.screen)
        pygame.display.flip()

    def run(self):
        while self.is_running:
            time_delta = self.clock.tick(60) / 1000.0
            self._handle_events()
            self._update(time_delta)
            self._render()
        pygame.quit()


if __name__ == "__main__":
    app = InteractivePlannerApp()
    app.run()
