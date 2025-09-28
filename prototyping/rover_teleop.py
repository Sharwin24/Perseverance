import matplotlib.pyplot as plt
from typing import Dict, Tuple
import numpy as np
from rover_constants import WHEEL_LOCATIONS, WHEEL_BASE, \
    WHEEL_DIAMETER, TRACK_WIDTH_MIDDLE, TRACK_WIDTH_STEERING, \
    ROVER_BODY_LENGTH, ROVER_BODY_WIDTH, STEERABLE_WHEELS
import pygame as pg
from typing import Dict

SCREEN_WIDTH, SCREEN_HEIGHT = 1200, 1200


class Pose:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta  # In radians

    def __add__(self, other):
        return Pose(
            x=self.x + other.x,
            y=self.y + other.y,
            theta=self.theta + other.theta
        )

    def __sub__(self, other):
        return Pose(
            x=self.x - other.x,
            y=self.y - other.y,
            theta=self.theta - other.theta
        )


class Twist:
    """
    Our own implementation of geometry_msgs/Twist
    for a 2D rover, we need only worry about linear.x, linear.y, and angular.z
    """

    def __init__(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        self.linear_x = linear_x
        self.linear_y = linear_y
        self.angular_z = angular_z

    def __add__(self, other):
        return Twist(
            linear_x=self.linear_x + other.linear_x,
            linear_y=self.linear_y + other.linear_y,
            angular_z=self.angular_z + other.angular_z
        )

    def __sub__(self, other):
        return Twist(
            linear_x=self.linear_x - other.linear_x,
            linear_y=self.linear_y - other.linear_y,
            angular_z=self.angular_z - other.angular_z
        )


def _normalize_angle(a: float) -> float:
    a = (a + np.pi) % (2*np.pi) - np.pi
    return float(a)


def twist_to_wheels_icr(cmd: Twist, eps_omega: float = 1e-4):
    """
    Project to feasible twist for fixed middle wheels (vy -> 0),
    compute ICR (if omega != 0), per-wheel steer angles for the 4 steerables,
    and wheel speeds for all 6. Also returns per-wheel lateral slip (should be ~0).
    """
    vx, vy, w = float(cmd.linear_x), float(cmd.linear_y), float(cmd.angular_z)
    vy_feas = 0.0
    feas = Twist(vx, vy_feas, w)

    wheel_angles: Dict[str, float] = {k: 0.0 for k in WHEEL_LOCATIONS.keys()}
    wheel_speeds: Dict[str, float] = {}
    slip_error: Dict[str, float] = {}

    # ICR
    if abs(w) > eps_omega:
        x_c = -vy_feas / w
        y_c = vx / w
        p_c = np.array([x_c, y_c])
    else:
        p_c = None  # "ICR at infinity"

    for name, (x_i, y_i) in WHEEL_LOCATIONS.items():
        v_i = np.array([vx - w*y_i, vy_feas + w*x_i])   # contact vel at wheel

        if name in STEERABLE_WHEELS:
            if p_c is not None:
                t = np.array([-(y_i - p_c[1]), (x_i - p_c[0])])  # tangent
                if np.allclose(t, 0.0):
                    t = v_i
                delta = float(np.arctan2(t[1], t[0]))
                u = t / (np.linalg.norm(t) + 1e-12)
            else:
                # straight; align with +x since vy_feas=0
                delta = 0.0
                u = np.array([1.0, 0.0])
            wheel_angles[name] = _normalize_angle(delta)
            s = float(v_i @ u)
            n = np.array([-u[1], u[0]])
            wheel_speeds[name] = s
            slip_error[name] = float(v_i @ n)
        else:
            # middle fixed: heading +x
            u = np.array([1.0, 0.0])
            s = float(v_i @ u)        # = vx - w*y_i
            wheel_speeds[name] = s
            slip_error[name] = float(v_i[1])  # lateral component
            wheel_angles[name] = 0.0

    return p_c, wheel_angles, wheel_speeds, feas, slip_error


def plot_steering_geometry(twist: Twist, title: str = ""):
    p_c, ang, spd, feas, slip = twist_to_wheels_icr(twist)

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_aspect("equal", "box")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.set_xlabel("Body X")
    ax.set_ylabel("Body Y")
    ax.set_title(
        title or f"Twist: vx={feas.linear_x:.2f}, vy={feas.linear_y:.2f}, w={feas.angular_z:.3f}")

    # Draw body rectangle (approx) for context
    ax.add_patch(plt.Rectangle((-ROVER_BODY_LENGTH/2, -ROVER_BODY_WIDTH/2),
                               ROVER_BODY_LENGTH, ROVER_BODY_WIDTH,
                               fill=False, lw=1.5))

    # Draw ICR (if finite) and turning circle through body origin
    if p_c is not None:
        ax.plot(p_c[0], p_c[1], "o", ms=6, label="ICR")
        R = np.hypot(p_c[0], p_c[1])
        circle = plt.Circle((p_c[0], p_c[1]), R, fill=False, ls=":", lw=1.2)
        ax.add_patch(circle)

    # For each wheel: draw pos, heading, speed vector, lateral slip
    for name, (x, y) in WHEEL_LOCATIONS.items():
        ax.plot(x, y, "ko", ms=4)
        ax.text(x + 0.01*ROVER_BODY_LENGTH, y + 0.01 *
                ROVER_BODY_WIDTH, name, fontsize=8)

        # Heading ray (blue for steerable, gray for fixed)
        delta = ang[name]
        u = np.array([np.cos(delta), np.sin(delta)])
        color = "C0" if name in STEERABLE_WHEELS else "0.5"
        ax.arrow(x, y, 0.15*ROVER_BODY_LENGTH*u[0], 0.15*ROVER_BODY_LENGTH*u[1],
                 head_width=0.02*ROVER_BODY_LENGTH, length_includes_head=True,
                 lw=2, color=color)

        # Contact velocity vector (green)
        vx = feas.linear_x - feas.angular_z * y
        vy = feas.linear_y + feas.angular_z * x
        ax.arrow(x, y, 0.12*ROVER_BODY_LENGTH*vx/max(1.0, abs(feas.linear_x)+abs(feas.angular_z)*max(1.0, ROVER_BODY_LENGTH)),
                 0.12*ROVER_BODY_LENGTH*vy /
                 max(1.0, abs(feas.linear_x)+abs(feas.angular_z)
                     * max(1.0, ROVER_BODY_LENGTH)),
                 head_width=0.02*ROVER_BODY_LENGTH, color="C2", length_includes_head=True, lw=1.5)

        # Lateral slip component (red, should be ~0)
        n = np.array([-u[1], u[0]])
        v = np.array([vx, vy])
        lat = float(v @ n)
        ax.arrow(x, y, 0.12*ROVER_BODY_LENGTH*lat*n[0],
                 0.12*ROVER_BODY_LENGTH*lat*n[1],
                 head_width=0.02*ROVER_BODY_LENGTH, color="C3",
                 length_includes_head=True, lw=1.2, alpha=0.9)

    ax.legend(loc="upper right")
    # auto-limits
    xs = [p[0] for p in WHEEL_LOCATIONS.values()]
    ys = [p[1] for p in WHEEL_LOCATIONS.values()]
    pad = 0.3*max(ROVER_BODY_LENGTH, ROVER_BODY_WIDTH)
    ax.set_xlim(min(xs)-pad, max(xs)+pad)
    ax.set_ylim(min(ys)-pad, max(ys)+pad)
    plt.show()


def rocker_bogie_forward_kinematics(wheel_drive_speeds, wheel_steer_angles) -> Twist:
    """
        Calculates the robot's body frame velocities (v_bx, v_by, omega_b)
        for a 6-wheeled rocker-bogie using a least-squares fit.

        Args:
            wheel_positions (dict): A dictionary mapping wheel names to their (x, y)
                                    coordinate tuples relative to the robot's center.
                                    e.g., {'fl': (x_fl, y_fl), ...}
            wheel_drive_speeds (dict): A dictionary mapping wheel names to their
                                    tangential drive speed (e.g., from encoders).
                                    e.g., {'fl': v_fl, ...}
            wheel_steer_angles (dict): A dictionary mapping wheel names to their
                                    current steering angle in radians.
                                    e.g., {'fl': delta_fl, ...}

        Returns:
            numpy.ndarray: A 3-element array containing [v_bx, v_by, omega_b].
        """
    # Define the order of wheels to ensure consistent matrix construction.
    # This order must match the order of data in the dictionaries.
    wheel_names = WHEEL_LOCATIONS.keys()

    # Initialize the A (12x3) and b (12x1) matrices with zeros.
    A = np.zeros((12, 3))
    b = np.zeros((12, 1))

    # Loop through each of the 6 wheels to populate the matrices.
    for i, name in enumerate(wheel_names):
        # Get the wheel's position, speed, and steering angle.
        x_i, y_i = WHEEL_LOCATIONS[name]
        v_i = wheel_drive_speeds[name]
        delta_i = wheel_steer_angles[name]

        # Calculate the starting row index for this wheel in the matrices.
        # Each wheel contributes two rows.
        row_start = i * 2

        # --- Populate the A matrix for the current wheel ---
        # Equation for the x-component of the wheel's velocity:
        # v_ix = v_bx - omega_b * y_i
        A[row_start, 0] = 1
        A[row_start, 1] = 0
        A[row_start, 2] = -y_i

        # Equation for the y-component of the wheel's velocity:
        # v_iy = v_by + omega_b * x_i
        A[row_start + 1, 0] = 0
        A[row_start + 1, 1] = 1
        A[row_start + 1, 2] = x_i

        # --- Populate the b vector for the current wheel ---
        # The observed x-component of the wheel's velocity vector.
        b[row_start, 0] = v_i * np.cos(delta_i)

        # The observed y-component of the wheel's velocity vector.
        b[row_start + 1, 0] = v_i * np.sin(delta_i)

    # --- Solve the system A * x = b for x ---
    # We use the Moore-Penrose pseudo-inverse (np.linalg.pinv) because it's
    # numerically stable and finds the best-fit (least-squares) solution
    # for an overdetermined system.
    # The solution x will be the [v_bx, v_by, omega_b] vector.
    try:
        body_vel_solution = np.linalg.pinv(A) @ b
    except np.linalg.LinAlgError:
        # This is unlikely but good practice to handle.
        # It could happen if A is a singular matrix (e.g., all wheels at the origin).
        return Twist()

    # Return as a 1D array [v_bx, v_by, omega_b]
    body_twist = body_vel_solution.flatten()
    return Twist(linear_x=body_twist[0],
                 linear_y=body_twist[1],
                 angular_z=body_twist[2])


# Control parameters
LINEAR_SPEED = 150.0   # pixels per second (acts like mm/s for screen)
# Ackermann curvature change rate (1/pixel per second); tuned for feel
KAPPA_RATE = 0.004
# Steering parameters (outer 4 wheels only)
# rad/s change when holding keys (legacy, unused with Ackermann)
STEER_RATE = np.deg2rad(60.0)
MAX_STEER_DEG = 30.0            # degrees limit for steering
# Visualization threshold: hide turning circle if steer angles are tiny
TURN_RADIUS_VIS_ANGLE_DEG = 2.0
# Point-turn angular rate when spinning in place with 'K' held (rad/s)
TURN_IN_PLACE_RATE = np.deg2rad(90.0)
VELOCITY_ARROW_SCALE = 0.0025  # seconds of motion to scale linear arrow length
VEL_ARROW_COLOR = (40, 160, 80)
ANGULAR_ARROW_COLOR = (40, 80, 160)


def compute_twist_from_keys(keys) -> Twist:
    """Compute body-frame twist from current key state (held keys).
    W/S: forward/back. Steering handled via Ackermann curvature separately.
    """
    vx = 0.0
    if keys[pg.K_w]:
        vx += LINEAR_SPEED
    if keys[pg.K_s]:
        vx -= LINEAR_SPEED
    # Ackermann steering sets omega later via omega = vx * kappa; vy = 0
    return Twist(vx, 0.0, 0.0)


def clamp_kappa_to_limits(kappa: float) -> float:
    """Clamp curvature so no steerable wheel exceeds MAX_STEER_DEG.

    We assume an instantaneous center of curvature (ICC) at (0, R) in body frame
    so that for each steerable wheel at (x_i, y_i): tan(delta_i) = x_i / (R - y_i).
    We limit kappa = 1/R so that max(|delta_i|) <= MAX_STEER_DEG.
    """
    if abs(kappa) < 1e-12:
        return 0.0
    sign = 1.0 if kappa > 0 else -1.0  # +: left turn, -: right turn
    R = 1.0 / abs(kappa)
    tan_max = np.tan(np.deg2rad(MAX_STEER_DEG))
    # Inner side wheels limit the angle: y has same sign as turn direction
    steerable = {"front_left", "front_right", "rear_left", "rear_right"}
    min_R_required = 0.0
    for name, (x_i, y_i) in WHEEL_LOCATIONS.items():
        if name not in steerable:
            continue
        # Consider only inner-side wheels for the current turn direction
        if sign * y_i <= 0:
            continue
        required = sign * y_i + abs(x_i) / max(tan_max, 1e-9)
        if required > min_R_required:
            min_R_required = required
    if min_R_required > 0.0 and R < min_R_required:
        R = min_R_required
    return sign / R


def compute_ackermann_wheel_angles(kappa: float) -> Dict[str, float]:
    """Compute per-wheel steer angles for Ackermann steering with curvature kappa.

    Curvature kappa = 1/R, positive for left turns (ICC at +Y). Angles are
    relative to body +X axis, positive CCW (toward +Y).
    """
    angles: Dict[str, float] = {
        "front_left": 0.0,
        "front_right": 0.0,
        "rear_left": 0.0,
        "rear_right": 0.0,
    }
    if abs(kappa) < 1e-12:
        return angles
    R = 1.0 / kappa  # signed
    for name in ("front_left", "front_right", "rear_left", "rear_right"):
        x_i, y_i = WHEEL_LOCATIONS[name]
        # tan(delta) = x_i / (R - y_i) -> delta = atan2(x_i, R - y_i)
        angles[name] = float(np.arctan2(x_i, (R - y_i)))
    return angles


def compute_point_turn_wheel_angles() -> Dict[str, float]:
    """Wheel angles for a point-turn about the robot center.

    For pure rotation (v=0, omega!=0), the wheel rolling direction at (x,y)
    must align with the tangent vector [-y, x]. Therefore, the steering angle
    relative to +X is delta = atan2(x, -y).

    Only the four outer steerable wheels are rotated; middle wheels remain fixed.
    """
    angles: Dict[str, float] = {
        "front_left": 0.0,
        "front_right": 0.0,
        "rear_left": 0.0,
        "rear_right": 0.0,
    }
    for name in ("front_left", "front_right", "rear_left", "rear_right"):
        x_i, y_i = WHEEL_LOCATIONS[name]
        angles[name] = float(np.arctan2(x_i, -y_i))
    return angles


def build_full_wheel_angles(base_angles: Dict[str, float]) -> Dict[str, float]:
    """Ensure angles for all wheels exist, defaulting non-steerable to 0."""
    full: Dict[str, float] = {}
    for name in WHEEL_LOCATIONS.keys():
        full[name] = float(base_angles.get(name, 0.0))
    return full


def compute_wheel_speeds_from_twist(twist: Twist, wheel_angles: Dict[str, float]) -> Dict[str, float]:
    """Project the wheel contact velocity onto each wheel's rolling direction.

    v_wheel_vec(x,y) = [v_bx, v_by] + omega * [-y, x]
    speed_i = dot(v_wheel_vec, u_i), where u_i = [cos(delta_i), sin(delta_i)]
    """
    speeds: Dict[str, float] = {}
    for name, (x_i, y_i) in WHEEL_LOCATIONS.items():
        vx = twist.linear_x - twist.angular_z * y_i
        vy = twist.linear_y + twist.angular_z * x_i
        delta = float(wheel_angles.get(name, 0.0))
        ux = np.cos(delta)
        uy = np.sin(delta)
        speeds[name] = float(vx * ux + vy * uy)
    return speeds


def draw_arrow(screen, start, end, color=(0, 0, 0), width=2, head_len=10, head_angle_deg=28):
    """Draw a line with an arrowhead from start to end."""
    x0, y0 = start
    x1, y1 = end
    pg.draw.line(screen, color, (x0, y0), (x1, y1), width)
    dx = x1 - x0
    dy = y1 - y0
    ang = np.arctan2(dy, dx)
    ha = np.deg2rad(head_angle_deg)
    lx = x1 - head_len * np.cos(ang - ha)
    ly = y1 - head_len * np.sin(ang - ha)
    rx = x1 - head_len * np.cos(ang + ha)
    ry = y1 - head_len * np.sin(ang + ha)
    pg.draw.polygon(screen, color, [(x1, y1), (lx, ly), (rx, ry)])


def draw_body_velocity(screen, pose: Pose, twist_fk: Twist):
    """Visualize body-frame FK velocities: linear vector and angular curved arrow."""
    # Linear velocity vector -> world
    vx_b = twist_fk.linear_x
    vy_b = twist_fk.linear_y
    omega = twist_fk.angular_z
    speed = np.hypot(vx_b, vy_b)
    if speed > 1e-6:
        theta = pose.theta
        vx_w = vx_b * np.cos(theta) - vy_b * np.sin(theta)
        vy_w = vx_b * np.sin(theta) + vy_b * np.cos(theta)
        # Scale arrow length by time constant
        L = speed * VELOCITY_ARROW_SCALE
        # Start at robot center in screen coords
        cx = int(SCREEN_WIDTH // 2 + pose.x)
        cy = int(SCREEN_HEIGHT // 2 - pose.y)
        end = (int(cx + L * vx_w), int(cy - L * vy_w))
        draw_arrow(screen, (cx, cy), end,
                   VEL_ARROW_COLOR, width=3, head_len=12)

    # Angular curved arrow around center
    if abs(omega) > 1e-3:
        cx = int(SCREEN_WIDTH // 2 + pose.x)
        cy = int(SCREEN_HEIGHT // 2 - pose.y)
        r = 80
        sweep = float(np.clip(abs(omega) * 0.9, 0.4, np.pi))
        # Screen Y grows downward; invert sign to get correct visual CCW
        sgn = -1.0 if omega > 0 else 1.0
        # Draw arc segments
        segments = 128
        dphi = sgn * sweep / segments
        # Start angle relative to screen X-axis
        phi0 = 0.0
        x_prev = cx + int(r * np.cos(phi0))
        y_prev = cy + int(r * np.sin(phi0))
        for i in range(1, segments + 1):
            phi = phi0 + i * dphi
            x_cur = cx + int(r * np.cos(phi))
            y_cur = cy + int(r * np.sin(phi))
            pg.draw.line(screen, ANGULAR_ARROW_COLOR,
                         (x_prev, y_prev), (x_cur, y_cur), 3)
            x_prev, y_prev = x_cur, y_cur
        # Arrowhead at arc end
        head_len = 12
        ang = phi0 + segments * dphi
        # Tangent direction at end
        tx = -np.sin(ang)
        ty = np.cos(ang)
        # Orient tangent to follow arc direction
        if sgn < 0:
            tx, ty = -tx, -ty
        endx = cx + int(r * np.cos(ang))
        endy = cy + int(r * np.sin(ang))
        draw_arrow(screen, (endx - int(tx * head_len), endy - int(ty * head_len)),
                   (endx, endy), ANGULAR_ARROW_COLOR, width=3, head_len=10)


def update_pose_from_twist(current_pose: Pose, twist: Twist, dt: float) -> Pose:
    """Integrate body-frame twist into world-frame pose using heading theta."""
    theta = current_pose.theta
    # Body -> World rotation
    dx_world = (twist.linear_x * np.cos(theta) -
                twist.linear_y * np.sin(theta)) * dt
    dy_world = (twist.linear_x * np.sin(theta) +
                twist.linear_y * np.cos(theta)) * dt
    dtheta = twist.angular_z * dt
    return Pose(
        x=current_pose.x + dx_world,
        y=current_pose.y + dy_world,
        theta=current_pose.theta + dtheta,
    )


def draw_turning_radius(screen, pose: Pose, kappa: float):
    """Draw the current turning circle (dashed) and ICC center when steering.

    - Only draws when |kappa| > tiny threshold (non-centered steering)
    - Circle is drawn in world frame, then mapped to screen
    - ICC is at body-frame (0, R) with R = 1/kappa (signed)
    """
    if abs(kappa) < 1e-12:
        return
    R = 1.0 / kappa  # signed radius; positive left, negative right
    theta = pose.theta
    # Hide if all steerable wheel angles are within threshold
    angle_thresh = np.deg2rad(TURN_RADIUS_VIS_ANGLE_DEG)
    max_abs_angle = 0.0
    for name in ("front_left", "front_right", "rear_left", "rear_right"):
        x_i, y_i = WHEEL_LOCATIONS[name]
        delta_i = float(np.arctan2(x_i, (R - y_i)))
        max_abs_angle = max(max_abs_angle, abs(delta_i))
    if max_abs_angle < angle_thresh:
        return
    # ICC world coordinates from body (0, R)
    icc_x = pose.x + (-np.sin(theta)) * R
    icc_y = pose.y + (np.cos(theta)) * R

    # Convert to screen coords
    scr_cx = int(SCREEN_WIDTH // 2 + icc_x)
    scr_cy = int(SCREEN_HEIGHT // 2 - icc_y)
    radius_center = abs(R)
    # Compute average lateral offsets for left/right wheels
    y_left_vals = [y for name,
                   (x, y) in WHEEL_LOCATIONS.items() if 'left' in name]
    y_right_vals = [y for name,
                    (x, y) in WHEEL_LOCATIONS.items() if 'right' in name]
    y_left_avg = np.mean(
        y_left_vals) if y_left_vals else TRACK_WIDTH_STEERING / 2.0
    y_right_avg = np.mean(y_right_vals) if y_right_vals else - \
        TRACK_WIDTH_STEERING / 2.0
    radius_left = abs(R - y_left_avg)
    radius_right = abs(R - y_right_avg)

    # Visual params
    color_left = (240, 140, 60)   # orange-ish for left wheel path
    color_center = (120, 180, 255)  # blue for center path
    color_right = (160, 120, 240)  # purple for right wheel path
    center_color = (20, 120, 240)
    width = 1

    def draw_dashed_circle(cx, cy, rad, color):
        rad_i = int(max(1, rad))
        # Keep segment count bounded for performance and proportional to circumference
        max_segments = 360
        segments = max(48, min(max_segments, int(2 * np.pi * rad_i / 8)))
        dphi = 2 * np.pi / segments
        for i in range(segments):
            if i % 2 == 1:
                continue  # gap
            a0 = i * dphi
            a1 = (i + 1) * dphi
            x0 = cx + int(rad_i * np.cos(a0))
            y0 = cy + int(rad_i * np.sin(a0))
            x1 = cx + int(rad_i * np.cos(a1))
            y1 = cy + int(rad_i * np.sin(a1))
            pg.draw.line(screen, color, (x0, y0), (x1, y1), width)

    # Draw three concentric dashed circles
    draw_dashed_circle(scr_cx, scr_cy, radius_left, color_left)
    draw_dashed_circle(scr_cx, scr_cy, radius_center, color_center)
    draw_dashed_circle(scr_cx, scr_cy, radius_right, color_right)

    # Draw ICC center
    pg.draw.circle(screen, center_color, (scr_cx, scr_cy), 4)


def draw_robot(screen, position, angle, steer_angles: Dict[str, float]):
    """Draw top-down rover with body and six wheels using body-frame geometry.

    - Build an off-screen surface in body frame (x right/forward, y up/left)
    - Draw body rectangle and wheels aligned with body X axis
    - Rotate by pose angle (CCW positive) and blit centered at position
    """
    # Colors
    body_color = (30, 30, 30)
    front_marker = (255, 60, 60)
    left_wheel_color = (50, 120, 255)
    right_wheel_color = (255, 80, 80)
    outline = (15, 15, 15)

    # Dimensions from constants (in pixels, consistent with our simple sim)
    body_len = ROVER_BODY_LENGTH
    body_wid = ROVER_BODY_WIDTH
    wheel_length = WHEEL_DIAMETER  # along body X
    # along body Y (visual thickness)
    wheel_width = WHEEL_DIAMETER / 4

    # Determine extents to size the body-frame canvas
    half_len = max(WHEEL_BASE / 2 + wheel_length / 2, body_len / 2) + 20
    half_wid = max(TRACK_WIDTH_MIDDLE / 2 + wheel_width / 2,
                   TRACK_WIDTH_STEERING / 2 + wheel_width / 2,
                   body_wid / 2) + 20
    surf_w, surf_h = int(2 * half_len), int(2 * half_wid)

    # Create transparent surface and define its center
    surf = pg.Surface((surf_w, surf_h), pg.SRCALPHA)
    cx, cy = surf_w // 2, surf_h // 2

    def to_surf(pt):
        bx, by = pt
        return int(cx + bx), int(cy - by)  # invert Y for screen

    # Draw body rectangle centered at (0,0) in body frame
    body_rect = pg.Rect(0, 0, int(body_len), int(body_wid))
    body_rect.center = (cx, cy)
    pg.draw.rect(surf, body_color, body_rect, border_radius=8)
    pg.draw.rect(surf, outline, body_rect, width=2, border_radius=8)

    # Front marker triangle at +X edge in body frame
    tip_b = (body_len / 2 - 6, 0)
    base1_b = (body_len / 2 - 24, 12)
    base2_b = (body_len / 2 - 24, -12)
    pg.draw.polygon(surf, front_marker, [to_surf(
        tip_b), to_surf(base1_b), to_surf(base2_b)])

    # Draw wheels at body-frame offsets
    for name, (dx, dy) in WHEEL_LOCATIONS.items():
        # Choose color by side
        color = left_wheel_color if 'left' in name else right_wheel_color
        wx, wy = to_surf((dx, dy))

        # Build a small wheel surface aligned with body X, then rotate by steer angle
        base_wheel = pg.Surface(
            (int(wheel_length), int(wheel_width)), pg.SRCALPHA)
        base_rect = base_wheel.get_rect()
        pg.draw.rect(base_wheel, color, base_rect, border_radius=4)
        pg.draw.rect(base_wheel, outline, base_rect, width=1, border_radius=4)

        # Determine steering angle for this wheel (front/rear steerable, middle fixed)
        steer_deg = 0.0
        if name in ("front_left", "front_right", "rear_left", "rear_right"):
            steer_deg = np.degrees(steer_angles.get(name, 0.0))

        wheel_img = pg.transform.rotate(base_wheel, steer_deg)
        wheel_rect = wheel_img.get_rect(center=(wx, wy))
        surf.blit(wheel_img, wheel_rect.topleft)

    # Rotate whole body-frame canvas by heading (CCW positive)
    rotated = pg.transform.rotate(surf, np.degrees(angle))
    rect = rotated.get_rect(center=position)
    screen.blit(rotated, rect.topleft)
    return rect


def draw_icr(screen, pose: Pose, icr_body: np.ndarray):
    """Draw ICR (blue dot) and a faint circle through robot center (dashed)."""
    if icr_body is None:
        return
    R = float(np.hypot(icr_body[0], icr_body[1]))
    theta = pose.theta
    # ICR in world
    icc_x = pose.x + np.cos(theta)*icr_body[0] - np.sin(theta)*icr_body[1]
    icc_y = pose.y + np.sin(theta)*icr_body[0] + np.cos(theta)*icr_body[1]
    cx = int(SCREEN_WIDTH//2 + icc_x)
    cy = int(SCREEN_HEIGHT//2 - icc_y)
    pg.draw.circle(screen, (25, 120, 240), (cx, cy), 4)

    # dashed circle through robot center
    rad = int(abs(R))
    if rad > 4 and rad < 4000:
        segs = max(64, int(2*np.pi*rad/10))
        for i in range(segs):
            if i % 2:   # gaps
                continue
            a0 = 2*np.pi*i/segs
            a1 = 2*np.pi*(i+1)/segs
            x0 = cx + int(rad*np.cos(a0))
            y0 = cy + int(rad*np.sin(a0))
            x1 = cx + int(rad*np.cos(a1))
            y1 = cy + int(rad*np.sin(a1))
            pg.draw.line(screen, (120, 180, 255), (x0, y0), (x1, y1), 1)


def draw_wheel_vectors(screen, pose: Pose, feas: Twist, wheel_angles: Dict[str, float]):
    """Draw per-wheel contact velocity (green) and lateral slip (red)."""
    theta = pose.theta
    # transform function: body (x,y) -> screen

    def to_screen_body(bx, by):
        # world point of wheel
        wx = pose.x + bx*np.cos(theta) - by*np.sin(theta)
        wy = pose.y + bx*np.sin(theta) + by*np.cos(theta)
        return int(SCREEN_WIDTH//2 + wx), int(SCREEN_HEIGHT//2 - wy)

    for name, (x_i, y_i) in WHEEL_LOCATIONS.items():
        # wheel world position
        sx, sy = to_screen_body(x_i, y_i)

        # contact velocity in body at wheel
        vx = feas.linear_x - feas.angular_z * y_i
        vy = feas.linear_y + feas.angular_z * x_i
        # heading
        delta = wheel_angles[name]
        u = np.array([np.cos(delta), np.sin(delta)])
        n = np.array([-u[1], u[0]])
        v = np.array([vx, vy])

        # scale for visibility
        scale = 0.2  # tune to taste
        tip_v = (sx + int(scale*v[0]), sy - int(scale*v[1]))
        tip_lat = (sx + int(scale*(v @ n)*n[0]), sy - int(scale*(v @ n)*n[1]))

        # contact velocity (green)
        draw_arrow(screen, (sx, sy), tip_v, color=(
            40, 160, 80), width=2, head_len=8)
        # lateral slip (red)
        draw_arrow(screen, (sx, sy), tip_lat, color=(
            200, 60, 60), width=2, head_len=8)


def update_steering_from_keys(keys, steer_angles: Dict[str, float], dt: float) -> Dict[str, float]:
    """Update per-wheel steering angles from held keys.

    Positive angle is CCW relative to body X (turning toward +Y). Limits applied.

    Controls:
    - Front Left:  T (+), G (-)
    - Front Right: Y (+), H (-)
    - Rear Left:   V (+), B (-)
    - Rear Right:  N (+), M (-)
    - Center all:  C
    """
    # Copy to avoid in-place mutation if caller reuses dict
    angles = dict(steer_angles)
    delta = STEER_RATE * dt

    # Front Left
    if keys[pg.K_t]:
        angles["front_left"] = angles.get("front_left", 0.0) + delta
    if keys[pg.K_g]:
        angles["front_left"] = angles.get("front_left", 0.0) - delta

    # Front Right
    if keys[pg.K_y]:
        angles["front_right"] = angles.get("front_right", 0.0) + delta
    if keys[pg.K_h]:
        angles["front_right"] = angles.get("front_right", 0.0) - delta

    # Rear Left
    if keys[pg.K_v]:
        angles["rear_left"] = angles.get("rear_left", 0.0) + delta
    if keys[pg.K_b]:
        angles["rear_left"] = angles.get("rear_left", 0.0) - delta

    # Rear Right
    if keys[pg.K_n]:
        angles["rear_right"] = angles.get("rear_right", 0.0) + delta
    if keys[pg.K_m]:
        angles["rear_right"] = angles.get("rear_right", 0.0) - delta

    # Center all
    if keys[pg.K_c]:
        angles["front_left"] = 0.0
        angles["front_right"] = 0.0
        angles["rear_left"] = 0.0
        angles["rear_right"] = 0.0

    # Clamp to limits
    max_rad = np.deg2rad(MAX_STEER_DEG)
    for k in ("front_left", "front_right", "rear_left", "rear_right"):
        angles[k] = float(np.clip(angles.get(k, 0.0), -max_rad, max_rad))

    return angles


def print_controls():
    print(f"Controls:")
    print(f" - W/S: Forward/Back")
    print(f" - A: Turn Left")
    print(f" - D: Turn Right")
    print(f" - C: Center Steering")
    print(f" - K: Hold to Point-Turn Mode (A/D to spin)")
    print(f" - T/G: Front Left Wheel + / -")
    print(f" - Y/H: Front Right Wheel + / -")
    print(f" - V/B: Rear Left Wheel + / -")
    print(f" - N/M: Rear Right Wheel + / -")
    print(f" - ESC: Quit")


def main():
    pg.init()
    screen = pg.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    clock = pg.time.Clock()
    font = pg.font.Font(None, 26)
    robot_twist = Twist()
    robot_pose = Pose()
    # Ackermann curvature (1/pixel); positive = left turn, negative = right
    kappa = 0.0
    running = True
    print_controls()
    while running:
        for event in pg.event.get():
            if event.type == pg.QUIT:
                running = False
            elif event.type == pg.KEYDOWN:
                if event.key == pg.K_ESCAPE:
                    running = False

        # Time step and current key state
        dt = max(clock.tick(60) / 1000.0, 1e-6)
        keys = pg.key.get_pressed()
        point_turn_mode = keys[pg.K_k]
        if point_turn_mode:
            # Point-turn: set wheel angles to tangents and rotate in place with A/D
            # Ignore curvature; no linear motion
            steer_angles = compute_point_turn_wheel_angles()
            omega = 0.0
            if keys[pg.K_a]:
                omega += TURN_IN_PLACE_RATE
            if keys[pg.K_d]:
                omega -= TURN_IN_PLACE_RATE
            robot_twist = Twist(0.0, 0.0, omega)
            robot_pose = update_pose_from_twist(robot_pose, robot_twist, dt)
        else:
            # Update curvature from steering keys (A: left, D: right, C: center)
            if keys[pg.K_a]:
                kappa += KAPPA_RATE * dt
            if keys[pg.K_d]:
                kappa -= KAPPA_RATE * dt
            if keys[pg.K_c]:
                kappa = 0.0
            kappa = clamp_kappa_to_limits(kappa)

            # Compute commanded forward velocity (vy=0) and yaw from curvature
            robot_twist = compute_twist_from_keys(keys)
            robot_twist.linear_y = 0.0
            robot_twist.angular_z = robot_twist.linear_x * kappa
            robot_pose = update_pose_from_twist(robot_pose, robot_twist, dt)
            steer_angles = compute_ackermann_wheel_angles(kappa)

        # Build full angle set including middle wheels
        full_angles = build_full_wheel_angles(steer_angles)

        # Use the ICR-based helper (projects vy->0 internally)
        icr_body, _, _, feas_twist, slip = twist_to_wheels_icr(robot_twist)
        # Compute wheel speeds from current commanded twist and angles
        wheel_speeds = compute_wheel_speeds_from_twist(
            robot_twist, full_angles)
        # Forward kinematics estimation from wheel speeds/angles
        fk_twist = rocker_bogie_forward_kinematics(wheel_speeds, full_angles)

        screen.fill((255, 255, 255))
        # Offset so (0,0) is in the center of the screen
        # Visualize turning radius beneath the robot (only relevant in Ackermann mode)
        if not point_turn_mode:
            draw_turning_radius(screen, robot_pose, kappa)
        rect = draw_robot(
            screen,
            (SCREEN_WIDTH // 2 + robot_pose.x, SCREEN_HEIGHT // 2 - robot_pose.y),
            robot_pose.theta,
            steer_angles,
        )
        draw_icr(screen, robot_pose, icr_body)
        draw_wheel_vectors(screen, robot_pose, feas_twist, full_angles)
        # Visualize FK-estimated body velocity at robot center
        draw_body_velocity(screen, robot_pose, fk_twist)
        # Display steering angles above the robot
        fl = np.degrees(steer_angles.get("front_left", 0.0))
        fr = np.degrees(steer_angles.get("front_right", 0.0))
        rl = np.degrees(steer_angles.get("rear_left", 0.0))
        rr = np.degrees(steer_angles.get("rear_right", 0.0))
        angle_text = f"FL: {fl:+.1f}°  FR: {fr:+.1f}°  RL: {rl:+.1f}°  RR: {rr:+.1f}°  (limit ±{MAX_STEER_DEG:.0f}°)"
        kappa_text = f"kappa: {kappa:+.5f}  R: {'∞' if abs(kappa) < 1e-6 else f'{(1.0/abs(kappa)):.1f}'} px"
        text_surf1 = font.render(angle_text, True, (0, 0, 0))
        text_rect1 = text_surf1.get_rect(
            midbottom=(rect.centerx, rect.top - 8))
        screen.blit(text_surf1, text_rect1)
        text_surf2 = font.render(kappa_text, True, (0, 0, 0))
        text_rect2 = text_surf2.get_rect(
            midbottom=(rect.centerx, text_rect1.top - 4))
        screen.blit(text_surf2, text_rect2) if abs(kappa) > 1e-12 else None
        pg.display.flip()
        # clock.tick is already used above for dt

    pg.quit()


if __name__ == "__main__":
    plot_steering_geometry(Twist(linear_x=200.0, linear_y=50.0,
                           angular_z=0.8), "Arbitrary Twist (vy projected to 0)")
    main()
