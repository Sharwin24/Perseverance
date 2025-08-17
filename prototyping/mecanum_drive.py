import numpy as np

# Constants
WHEEL_RADIUS = 50  # [mm]
WHEEL_BASE = 1000  # Robot center to wheel center along Robot X-axis[mm]
TRACK_WIDTH = 800  # Robot center to wheel center along Robot Y-axis[mm]


def H0() -> np.array:
    R = WHEEL_RADIUS
    L = TRACK_WIDTH / 2.0
    W = WHEEL_BASE / 2.0
    return (1 / R) * np.array([
        [-L-W, 1, -1],
        [L+W, 1, 1],
        [L+W, 1, -1],
        [-L-W, 1, 1]
    ])


def F() -> np.array:
    """F = pinv(H0)"""
    R = WHEEL_RADIUS
    L = TRACK_WIDTH / 2.0
    W = WHEEL_BASE / 2.0
    return (R / 4) * np.array([
        [-1 / (L + W), 1/(L + W), 1/(L + W), -1/(L + W)],
        [1, 1, 1, 1],
        [-1, 1, -1, 1]
    ])


def forward_kinematics(wheel_speeds: list[float]) -> tuple[float, float, float]:
    """ Given wheel speeds, calculate the robot's body-frame linear velocities and angular velocity.

    Args:
        wheel_speeds (list[float]): List of wheel speeds [w_fl, w_fr, w_rl, w_rr] in rad/s

    Raises:
        ValueError: If the input list is not of length 4.

    Returns:
        tuple[float, float, float]: The linear velocities (vx, vy) and angular velocity (omega) in the robot's body frame.
    """
    if len(wheel_speeds) != 4:
        raise ValueError("Wheel speeds must be a list of 4 elements.")
    w_fl, w_fr, w_rl, w_rr = wheel_speeds
    V = F() @ np.array([w_fl, w_fr, w_rl, w_rr])
    assert len(V) == 3, "Velocity vector should have 3 components."
    print(f"Calculated velocities with shape {V.shape}:\n{V}")
    return (
        V[0],  # vx
        V[1],  # vy
        V[2]   # omega
    )


def inverse_kinematics(vx: float, vy: float, omega: float) -> list[float]:
    """ Given robot's body-frame velocities, calculate the wheel speeds.

    Args:
        vx (float): Linear velocity in the x direction (m/s).
        vy (float): Linear velocity in the y direction (m/s).
        omega (float): Angular velocity (rad/s).

    Returns:
        list[float]: List of wheel speeds [w_fl, w_fr, w_rl, w_rr] in rad/s.
    """
    V = np.array([vx, vy, omega])
    wheel_speeds = H0() @ V
    return wheel_speeds.tolist()


def relative_to_global(robot_velocity: tuple[float, float, float], heading: float) -> tuple[float, float]:
    """ Convert robot's body-frame velocities to global frame.

    Args:
        robot_velocity (tuple[float, float, float]): Robot's body-frame velocities (vx, vy, omega).
        heading (float): Robot's heading angle in radians.

    Returns:
        tuple[float, float]: Global frame velocities (vx_global, vy_global).
    """
    vx, vy, omega = robot_velocity
    vx_global = vx * np.cos(heading) - vy * np.sin(heading)
    vy_global = vx * np.sin(heading) + vy * np.cos(heading)
    return vx_global, vy_global


if __name__ == "__main__":
    wheel_speeds = [-30, 30, -30, 30]  # Example wheel speeds
    vx, vy, omega = forward_kinematics(wheel_speeds)
    print(
        f"Mecanum Wheel FK: [{wheel_speeds[0]:.2f}, {wheel_speeds[1]:.2f}, {wheel_speeds[2]:.2f}, {wheel_speeds[3]:.2f}]"
        f" -> [{vx:.2f}, {vy:.2f}, {omega:.2f}] (vx, vy, omega)"
    )

    # Verify FK <-> IK
    wheel_speeds_ik = inverse_kinematics(vx, vy, omega)
    print(
        f"Mecanum Wheel IK: [{vx:.2f}, {vy:.2f}, {omega:.2f}] (vx, vy, omega)"
        f" -> [{wheel_speeds_ik[0]:.2f}, {wheel_speeds_ik[1]:.2f}, {wheel_speeds_ik[2]:.2f}, {wheel_speeds_ik[3]:.2f}]"
        f" (w_fl, w_fr, w_rl, w_rr)"
    )

    assert np.allclose(
        wheel_speeds,
        wheel_speeds_ik,
        atol=1e-5
    ), "Inverse kinematics did not return the original wheel speeds."
