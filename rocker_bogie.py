# Vector-Based Least Squares method for
# determining body frame velocities from wheel speeds
# for a 6-wheeled rocker-bogie robot (e.g., Mars rovers).
import numpy as np
import matplotlib.pyplot as plt

# Constants
WHEEL_RADIUS = 50  # The wheel radius [mm]
WHEEL_BASE = 250  # Wheel to Wheel distance along the Robot Y-axis [mm]
# The back to front wheel distance along the Robot X-axis [mm]
TRACK_WIDTH = 350


# For any wheel i at position (xi, yi)
# its velocity vector (v_ix, v_iy) in the robot frame
# is a result of the chassis's linear velocity (v_bx, v_by)
# plus the rotational velocity ω_b around the center.
# vix​ = vbx​−ωb​⋅yi​
# viy​ = vby​+ωb​⋅xi


# Wheel locations: WHEEL_BASE is Y-axis (left-right), TRACK_WIDTH is X-axis (front-back)
WHEEL_LOCATIONS = {
    "front_left": (TRACK_WIDTH / 2, WHEEL_BASE / 2),      # Front left
    "middle_left": (0, WHEEL_BASE / 2),                   # Middle left
    "rear_left": (-TRACK_WIDTH / 2, WHEEL_BASE / 2),      # Rear left
    "front_right": (TRACK_WIDTH / 2, -WHEEL_BASE / 2),    # Front right
    "middle_right": (0, -WHEEL_BASE / 2),                 # Middle right
    "rear_right": (-TRACK_WIDTH / 2, -WHEEL_BASE / 2)     # Rear right
}

# We also know what the wheel's velocity vector should be
# based on its own drive speed v_i and steering angle δ_i:
# vix​ = vi​cos(δi​)
# viy​ = vi​sin(δi​)

# Build a linear system of equations: A * x = b


def calculate_body_velocities(wheel_drive_speeds, wheel_steer_angles):
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
        return np.zeros(3)

    # Return as a 1D array [v_bx, v_by, omega_b]
    return body_vel_solution.flatten()


if __name__ == '__main__':
    # 2. Define a motion scenario
    print("--- Scenario 1: Driving Straight Forward ---")
    # All wheels drive forward at the same speed with no steering.
    speeds_straight = {'front_left': 10, 'front_right': 10,
                       'middle_left': 10, 'middle_right': 10, 'rear_left': 10, 'rear_right': 10}
    angles_straight = {'front_left': 0, 'front_right': 0,
                       'middle_left': 0, 'middle_right': 0, 'rear_left': 0, 'rear_right': 0}

    body_vel_straight = calculate_body_velocities(
        speeds_straight, angles_straight
    )
    print(
        f"Calculated Body Velocity: [v_bx, v_by, omega_b] = {np.round(body_vel_straight, 2)}")
    # Assert the expected values with some tolerance for floating point precision
    expected_x_vel = 10.0  # All wheels drive forward at 10 mm/s
    assert np.allclose(body_vel_straight, [
                       expected_x_vel, 0.0, 0.0], atol=1e-6), f"Expected [10.0, 0.0, 0.0], got {body_vel_straight}"
    print("✓ Test passed: Expected [10.0, 0.0, 0.0]\n")

    print("--- Scenario 2: Point Turn (Spinning in Place) ---")
    # Left wheels drive backward, right wheels drive forward.
    # Front and rear wheels are steered inward to facilitate the turn.
    turn_speed = 5
    steer_angle = np.deg2rad(45)
    speeds_turn = {'front_left': -turn_speed, 'front_right': turn_speed, 'middle_left': -
                   turn_speed, 'middle_right': turn_speed, 'rear_left': -turn_speed, 'rear_right': turn_speed}
    angles_turn = {'front_left': -steer_angle, 'front_right': steer_angle,
                   'middle_left': 0, 'middle_right': 0, 'rear_left': steer_angle, 'rear_right': -steer_angle}

    body_vel_turn = calculate_body_velocities(
        speeds_turn, angles_turn)
    print(
        f"Calculated Body Velocity: [v_bx, v_by, omega_b] = {np.round(body_vel_turn, 2)}")
    # Assert that we get approximately zero translation and some positive rotation
    assert np.allclose(body_vel_turn[:2], [
                       0.0, 0.0], atol=1e-6), f"Expected translation [0.0, 0.0], got {body_vel_turn[:2]}"
    assert body_vel_turn[2] > 0, f"Expected positive angular velocity, got {body_vel_turn[2]}"
    print("✓ Test passed: Expected [0.0, 0.0, positive_value]\n")
