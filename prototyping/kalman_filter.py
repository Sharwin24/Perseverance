from rover_constants import (
    WHEEL_RADIUS,
    WHEEL_BASE,            # Longitudinal front↔rear distance (A↔C)
    TRACK_WIDTH_MIDDLE,    # Lateral middle wheel spacing
    TRACK_WIDTH_STEERING,  # Lateral steering wheel spacing
    WHEEL_LOCATIONS,
)
"""Kalman filter prototype for rover state estimation.

Geometry Naming (migrated from legacy inversion):
    WHEEL_BASE: longitudinal front↔rear (previously named TRACK_WIDTH).
    TRACK_WIDTH_MIDDLE: lateral middle pair spacing (previously WHEEL_BASE).
    TRACK_WIDTH_STEERING: lateral steering pair spacing (previously STEERING_WHEEL_BASE).

Mecanum Kinematics Note:
    Typical formulation uses half-length L = WHEEL_BASE / 2 and half-width W = TRACK_WIDTH_MIDDLE / 2.
    Legacy code previously swapped these; updated here.
"""
import numpy as np
import matplotlib.pyplot as plt
import os
from draw_rocker_bogie import draw_rocker_bogie_at_state

# Create plots directory if it doesn't exist
PLOTS_DIR = "plots"
if not os.path.exists(PLOTS_DIR):
    os.makedirs(PLOTS_DIR)


class RobotState:
    def __init__(self, x, y, theta, vx, vy, omega):
        self.x = x  # Position in x direction [mm]
        self.y = y  # Position in y direction [mm]
        self.theta = theta  # Orientation in radians
        self.vx = vx  # Linear velocity in x direction [mm/s]
        self.vy = vy  # Linear velocity in y direction [mm/s]
        self.omega = omega  # Angular velocity [rad/s]

    def to_array(self):
        # Convert the state to a numpy array for easier manipulation
        return np.array([self.x, self.y, self.theta, self.vx, self.vy, self.omega])

    @classmethod
    def from_npy(cls, arr):
        # Create a RobotState instance from a numpy array
        if len(arr) != 6:
            raise ValueError("Array must have exactly 6 elements.")
        return cls(arr[0], arr[1], arr[2], arr[3], arr[4], arr[5])


class OdomData:
    def __init__(self, x, y, theta, timestamp):
        self.odom_x: float = x  # Odometer x position [mm]
        self.odom_y: float = y  # Odometer y position [mm]
        self.odom_theta: float = theta  # Odometer orientation [radians]
        # Timestamp of the odometry data [seconds]
        self.timestamp: float = timestamp


class IMUData:
    def __init__(self, acc_x, acc_y, gyro_z, timestamp):
        # Acceleration in robot's x direction [mm/s^2]
        self.acc_x: float = acc_x
        # Acceleration in robot's y direction [mm/s^2]
        self.acc_y: float = acc_y
        self.gyro_z: float = gyro_z  # Angular velocity around z axis [rad/s]
        # Timestamp of the IMU data [seconds]
        self.timestamp: float = timestamp


class KalmanFilter:
    def __init__(self, initial_state: RobotState, initial_timestamp: float = 0.0):
        # State vector: [x, y, theta, vx, vy, omega]
        self.state = initial_state  # Initial state of the robot
        self.P = np.eye(6)  # Initial covariance matrix
        self.P[0, 0] = 0.025  # Position uncertainty in x [mm^2]
        self.P[1, 1] = 0.025  # Position uncertainty in y [mm^2]
        self.P[2, 2] = 0.01  # Orientation uncertainty [rad^2]
        self.P[3, 3] = 0.1  # Velocity uncertainty in x [mm^2/s^2]
        self.P[4, 4] = 0.1  # Velocity uncertainty in y [mm^2/s^2]
        self.P[5, 5] = 0.0825  # Angular velocity uncertainty [rad^2/s^2]
        self.Q = np.eye(6)  # Process noise covariance
        self.Q[0, 0] = 0.01    # X position noise [mm^2]
        self.Q[1, 1] = 0.01    # Y position noise [mm^2]
        self.Q[2, 2] = 0.005   # Orientation noise [rad^2]
        self.Q[3, 3] = 0.075     # Velocity noise in x [mm^2/s^2]
        self.Q[4, 4] = 0.075     # Velocity noise in y [mm^2/s^2]
        self.Q[5, 5] = 0.005    # Angular velocity noise [rad^2/s^2]
        # [x, y, theta] Odometry measurement noise covariance
        self.R_odom = np.eye(3)
        self.R_odom[0, 0] = 10.0  # Position uncertainty in x [mm^2]
        self.R_odom[1, 1] = 10.0  # Position uncertainty in y [mm^2]
        self.R_odom[2, 2] = 0.2  # Orientation uncertainty [rad^2]
        # [omega] IMU measurement noise covariance
        self.R_imu = np.array([[0.0075]])  # Yaw rate noise [rad^2/s^2]
        self.last_timestamp = initial_timestamp  # [seconds]
        print("Kalman Filter initialized with state:", self.state.to_array())
        print(f"Initial covariance matrix:\n{np.round(self.P, 2)}\n")
        print(f"Process noise covariance matrix:\n{np.round(self.Q, 2)}\n")
        print(
            f"Measurement noise covariance matrix (Odometry):\n{np.round(self.R_odom, 2)}\n")
        print(
            f"Measurement noise covariance matrix (IMU):\n{np.round(self.R_imu, 2)}\n")

    def normalize_angle(self, angle: float) -> float:
        """Normalizes an angle to be within -pi to pi."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def F(self, dt):
        # State Transition Matrix (F) for a constant acceleration model
        # Each row is a partial derivative of the new state with respect to the old state
        # [dX/dx, dX/dy, dX/dtheta, dX/dvx, dX/dvy, dX/domega]
        # [dY/dx, dY/dy, dY/dtheta, dY/dvx, dY/dvy, dY/domega]
        # [dTheta/dx, dTheta/dy, dTheta/dtheta, dTheta/dvx, dTheta/dvy, dTheta/domega]
        # [dVx/dx, dVx/dy, dVx/dtheta, dVx/dvx, dVx/dvy, dVx/domega]
        # [dVy/dx, dVy/dy, dVy/dtheta, dVy/dvx, dVy/dvy, dVy/domega]
        # [dOmega/dx, dOmega/dy, dOmega/dtheta, dOmega/dvx, dOmega/dvy, dOmega/domega]
        return np.array([
            [1, 0, 0, dt, 0,  0],
            [0, 1, 0, 0,  dt, 0],
            [0, 0, 1, 0,  0,  dt],
            [0, 0, 0, 1,  0,  0],
            [0, 0, 0, 0,  1,  0],
            [0, 0, 0, 0,  0,  1]
        ])

    def B(self, dt):
        # Control Input Model (B) for a constant acceleration model
        # Each row is a partial derivative of the new state with respect to the control input
        # [dX/dax, dX/day]
        # [dY/dax, dY/day]
        # [dTheta/dax, dTheta/day]
        # [dVx/dax, dVx/day]
        # [dVy/dax, dVy/day]
        # [dOmega/dax, dOmega/day]
        return np.array([
            [0.5 * dt**2, 0],
            [0, 0.5 * dt**2],
            [0, 0],
            [dt, 0],
            [0, dt],
            [0, 0]
        ])

    def mecanum_forward_kinematics(self) -> np.array:
        """F = pinv(H0)"""
        R = WHEEL_RADIUS
        L = WHEEL_BASE / 2.0          # Half-length
        W = TRACK_WIDTH_MIDDLE / 2.0  # Half-width
        return (R / 4) * np.array([
            [-1 / (L + W), 1/(L + W), 1/(L + W), -1/(L + W)],
            [1, 1, 1, 1],
            [-1, 1, -1, 1]
        ])

    def mecanum_inverse_kinematics(self, vx: float, vy: float, omega: float) -> np.array:
        """Calculate wheel velocities from robot body velocities
        Returns: [w_fl, w_fr, w_rl, w_rr] (front left, front right, rear left, rear right)
        """
        R = WHEEL_RADIUS
        L = WHEEL_BASE / 2.0          # Half-length
        W = TRACK_WIDTH_MIDDLE / 2.0  # Half-width
        return (1 / R) * np.array([
            (-L - W) * vx + vy + (-1) * omega,  # Front left wheel
            (L + W) * vx + vy + (1) * omega,    # Front right wheel
            (L + W) * vx + vy + (-1) * omega,   # Rear left wheel
            (-L - W) * vx + vy + (1) * omega    # Rear right wheel
        ])

    def rocker_bogie_forward_kinematics(self, wheel_drive_speeds, wheel_steer_angles):
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

    def predict_with_dynamics_model(self, imu_data: IMUData):
        """
        Predicts the next state using a constant acceleration model.
        IMU accelerations are treated as control inputs.
        """
        dt = imu_data.timestamp - self.last_timestamp
        if dt <= 0:
            print("Warning: Non-positive time difference in dynamics prediction.")
            return

        theta = self.state.theta
        # Convert robot-frame acceleration to global-frame acceleration
        global_acc_x = imu_data.acc_x * \
            np.cos(theta) - imu_data.acc_y * np.sin(theta)
        global_acc_y = imu_data.acc_x * \
            np.sin(theta) + imu_data.acc_y * np.cos(theta)

        # State Transition Matrix (F) for a constant acceleration model
        F = self.F(dt)

        # Control Input Model (B)
        B = self.B(dt)

        # Control Vector (u)
        u = np.array([global_acc_x, global_acc_y])

        # Predict state: x_pred = F * x + B * u
        predicted_state_arr = F @ self.state.to_array() + B @ u
        self.state = RobotState.from_npy(predicted_state_arr)
        self.state.theta = self.normalize_angle(self.state.theta)

        # Predict covariance: P_pred = F * P * F.T + Q
        self.P = F @ self.P @ F.T + self.Q
        # Update the last timestamp
        self.last_timestamp = imu_data.timestamp

    def predict_with_diff_drive_kinematics_model(self, left_vel: float, right_vel: float, timestamp: float):
        """
        Predicts the next state using a differential drive kinematics model.
        left_vel and right_vel are wheel velocities in mm/s.
        """
        dt = timestamp - self.last_timestamp
        if dt <= 0:
            print("Warning: Non-positive time difference in kinematics prediction.")
            return

        # --- 1. Calculate kinematic inputs V (forward) and W (angular) ---
        V = (left_vel + right_vel) / 2.0
        W = (right_vel - left_vel) / WHEEL_BASE

        # --- 2. Define the non-linear state transition function ---
        x, y, theta, _, _, _ = self.state.to_array()  # Unpack old state

        # Predict new state based on kinematics
        x_new = x + V * np.cos(theta) * dt
        y_new = y + V * np.sin(theta) * dt
        theta_new = self.normalize_angle(theta + W * dt)
        # New velocity is based on new orientation
        vx_new = V * np.cos(theta_new)
        vy_new = V * np.sin(theta_new)
        omega_new = W
        self.state = RobotState(x_new, y_new, theta_new,
                                vx_new, vy_new, omega_new)

        # --- 3. Calculate the Jacobian (F) ---
        # F is the partial derivative of the new state w.r.t the old state
        F = np.zeros((6, 6))
        F[0, 0] = 1.0  # dx'/dx
        F[0, 2] = V * -np.sin(theta_new) * dt  # dx'/dtheta
        F[1, 1] = 1.0  # dy'/dy
        F[1, 2] = V * np.cos(theta_new) * dt  # dy'/dtheta
        F[2, 2] = 1.0  # dtheta'/dtheta
        # Velocities are determined by inputs, not previous velocities,
        # but their orientation depends on the old theta.
        # d(vx')/dtheta via chain rule
        F[3, 2] = V * -np.sin(theta_new) * (W*dt)
        # d(vy')/dtheta via chain rule
        F[4, 2] = V * np.cos(theta_new) * (W*dt)
        # Omega is determined entirely by input W.

        # --- 4. Update the covariance ---
        self.P = F @ self.P @ F.T + self.Q
        self.last_timestamp = timestamp

    def predict_with_mecanum_kinematics_model(self, wheel_vels: list[float], timestamp: float):
        dt = timestamp - self.last_timestamp
        if dt <= 0:
            print(
                "Warning: Non-positive time difference in mecanum kinematics prediction.")
            return

        # --- 1. Calculate the robot body velocities ---
        robot_body_velocity = self.mecanum_forward_kinematics() @ np.array(wheel_vels)
        vx, vy, omega = robot_body_velocity[0], robot_body_velocity[1], robot_body_velocity[2]
        # Convert to global frame
        heading = self.state.theta
        vx_global = vx * np.cos(heading) - vy * np.sin(heading)
        vy_global = vx * np.sin(heading) + vy * np.cos(heading)

        # --- 2. Predict new state ---
        x_new = self.state.x + vx_global * dt
        y_new = self.state.y + vy_global * dt
        theta_new = self.normalize_angle(self.state.theta + omega * dt)
        # Update state with new values
        self.state = RobotState(x_new, y_new, theta_new,
                                vx_global, vy_global, omega)

        # --- 3. Calculate the Jacobian ---
        # The Jacobian F must be re-derived for this new state transition.
        # For example, dx'/dtheta = (-vx*sin(heading) - vy*cos(heading)) * dt
        # Note: This is a simplified Jacobian. A full derivation is more involved.
        F = np.eye(6)
        F[0, 2] = (-vx * np.sin(heading) - vy *
                   np.cos(heading)) * dt  # dx'/dtheta
        F[1, 2] = (vx * np.cos(heading) - vy *
                   np.sin(heading)) * dt  # dy'/dtheta
        F[0, 3] = dt  # dx'/dvx
        F[1, 4] = dt  # dy'/dvy
        F[2, 5] = dt  # dtheta'/domega

        # --- 4. Update the covariance ---
        self.P = F @ self.P @ F.T + self.Q
        self.last_timestamp = timestamp

    def predict_with_rocker_bogie_kinematics_model(
            self,
            wheel_drive_speeds: dict[str, float],
            wheel_steer_angles: dict[str, float],
            timestamp: float):
        """Predict the state using the rocker-bogie kinematics model.

        Wheel names: ["front_left", "middle_left", "rear_left",
                      "front_right", "middle_right", "rear_right"]

        Args:
            wheel_drive_speeds (dict[str, float]): The drive speeds for each wheel using their names as the keys.
            wheel_steer_angles (dict[str, float]): The steer angles for each wheel.
            timestamp (float): The current timestamp.
        """
        dt = timestamp - self.last_timestamp
        if dt <= 0:
            print(
                "Warning: Non-positive time difference in mecanum kinematics prediction.")
            return

        # --- 1. Calculate the wheel velocities in the robot frame ---
        robot_velocity = self.rocker_bogie_forward_kinematics(
            wheel_drive_speeds, wheel_steer_angles
        )
        vx, vy, omega = robot_velocity[0], robot_velocity[1], robot_velocity[2]
        # Convert to global frame
        heading = self.state.theta
        sin_heading = np.sin(heading)
        cos_heading = np.cos(heading)
        vx_global = vx * cos_heading - vy * sin_heading
        vy_global = vx * sin_heading + vy * cos_heading

        # --- 2. Predict new state ---
        x_new = self.state.x + vx_global * dt
        y_new = self.state.y + vy_global * dt
        theta_new = self.normalize_angle(self.state.theta + omega * dt)

        # Update state with new values. The new velocities are the global velocities.
        self.state = RobotState(x_new, y_new, theta_new,
                                vx_global, vy_global, omega)

        # --- 3. Calculate Jacobian ---
        F = np.eye(6)
        # Partial derivative of new position w.r.t old orientation (theta)
        # This comes from the chain rule on the coordinate transformation.
        # d(x')/d(theta) = (-vx*sin(h) - vy*cos(h)) * dt
        F[0, 2] = (-vx * sin_heading - vy * cos_heading) * dt
        # d(y')/d(theta) = (vx*cos(h) - vy*sin(h)) * dt
        F[1, 2] = (vx * cos_heading - vy * sin_heading) * dt

        # Partial derivative of new velocity w.r.t old orientation (theta)
        # d(vx_global)/d(theta) = -vx*sin(h) - vy*cos(h)
        F[3, 2] = -vx * sin_heading - vy * cos_heading
        # d(vy_global)/d(theta) = vx*cos(h) - vy*sin(h)
        F[4, 2] = vx * cos_heading - vy * sin_heading

        # In this kinematic model, the new velocities and angular rate depend only
        # on the control inputs (wheel speeds), not the previous velocities.
        # Therefore, the derivatives of the new velocities w.r.t old velocities are 0.
        F[3, 3] = F[3, 4] = F[3, 5] = 0.0
        F[4, 3] = F[4, 4] = F[4, 5] = 0.0
        F[5, 3] = F[5, 4] = F[5, 5] = 0.0

        # --- 4. Update the covariance ---
        self.P = F @ self.P @ F.T + self.Q
        self.last_timestamp = timestamp

    def update_odom(self, odom_data: OdomData):
        z = np.array(
            [odom_data.odom_x, odom_data.odom_y, odom_data.odom_theta])
        # Odometry Measurement Model
        H = np.array([
            [1, 0, 0, 0, 0, 0],  # [x]
            [0, 1, 0, 0, 0, 0],  # [y]
            [0, 0, 1, 0, 0, 0]  # [theta]
        ])
        # Calculate the innovation and normalize theta
        y = z - H @ self.state.to_array()
        y[2] = self.normalize_angle(y[2])
        # Calculate the Kalman Gain [K] using the innovation covariance [S]
        S = H @ self.P @ H.T + self.R_odom
        K = self.P @ H.T @ np.linalg.inv(S)
        # Update the state estimate and covariance
        self.state = RobotState.from_npy(self.state.to_array() + K @ y)
        self.state.theta = self.normalize_angle(self.state.theta)
        self.P = (np.eye(self.P.shape[0]) - K @ H) @ self.P

    def update_imu(self, imu_data: IMUData):
        z = np.array([imu_data.gyro_z])
        # IMU Measurement Model
        H = np.array([
            [0, 0, 0, 0, 0, 1]  # [omega]
        ])
        # Calculate the innovation
        y = z - H @ self.state.to_array()
        # Calculate the Kalman Gain [K] using the innovation covariance [S]
        S = H @ self.P @ H.T + self.R_imu
        K = self.P @ H.T @ np.linalg.inv(S)
        # Update the state estimate and covariance
        self.state = RobotState.from_npy(self.state.to_array() + K @ y)
        self.state.theta = self.normalize_angle(self.state.theta)
        self.P = (np.eye(self.P.shape[0]) - K @ H) @ self.P


if __name__ == "__main__":
    initial_timestamp = 0.0
    initial_state = RobotState(
        x=0, y=0, theta=np.pi/4, vx=100.0, vy=0, omega=0.2)
    V = 100.0
    initial_state.vx = V * np.cos(initial_state.theta)
    initial_state.vy = V * np.sin(initial_state.theta)

    kf_dynamic_model = KalmanFilter(initial_state, initial_timestamp)
    kf_kinematic_model = KalmanFilter(initial_state, initial_timestamp)
    kf_mecanum_model = KalmanFilter(initial_state, initial_timestamp)
    kf_rocker_bogie_model = KalmanFilter(initial_state, initial_timestamp)

    timestamps = [initial_timestamp]
    estimated_states_dynamic = [kf_dynamic_model.state.to_array().copy()]
    estimated_states_kinematic = [kf_kinematic_model.state.to_array().copy()]
    estimated_states_mecanum = [kf_mecanum_model.state.to_array().copy()]
    estimated_states_rocker_bogie = [
        kf_rocker_bogie_model.state.to_array().copy()]
    true_states = [initial_state.to_array().copy()]
    odom_readings_dynamic = []
    odom_readings_kinematic = []
    odom_readings_mecanum = []
    odom_readings_rocker_bogie = []
    robot_state_history: dict[str, list[RobotState]] = {
        "dynamic": [], "kinematic": [], "mecanum": [], "rocker_bogie": []
    }

    total_time = 10.0
    dt = 0.1
    num_steps = int(total_time / dt)

    for i in range(1, num_steps + 1):
        timestamp = i * dt
        last_true_state = true_states[-1]
        total_distance = 2000.0
        progress = timestamp / total_time
        v_0 = 20.0
        v_1 = 0.0
        A = 2 * total_distance - v_0 - v_1
        B = -3 * total_distance + 2 * v_0 + v_1
        C = v_0
        D = 0.0
        true_x_new = A * (progress**3) + B * (progress**2) + C * progress + D
        true_y_new = total_distance * progress
        dt_dprogress = dt / total_time
        dx_dt = (3*A*(progress**2) + 2*B*progress + C) * (1/total_time)
        dy_dt = total_distance / total_time
        true_vx_new = dx_dt
        true_vy_new = dy_dt
        actual_speed = np.sqrt(true_vx_new**2 + true_vy_new**2)
        true_theta_new = np.arctan2(true_vy_new, true_vx_new)

        if len(true_states) > 0:
            last_theta = last_true_state[2]
            theta_change = true_theta_new - last_theta
            while theta_change > np.pi:
                theta_change -= 2*np.pi
            while theta_change < -np.pi:
                theta_change += 2*np.pi
            true_omega = theta_change / dt
        else:
            true_omega = 0.0

        if len(true_states) > 0:
            last_speed = np.sqrt(last_true_state[3]**2 + last_true_state[4]**2)
            speed_change = actual_speed - last_speed
            true_acc_x = speed_change / dt
            true_acc_y = actual_speed * abs(true_omega)
        else:
            true_acc_x, true_acc_y = 0.0, 0.0

        current_true_state = np.array(
            [true_x_new, true_y_new, true_theta_new, true_vx_new, true_vy_new, true_omega])
        true_states.append(current_true_state)

        predict_imu = IMUData(
            acc_x=true_acc_x, acc_y=true_acc_y, gyro_z=0, timestamp=timestamp)
        kf_dynamic_model.predict_with_dynamics_model(predict_imu)

        robot_speed = np.sqrt(true_vx_new**2 + true_vy_new**2)
        kf_kinematic_model.predict_with_diff_drive_kinematics_model(
            left_vel=robot_speed - (true_omega * WHEEL_BASE / 2),
            right_vel=robot_speed + (true_omega * WHEEL_BASE / 2),
            timestamp=timestamp
        )
        # Rocker-bogie uses wheel speeds and steering angles for the 2 front and 2 rear wheels
        # Generate Rocker-Bogie wheel inputs from true motion
        body_vx = true_vx_new * \
            np.cos(last_theta) + true_vy_new * np.sin(last_theta)
        turn_radius = np.inf if abs(
            true_omega) < 1e-6 else body_vx / true_omega
        # Ackermann-like approximation: tan(delta) ≈ wheelbase / turn_radius
        steer_angle = 0 if not np.isfinite(turn_radius) else np.arctan(
            WHEEL_BASE / (2 * turn_radius))

        drive_speed = body_vx
        wheel_drive_speeds = {
            name: drive_speed for name in WHEEL_LOCATIONS.keys()}
        wheel_steer_angles = {
            "front_left": steer_angle, "middle_left": 0, "rear_left": -steer_angle,
            "front_right": steer_angle, "middle_right": 0, "rear_right": -steer_angle
        }

        kf_rocker_bogie_model.predict_with_rocker_bogie_kinematics_model(
            wheel_drive_speeds,
            wheel_steer_angles,
            timestamp
        )

        robot_heading = true_theta_new
        body_vx = true_vx_new * \
            np.cos(robot_heading) + true_vy_new * np.sin(robot_heading)
        body_vy = -true_vx_new * \
            np.sin(robot_heading) + true_vy_new * np.cos(robot_heading)
        wheel_velocities = kf_mecanum_model.mecanum_inverse_kinematics(
            body_vx, body_vy, true_omega)
        kf_mecanum_model.predict_with_mecanum_kinematics_model(
            wheel_vels=wheel_velocities.tolist(), timestamp=timestamp)

        # Correction Step
        odom_noise_dynamic = np.random.multivariate_normal(
            np.zeros(3), kf_dynamic_model.R_odom)
        odom_noise_kinematic = np.random.multivariate_normal(
            np.zeros(3), kf_kinematic_model.R_odom)
        odom_noise_mecanum = np.random.multivariate_normal(
            np.zeros(3), kf_mecanum_model.R_odom)
        odom_noise_rocker_bogie = np.random.multivariate_normal(
            np.zeros(3), kf_rocker_bogie_model.R_odom)

        odom_data_dynamic = OdomData(x=true_x_new + odom_noise_dynamic[0], y=true_y_new + odom_noise_dynamic[1],
                                     theta=kf_dynamic_model.normalize_angle(true_theta_new + odom_noise_dynamic[2]), timestamp=timestamp)
        odom_data_kinematic = OdomData(x=true_x_new + odom_noise_kinematic[0], y=true_y_new + odom_noise_kinematic[1],
                                       theta=kf_kinematic_model.normalize_angle(true_theta_new + odom_noise_kinematic[2]), timestamp=timestamp)
        odom_data_mecanum = OdomData(x=true_x_new + odom_noise_mecanum[0], y=true_y_new + odom_noise_mecanum[1],
                                     theta=kf_mecanum_model.normalize_angle(true_theta_new + odom_noise_mecanum[2]), timestamp=timestamp)
        odom_data_rocker_bogie = OdomData(x=true_x_new + odom_noise_rocker_bogie[0], y=true_y_new + odom_noise_rocker_bogie[1],
                                          theta=kf_rocker_bogie_model.normalize_angle(true_theta_new + odom_noise_rocker_bogie[2]), timestamp=timestamp)

        kf_dynamic_model.update_odom(odom_data_dynamic)
        kf_kinematic_model.update_odom(odom_data_kinematic)
        kf_mecanum_model.update_odom(odom_data_mecanum)
        kf_rocker_bogie_model.update_odom(odom_data_rocker_bogie)

        odom_readings_dynamic.append(
            [odom_data_dynamic.odom_x, odom_data_dynamic.odom_y])
        odom_readings_kinematic.append(
            [odom_data_kinematic.odom_x, odom_data_kinematic.odom_y])
        odom_readings_mecanum.append(
            [odom_data_mecanum.odom_x, odom_data_mecanum.odom_y])
        odom_readings_rocker_bogie.append(
            [odom_data_rocker_bogie.odom_x, odom_data_rocker_bogie.odom_y])

        imu_gyro_noise_dynamic = np.random.normal(
            0, np.sqrt(kf_dynamic_model.R_imu[0, 0]))
        imu_gyro_noise_kinematic = np.random.normal(
            0, np.sqrt(kf_kinematic_model.R_imu[0, 0]))
        imu_gyro_noise_mecanum = np.random.normal(
            0, np.sqrt(kf_mecanum_model.R_imu[0, 0]))
        imu_gyro_noise_rocker_bogie = np.random.normal(
            0, np.sqrt(kf_rocker_bogie_model.R_imu[0, 0]))

        update_imu_dynamic = IMUData(acc_x=true_acc_x, acc_y=true_acc_y,
                                     gyro_z=true_omega + imu_gyro_noise_dynamic, timestamp=timestamp)
        update_imu_kinematic = IMUData(acc_x=true_acc_x, acc_y=true_acc_y,
                                       gyro_z=true_omega + imu_gyro_noise_kinematic, timestamp=timestamp)
        update_imu_mecanum = IMUData(acc_x=true_acc_x, acc_y=true_acc_y,
                                     gyro_z=true_omega + imu_gyro_noise_mecanum, timestamp=timestamp)
        update_imu_rocker_bogie = IMUData(
            acc_x=true_acc_x, acc_y=true_acc_y, gyro_z=true_omega + imu_gyro_noise_rocker_bogie, timestamp=timestamp)

        kf_dynamic_model.update_imu(update_imu_dynamic)
        kf_kinematic_model.update_imu(update_imu_kinematic)
        kf_mecanum_model.update_imu(update_imu_mecanum)
        kf_rocker_bogie_model.update_imu(update_imu_rocker_bogie)

        timestamps.append(timestamp)
        estimated_states_dynamic.append(
            kf_dynamic_model.state.to_array().copy())
        estimated_states_kinematic.append(
            kf_kinematic_model.state.to_array().copy())
        estimated_states_mecanum.append(
            kf_mecanum_model.state.to_array().copy())
        estimated_states_rocker_bogie.append(
            kf_rocker_bogie_model.state.to_array().copy())

        if i % 20 == 0:
            robot_state_history["dynamic"].append(kf_dynamic_model.state)
            robot_state_history["kinematic"].append(kf_kinematic_model.state)
            robot_state_history["mecanum"].append(kf_mecanum_model.state)
            robot_state_history["rocker_bogie"].append(
                kf_rocker_bogie_model.state)

    estimated_states_dynamic = np.array(estimated_states_dynamic)
    estimated_states_kinematic = np.array(estimated_states_kinematic)
    estimated_states_mecanum = np.array(estimated_states_mecanum)
    estimated_states_rocker_bogie = np.array(estimated_states_rocker_bogie)
    true_states = np.array(true_states)
    odom_readings_dynamic = np.array(odom_readings_dynamic)
    odom_readings_kinematic = np.array(odom_readings_kinematic)
    odom_readings_mecanum = np.array(odom_readings_mecanum)
    odom_readings_rocker_bogie = np.array(odom_readings_rocker_bogie)

    # --- Plotting ---

    # Plot 1: Dynamic Model
    plt.figure(figsize=(12, 10))
    plt.plot(true_states[:, 0], true_states[:, 1], 'g-',
             linewidth=6, alpha=0.8, label='True Path')
    plt.plot(estimated_states_dynamic[:, 0], estimated_states_dynamic[:,
             1], 'b-', linewidth=2, alpha=0.6, label='Dynamic Model KF')
    plt.scatter(odom_readings_dynamic[:, 0], odom_readings_dynamic[:, 1],
                c='orange', marker='x', s=60, alpha=0.8, label='Noisy Odometry')
    plt.xlabel('x [mm]')
    plt.ylabel('y [mm]')
    plt.title('Robot Trajectory Estimation: Dynamic Model')
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.axis('equal')
    plt.tight_layout()
    plt.savefig(os.path.join(
        PLOTS_DIR, 'robot_trajectory_dynamic_model.png'), dpi=300)
    plt.close()

    # Plot 2: Diff-Drive Kinematic Model
    plt.figure(figsize=(12, 10))
    plt.plot(true_states[:, 0], true_states[:, 1], 'g-',
             linewidth=6, alpha=0.8, label='True Path')
    plt.plot(estimated_states_kinematic[:, 0], estimated_states_kinematic[:, 1],
             'r-', linewidth=2, alpha=0.6, label='Diff-Drive Kinematic Model KF')
    plt.scatter(odom_readings_kinematic[:, 0], odom_readings_kinematic[:, 1],
                c='purple', marker='+', s=60, alpha=0.8, label='Noisy Odometry')
    plt.xlabel('x [mm]')
    plt.ylabel('y [mm]')
    plt.title('Robot Trajectory Estimation: Diff-Drive Kinematic Model')
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.axis('equal')
    plt.tight_layout()
    plt.savefig(os.path.join(
        PLOTS_DIR, 'robot_trajectory_diff_drive_kinematic.png'), dpi=300)
    plt.close()

    # Plot 3: Mecanum Kinematic Model
    plt.figure(figsize=(12, 10))
    plt.plot(true_states[:, 0], true_states[:, 1], 'g-',
             linewidth=6, alpha=0.8, label='True Path')
    plt.plot(estimated_states_mecanum[:, 0], estimated_states_mecanum[:,
             1], 'm-', linewidth=2, alpha=0.6, label='Mecanum Model KF')
    plt.scatter(odom_readings_mecanum[:, 0], odom_readings_mecanum[:, 1],
                c='cyan', marker='o', s=30, alpha=0.8, label='Noisy Odometry')
    plt.xlabel('x [mm]')
    plt.ylabel('y [mm]')
    plt.title('Robot Trajectory Estimation: Mecanum Kinematic Model')
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.axis('equal')
    plt.tight_layout()
    plt.savefig(os.path.join(
        PLOTS_DIR, 'robot_trajectory_mecanum_kinematic.png'), dpi=300)
    plt.close()

    # Plot 4: Rocker-Bogie Kinematic Model
    plt.figure(figsize=(12, 10))
    ax = plt.gca()
    plt.plot(true_states[:, 0], true_states[:, 1], 'g-',
             linewidth=6, alpha=0.8, label='True Path')
    plt.plot(estimated_states_rocker_bogie[:, 0], estimated_states_rocker_bogie[:, 1],
             'y-', linewidth=2, alpha=0.6, label='Rocker-Bogie Kinematic Model KF')
    plt.scatter(odom_readings_rocker_bogie[:, 0], odom_readings_rocker_bogie[:, 1],
                c='brown', marker='s', s=30, alpha=0.8, label='Noisy Odometry')
    for state in robot_state_history["rocker_bogie"]:
        draw_rocker_bogie_at_state(ax, show_text=False, x=state.x, y=state.y, theta=state.theta,
                                   vx=state.vx, vy=state.vy, omega=state.omega)
    plt.xlabel('x [mm]')
    plt.ylabel('y [mm]')
    plt.title('Robot Trajectory Estimation: Rocker-Bogie Kinematic Model')
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.axis('equal')
    plt.tight_layout()
    plt.savefig(os.path.join(
        PLOTS_DIR, 'robot_trajectory_rocker_bogie_kinematic.png'), dpi=300)
    plt.close()

    # Plot 5: Model Comparisons
    plt.figure(figsize=(12, 10))
    plt.plot(true_states[:, 0], true_states[:, 1], 'g-',
             linewidth=6, alpha=0.8, label='True Path')
    plt.plot(estimated_states_dynamic[:, 0], estimated_states_dynamic[:, 1],
             'b-', linewidth=2, alpha=0.6, label='Dynamic Model EKF')
    plt.plot(estimated_states_kinematic[:, 0], estimated_states_kinematic[:, 1],
             'r-', linewidth=2, alpha=0.6, label='Diff-Drive Kinematic Model EKF')
    plt.plot(estimated_states_mecanum[:, 0], estimated_states_mecanum[:, 1],
             'm-', linewidth=2, alpha=0.6, label='Mecanum Model EKF')
    plt.plot(estimated_states_rocker_bogie[:, 0], estimated_states_rocker_bogie[:, 1],
             'y-', linewidth=2, alpha=0.6, label='Rocker-Bogie Model EKF')
    plt.scatter(odom_readings_rocker_bogie[:, 0], odom_readings_rocker_bogie[:, 1],
                c='orange', marker='x', s=60, alpha=0.8, label='Noisy Odometry')
    plt.xlabel('x [mm]')
    plt.ylabel('y [mm]')
    plt.title('Robot Trajectory Estimation: Model Comparisons')
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.axis('equal')
    plt.tight_layout()
    plt.savefig(os.path.join(
        PLOTS_DIR, 'robot_trajectory_comparison.png'), dpi=300)
    plt.close()

    print(f"\nFigures saved successfully to '{PLOTS_DIR}/' directory:")
    print("- robot_trajectory_dynamic_model.png")
    print("- robot_trajectory_diff_drive_kinematic.png")
    print("- robot_trajectory_mecanum_kinematic.png")
    print("- robot_trajectory_rocker_bogie_kinematic.png")
    print("- robot_trajectory_comparison.png")
