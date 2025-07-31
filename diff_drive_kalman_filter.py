import math
import numpy as np
import matplotlib.pyplot as plt

# Constants
WHEEL_RADIUS = 50  # [mm]
WHEEL_BASE = 1000  # [mm]


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
        self.P[0, 0] = 0.1  # Position uncertainty in x [mm^2]
        self.P[1, 1] = 0.1  # Position uncertainty in y [mm^2]
        self.P[2, 2] = 0.01  # Orientation uncertainty [rad^2]
        self.P[3, 3] = 0.1  # Velocity uncertainty in x [mm^2/s^2]
        self.P[4, 4] = 0.1  # Velocity uncertainty in y [mm^2/s^2]
        self.P[5, 5] = 0.1  # Angular velocity uncertainty [rad^2/s^2]
        self.Q = np.eye(6)  # Process noise covariance
        self.Q[0, 0] = 0.01    # X position noise
        self.Q[1, 1] = 0.01    # Y position noise
        self.Q[2, 2] = 0.005   # Orientation noise
        self.Q[3, 3] = 0.1     # Velocity noise in x
        self.Q[4, 4] = 0.1     # Velocity noise in y
        self.Q[5, 5] = 0.05    # Angular velocity noise
        # [x, y, theta] Odometry measurement noise covariance
        self.R_odom = np.eye(3)
        self.R_odom[0, 0] = 5.0  # Position uncertainty in x [mm^2]
        self.R_odom[1, 1] = 5.0  # Position uncertainty in y [mm^2]
        self.R_odom[2, 2] = 0.1  # Orientation uncertainty [rad^2]
        # [omega] IMU measurement noise covariance
        self.R_imu = np.array([[0.05]])  # Yaw rate noise [rad^2/s^2]
        self.last_timestamp = initial_timestamp  # [seconds]

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

    def predict_with_kinematics_model(self, left_vel: float, right_vel: float, timestamp: float):
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
        F[0, 2] = V * -np.sin(theta) * dt  # dx'/dtheta
        F[1, 1] = 1.0  # dy'/dy
        F[1, 2] = V * np.cos(theta) * dt  # dy'/dtheta
        F[2, 2] = 1.0  # dtheta'/dtheta
        # Velocities are determined by inputs, not previous velocities,
        # but their orientation depends on the old theta.
        # d(vx')/dtheta via chain rule
        F[3, 2] = V * -np.sin(theta_new) * (W*dt)
        # d(vy')/dtheta via chain rule
        F[4, 2] = V * np.cos(theta_new) * (W*dt)
        # Omega is determined entirely by input W.

        # --- 4. Predict the covariance ---
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
    # --- Simulation Setup ---
    # Initial state and time
    initial_timestamp = 0.0
    initial_state = RobotState(
        x=0, y=0, theta=np.pi/4, vx=100.0, vy=0, omega=0.2
    )

    # Let's adjust initial velocities to match the initial theta for a smooth start
    V = 100.0  # Speed in the robot's forward direction
    initial_state.vx = V * np.cos(initial_state.theta)
    initial_state.vy = V * np.sin(initial_state.theta)

    kf_dynamic_model = KalmanFilter(initial_state, initial_timestamp)
    kf_kinematic_model = KalmanFilter(initial_state, initial_timestamp)

    # Lists to store data for plotting
    timestamps = [initial_timestamp]
    estimated_states_dynamic = [kf_dynamic_model.state.to_array().copy()]
    estimated_states_kinematic = [kf_kinematic_model.state.to_array().copy()]
    true_states = [initial_state.to_array().copy()]
    odom_readings_dynamic = []
    odom_readings_kinematic = []

    # Simulation parameters
    total_time = 10.0  # Total simulation time [seconds]
    dt = 0.1
    num_steps = int(total_time / dt)

    # --- Simulation Loop ---
    for i in range(1, num_steps + 1):
        timestamp = i * dt

        # 1. Define True Motion (a cubic trajectory)
        # Create a cubic path from (0,0) to (0,50) with interesting curvature
        # Using parametric equations based on time progression

        # Get previous true state
        last_true_state = true_states[-1]

        # Define trajectory parameters
        total_distance = 50.0  # Total distance to travel [mm]
        progress = timestamp / total_time  # Progress from 0 to 1

        # Cubic trajectory: x follows a cubic curve, y increases linearly
        # x = A * t^3 + B * t^2 + C * t + D
        # Boundary conditions: x(0)=0, x(1)=0, dx/dt(0)=20, dx/dt(1)=-20
        # This creates an S-curve that starts at (0,0) and ends at (0,50)

        A = -40.0  # Coefficient for t^3
        B = 60.0   # Coefficient for t^2
        C = 20.0   # Coefficient for t
        D = 0.0    # Constant term

        # Calculate position
        true_x_new = A * (progress**3) + B * (progress**2) + C * progress + D
        true_y_new = total_distance * progress

        # Calculate velocities (derivatives with respect to time)
        dt_dprogress = dt / total_time
        dx_dt = (3*A*(progress**2) + 2*B*progress + C) * (1/total_time)
        dy_dt = total_distance / total_time

        true_vx_new = dx_dt
        true_vy_new = dy_dt
        actual_speed = np.sqrt(true_vx_new**2 + true_vy_new**2)

        # Calculate heading and angular velocity
        true_theta_new = np.arctan2(true_vy_new, true_vx_new)

        # Calculate angular velocity from change in heading
        if len(true_states) > 0:
            last_theta = last_true_state[2]
            theta_change = true_theta_new - last_theta
            # Normalize angle difference
            while theta_change > np.pi:
                theta_change -= 2*np.pi
            while theta_change < -np.pi:
                theta_change += 2*np.pi
            true_omega = theta_change / dt
        else:
            true_omega = 0.0

        # Calculate accelerations
        if len(true_states) > 0:
            last_speed = np.sqrt(last_true_state[3]**2 + last_true_state[4]**2)
            speed_change = actual_speed - last_speed
            true_acc_x = speed_change / dt  # Tangential acceleration
            # Centripetal acceleration
            true_acc_y = actual_speed * abs(true_omega)
        else:
            true_acc_x = 0.0
            true_acc_y = 0.0

        current_true_state = np.array(
            [true_x_new, true_y_new, true_theta_new, true_vx_new, true_vy_new, true_omega])
        true_states.append(current_true_state)

        # 2. Prediction Step
        # Create IMU data object for prediction (using true acceleration as control input)
        predict_imu = IMUData(
            acc_x=true_acc_x, acc_y=true_acc_y, gyro_z=0, timestamp=timestamp)
        kf_dynamic_model.predict_with_dynamics_model(predict_imu)

        # For kinematic model, calculate wheel velocities from robot motion
        robot_speed = np.sqrt(true_vx_new**2 + true_vy_new**2)
        kf_kinematic_model.predict_with_kinematics_model(
            left_vel=robot_speed - (true_omega * WHEEL_BASE / 2),
            right_vel=robot_speed + (true_omega * WHEEL_BASE / 2),
            timestamp=timestamp
        )

        # 3. Correction Step (with noisy sensor data)
        # Create noisy Odometry measurement
        odom_noise_dynamic_model = np.random.multivariate_normal(
            np.zeros(3), kf_dynamic_model.R_odom)
        odom_noise_kinematic = np.random.multivariate_normal(
            np.zeros(3), kf_kinematic_model.R_odom)
        odom_data_dynamic_model = OdomData(
            x=true_x_new + odom_noise_dynamic_model[0],
            y=true_y_new + odom_noise_dynamic_model[1],
            theta=kf_dynamic_model.normalize_angle(
                true_theta_new + odom_noise_dynamic_model[2]),
            timestamp=timestamp
        )
        odom_data_kinematic_model = OdomData(
            x=true_x_new + odom_noise_kinematic[0],
            y=true_y_new + odom_noise_kinematic[1],
            theta=kf_kinematic_model.normalize_angle(
                true_theta_new + odom_noise_kinematic[2]),
            timestamp=timestamp
        )
        kf_dynamic_model.update_odom(odom_data_dynamic_model)
        kf_kinematic_model.update_odom(odom_data_kinematic_model)
        odom_readings_dynamic.append(
            [odom_data_dynamic_model.odom_x, odom_data_dynamic_model.odom_y])
        odom_readings_kinematic.append(
            [odom_data_kinematic_model.odom_x, odom_data_kinematic_model.odom_y])

        # Create noisy IMU measurement
        imu_gyro_noise_dynamic_model = np.random.normal(
            0, np.sqrt(kf_dynamic_model.R_imu[0, 0]))
        imu_gyro_noise_kinematic_model = np.random.normal(
            0, np.sqrt(kf_kinematic_model.R_imu[0, 0]))
        update_imu_dynamic = IMUData(
            acc_x=true_acc_x,  # Accelerations not used in update
            acc_y=true_acc_y,
            gyro_z=true_omega + imu_gyro_noise_dynamic_model,
            timestamp=timestamp
        )
        update_imu_kinematic = IMUData(
            acc_x=true_acc_x,
            acc_y=true_acc_y,
            gyro_z=true_omega + imu_gyro_noise_kinematic_model,
            timestamp=timestamp
        )
        kf_dynamic_model.update_imu(update_imu_dynamic)
        kf_kinematic_model.update_imu(update_imu_kinematic)

        # Store results for plotting
        timestamps.append(timestamp)
        estimated_states_dynamic.append(
            kf_dynamic_model.state.to_array().copy())
        estimated_states_kinematic.append(
            kf_kinematic_model.state.to_array().copy())

    # --- Plotting ---
    import os

    # Create plots directory if it doesn't exist
    plots_dir = "plots"
    if not os.path.exists(plots_dir):
        os.makedirs(plots_dir)

    estimated_states_dynamic = np.array(estimated_states_dynamic)
    estimated_states_kinematic = np.array(estimated_states_kinematic)
    true_states = np.array(true_states)
    odom_readings_dynamic = np.array(odom_readings_dynamic)
    odom_readings_kinematic = np.array(odom_readings_kinematic)

    # Plot Robot Trajectory Comparison
    plt.figure(figsize=(12, 10))
    plt.plot(true_states[:, 0], true_states[:, 1], 'g-',
             linewidth=6, alpha=0.8, label='True Path')
    plt.plot(estimated_states_dynamic[:, 0], estimated_states_dynamic[:, 1], 'b-',
             linewidth=2, alpha=0.6, label='Dynamic Model KF')
    plt.plot(estimated_states_kinematic[:, 0], estimated_states_kinematic[:, 1], 'r-',
             linewidth=2, alpha=0.6, label='Kinematic Model KF')
    plt.scatter(odom_readings_dynamic[:, 0], odom_readings_dynamic[:, 1],
                c='orange', marker='x', s=60, alpha=0.8, label='Noisy Odometry (Dynamic)')
    plt.scatter(odom_readings_kinematic[:, 0], odom_readings_kinematic[:, 1],
                c='purple', marker='+', s=60, alpha=0.8, label='Noisy Odometry (Kinematic)')
    plt.xlabel('x [mm]')
    plt.ylabel('y [mm]')
    plt.title('Robot Trajectory Estimation: Dynamic vs Kinematic Models')
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.axis('equal')
    plt.tight_layout()
    plt.savefig(os.path.join(
        plots_dir, 'robot_trajectory_comparison.png'), dpi=300)
    plt.show()

    # Plot individual states comparison
    fig, axes = plt.subplots(3, 2, figsize=(16, 14), sharex=True)
    state_labels = ['x [mm]', 'y [mm]', 'theta [rad]',
                    'vx [mm/s]', 'vy [mm/s]', 'omega [rad/s]']
    for i, (ax, label) in enumerate(zip(axes.flat, state_labels)):
        ax.plot(timestamps, true_states[:, i], 'g-',
                linewidth=3, alpha=0.7, label='True')
        ax.plot(timestamps, estimated_states_dynamic[:, i],
                'b-', linewidth=2, label='Dynamic Model KF')
        ax.plot(timestamps, estimated_states_kinematic[:, i],
                'r-', linewidth=2, label='Kinematic Model KF')
        ax.set_ylabel(label)
        ax.set_title(f'Evolution of State: {label}')
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.legend()
    axes.flat[-1].set_xlabel('Time [s]')
    axes.flat[-2].set_xlabel('Time [s]')
    fig.suptitle(
        'Kalman Filter State Evolution: Dynamic vs Kinematic Models', fontsize=16)
    plt.tight_layout(rect=[0, 0.03, 1, 0.96])
    plt.savefig(os.path.join(
        plots_dir, 'kalman_filter_states_comparison.png'), dpi=300)
    plt.show()

    # Plot error comparison
    fig, axes = plt.subplots(3, 2, figsize=(16, 14), sharex=True)
    error_dynamic = estimated_states_dynamic - true_states
    error_kinematic = estimated_states_kinematic - true_states

    # Normalize angle errors
    error_dynamic[:, 2] = np.array(
        [kf_dynamic_model.normalize_angle(e) for e in error_dynamic[:, 2]])
    error_kinematic[:, 2] = np.array(
        [kf_kinematic_model.normalize_angle(e) for e in error_kinematic[:, 2]])

    error_labels = ['x Error [mm]', 'y Error [mm]', 'theta Error [rad]',
                    'vx Error [mm/s]', 'vy Error [mm/s]', 'omega Error [rad/s]']

    for i, (ax, label) in enumerate(zip(axes.flat, error_labels)):
        ax.plot(timestamps, error_dynamic[:, i], 'b-',
                linewidth=2, label='Dynamic Model KF')
        ax.plot(timestamps, error_kinematic[:, i], 'r-',
                linewidth=2, label='Kinematic Model KF')
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.5)
        ax.set_ylabel(label)
        ax.set_title(f'Estimation Error: {label}')
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.legend()

    axes.flat[-1].set_xlabel('Time [s]')
    axes.flat[-2].set_xlabel('Time [s]')
    fig.suptitle(
        'Kalman Filter Estimation Errors: Dynamic vs Kinematic Models', fontsize=16)
    plt.tight_layout(rect=[0, 0.03, 1, 0.96])
    plt.savefig(os.path.join(
        plots_dir, 'kalman_filter_errors_comparison.png'), dpi=300)
    plt.show()

    # Print performance metrics
    rmse_dynamic = np.sqrt(np.mean(error_dynamic**2, axis=0))
    rmse_kinematic = np.sqrt(np.mean(error_kinematic**2, axis=0))

    # Prepare performance summary text
    summary_lines = []
    summary_lines.append("\n" + "="*60)
    summary_lines.append("KALMAN FILTER PERFORMANCE COMPARISON")
    summary_lines.append("="*60)
    summary_lines.append(
        f"{'State':<15} {'Dynamic RMSE':<15} {'Kinematic RMSE':<15} {'Better Model'}")
    summary_lines.append("-"*60)

    state_names = ['x [mm]', 'y [mm]', 'theta [rad]',
                   'vx [mm/s]', 'vy [mm/s]', 'omega [rad/s]']
    for i, state_name in enumerate(state_names):
        better = "Dynamic" if rmse_dynamic[i] < rmse_kinematic[i] else "Kinematic"
        summary_lines.append(
            f"{state_name:<15} {rmse_dynamic[i]:<15.3f} {rmse_kinematic[i]:<15.3f} {better}")

    summary_lines.append("-"*60)
    overall_rmse_dynamic = np.mean(rmse_dynamic)
    overall_rmse_kinematic = np.mean(rmse_kinematic)
    overall_better = "Dynamic" if overall_rmse_dynamic < overall_rmse_kinematic else "Kinematic"
    summary_lines.append(
        f"{'Overall':<15} {overall_rmse_dynamic:<15.3f} {overall_rmse_kinematic:<15.3f} {overall_better}")

    # Print to terminal
    for line in summary_lines:
        print(line)

    # Save to file
    with open(os.path.join(plots_dir, "KF_results.txt"), "w") as f:
        for line in summary_lines:
            f.write(line + "\n")

    print(f"\nFigures saved successfully to '{plots_dir}/' directory:")
    print("- robot_trajectory_comparison.png")
    print("- kalman_filter_states_comparison.png")
    print("- kalman_filter_errors_comparison.png")
    print("- KF_results.txt")
