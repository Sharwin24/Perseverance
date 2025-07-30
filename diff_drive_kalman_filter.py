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
        self.odom_x = x  # Odometer x position [mm]
        self.odom_y = y  # Odometer y position [mm]
        self.odom_theta = theta  # Odometer orientation [radians]
        self.timestamp = timestamp  # Timestamp of the odometry data [seconds]


class IMUData:
    def __init__(self, acc_x, acc_y, gyro_z, timestamp):
        self.acc_x = acc_x  # Acceleration in robot's x direction [mm/s^2]
        self.acc_y = acc_y  # Acceleration in robot's y direction [mm/s^2]
        self.gyro_z = gyro_z  # Angular velocity around z axis [rad/s]
        self.timestamp = timestamp  # Timestamp of the IMU data [seconds]


class KalmanFilter:
    def __init__(self, initial_state: RobotState, initial_timestamp: float):
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

    def normalize_angle(self, angle):
        """Normalizes an angle to be within -pi to pi."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def predict(self, imu_data: IMUData):
        """
        Predicts the next state using a constant acceleration model.
        IMU accelerations are treated as control inputs.
        """
        dt = imu_data.timestamp - self.last_timestamp
        if dt <= 0:
            return

        theta = self.state.theta
        # Convert robot-frame acceleration to global-frame acceleration
        global_acc_x = imu_data.acc_x * \
            np.cos(theta) - imu_data.acc_y * np.sin(theta)
        global_acc_y = imu_data.acc_x * \
            np.sin(theta) + imu_data.acc_y * np.cos(theta)

        # State Transition Matrix (F) for a constant acceleration model
        F = np.array([
            [1, 0, 0, dt, 0,  0],
            [0, 1, 0, 0,  dt, 0],
            [0, 0, 1, 0,  0,  dt],
            [0, 0, 0, 1,  0,  0],
            [0, 0, 0, 0,  1,  0],
            [0, 0, 0, 0,  0,  1]
        ])

        # Control Input Model (B)
        B = np.array([
            [0.5 * dt**2, 0],
            [0, 0.5 * dt**2],
            [0, 0],
            [dt, 0],
            [0, dt],
            [0, 0]
        ])

        # Control Vector (u)
        u = np.array([global_acc_x, global_acc_y])

        # Predict state: x_pred = F * x + B * u
        predicted_state_arr = F @ self.state.to_array() + B @ u
        self.state = RobotState.from_npy(predicted_state_arr)
        self.state.theta = self.normalize_angle(self.state.theta)

        # Predict covariance: P_pred = F * P * F.T + Q
        self.P = F @ self.P @ F.T + self.Q

        self.last_timestamp = imu_data.timestamp

    def update_odom(self, odom_data: OdomData):
        # Create measurement vector from odometry data
        z = np.array(
            [odom_data.odom_x, odom_data.odom_y, odom_data.odom_theta])
        H = np.array(
            [[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0]])
        y = z - H @ self.state.to_array()
        y[2] = self.normalize_angle(y[2])
        S = H @ self.P @ H.T + self.R_odom
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = RobotState.from_npy(self.state.to_array() + K @ y)
        self.state.theta = self.normalize_angle(self.state.theta)
        self.P = (np.eye(self.P.shape[0]) - K @ H) @ self.P

    def update_imu(self, imu_data: IMUData):
        z = np.array([imu_data.gyro_z])
        H = np.array([[0, 0, 0, 0, 0, 1]])
        y = z - H @ self.state.to_array()
        S = H @ self.P @ H.T + self.R_imu
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = RobotState.from_npy(self.state.to_array() + K @ y)
        self.state.theta = self.normalize_angle(self.state.theta)
        self.P = (np.eye(self.P.shape[0]) - K @ H) @ self.P


if __name__ == "__main__":
    # --- Simulation Setup ---
    # Initial state and time
    initial_timestamp = 0.0
    initial_state = RobotState(
        x=0, y=0, theta=np.pi/4, vx=100.0, vy=0, omega=0.2)

    # Let's adjust initial velocities to match the initial theta for a smooth start
    V = 100.0  # Speed in the robot's forward direction
    initial_state.vx = V * np.cos(initial_state.theta)
    initial_state.vy = V * np.sin(initial_state.theta)

    kf = KalmanFilter(initial_state, initial_timestamp)

    # Lists to store data for plotting
    timestamps = [initial_timestamp]
    estimated_states = [kf.state.to_array().copy()]
    true_states = [initial_state.to_array().copy()]
    odom_readings = []

    # Simulation parameters
    total_time = 10.0
    dt = 0.1
    num_steps = int(total_time / dt)

    # --- Simulation Loop ---
    for i in range(1, num_steps + 1):
        timestamp = i * dt

        # 1. Define True Motion (a gentle circular arc)
        true_V = 100.0  # Constant forward speed
        true_omega = 0.2  # Constant turn rate [rad/s]

        # Robot-frame centripetal acceleration needed to turn
        # This is our "control input" for the prediction step
        true_acc_x = 0  # No change in forward speed
        true_acc_y = true_V * true_omega  # Centripetal acceleration a = V*omega

        # Get previous true state
        last_true_state = true_states[-1]

        # Update true state based on motion
        true_theta_new = last_true_state[2] + true_omega * dt
        true_vx_new = true_V * np.cos(true_theta_new)
        true_vy_new = true_V * np.sin(true_theta_new)
        true_x_new = last_true_state[0] + true_vx_new * dt
        true_y_new = last_true_state[1] + true_vy_new * dt

        current_true_state = np.array(
            [true_x_new, true_y_new, true_theta_new, true_vx_new, true_vy_new, true_omega])
        true_states.append(current_true_state)

        # 2. Prediction Step
        # Create IMU data object for prediction (using true acceleration as control input)
        predict_imu = IMUData(
            acc_x=true_acc_x, acc_y=true_acc_y, gyro_z=0, timestamp=timestamp)
        kf.predict(predict_imu)

        # 3. Correction Step (with noisy sensor data)
        # Create noisy Odometry measurement
        odom_noise = np.random.multivariate_normal(np.zeros(3), kf.R_odom)
        odom_data = OdomData(
            x=true_x_new + odom_noise[0],
            y=true_y_new + odom_noise[1],
            theta=kf.normalize_angle(true_theta_new + odom_noise[2]),
            timestamp=timestamp
        )
        kf.update_odom(odom_data)
        odom_readings.append([odom_data.odom_x, odom_data.odom_y])

        # Create noisy IMU measurement
        imu_gyro_noise = np.random.normal(0, np.sqrt(kf.R_imu[0, 0]))
        update_imu = IMUData(
            acc_x=true_acc_x,  # Accelerations not used in update
            acc_y=true_acc_y,
            gyro_z=true_omega + imu_gyro_noise,
            timestamp=timestamp
        )
        kf.update_imu(update_imu)

        # Store results for plotting
        timestamps.append(timestamp)
        estimated_states.append(kf.state.to_array().copy())

    # --- Plotting ---
    estimated_states = np.array(estimated_states)
    true_states = np.array(true_states)
    odom_readings = np.array(odom_readings)

    # Plot Robot Trajectory
    plt.figure(figsize=(10, 8))
    plt.plot(true_states[:, 0], true_states[:, 1], 'g-',
             linewidth=4, alpha=0.5, label='True Path')
    plt.plot(estimated_states[:, 0], estimated_states[:, 1], 'b.-',
             linewidth=2, markersize=8, label='Kalman Filter Estimate')
    plt.scatter(odom_readings[:, 0], odom_readings[:, 1],
                c='r', marker='x', s=50, label='Noisy Odometry')
    plt.xlabel('x [mm]')
    plt.ylabel('y [mm]')
    plt.title('Robot Trajectory Estimation')
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.axis('equal')
    plt.tight_layout()
    plt.savefig('robot_trajectory.png', dpi=300)
    plt.show()

    # Plot individual states
    fig, axes = plt.subplots(3, 2, figsize=(15, 12), sharex=True)
    state_labels = ['x [mm]', 'y [mm]', 'theta [rad]',
                    'vx [mm/s]', 'vy [mm/s]', 'omega [rad/s]']
    for i, (ax, label) in enumerate(zip(axes.flat, state_labels)):
        ax.plot(timestamps, true_states[:, i], 'g-',
                linewidth=3, alpha=0.5, label='True')
        ax.plot(timestamps, estimated_states[:, i],
                'b-', linewidth=2, label='Estimate')
        ax.set_ylabel(label)
        ax.set_title(f'Evolution of State: {label}')
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.legend()
    axes.flat[-1].set_xlabel('Time [s]')
    axes.flat[-2].set_xlabel('Time [s]')
    fig.suptitle('Kalman Filter State Evolution', fontsize=16)
    plt.tight_layout(rect=[0, 0.03, 1, 0.96])
    plt.savefig('kalman_filter_states.png', dpi=300)
    plt.show()

    print("Figures saved successfully.")
