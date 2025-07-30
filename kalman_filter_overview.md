# Kalman Filter Overview

## Robot State & Dynamics
Define state vector $x$ and how control inputs to the robot contribute to the robot's state. The robot's state vector $X$ and control vector $U$ can be expressed as:

$$X = \left[x, y, \theta, v_x, v_y\right]$$

$$U = \left[v_{left}, v_{right}\right]$$

In order to derive the dynamics and eventually the state transition function, we need to identify how left and right velocities contribute to each entry in the state vector (at each timestep $\Delta t$).

$$V = \frac{v_{left} + v_{right}}{2} \\
\omega = \frac{v_{right} - v_{left}}{L_{base}}
$$

$$\begin{bmatrix}x \\ y \\ \theta \\ v_{x} \\ v_{y} \end{bmatrix} = \underbrace{\begin{bmatrix}x_{prev} + V \cos\theta_{prev} \Delta t\\y_{prev} + V\sin\theta _{prev}\Delta t\\ \theta_{prev} + \omega \Delta t\\ V a_x \Delta t\\ V a_y \Delta t\end{bmatrix}}_{\text{accel from IMU}}$$

$$
\text{dynamics}\left(X_{prev}, a_x, a_y, \Delta t\right) \to X_{next}
$$`

### State Transition Matrix
The state transition matrix $F$ relates a new state to the previous state as a matrix of derivatives:

$$
F = \begin{bmatrix}\frac{\delta x_{new}}{\delta x} & \frac{\delta x_{new}}{\delta y} & \frac{\delta x_{new}}{\delta \theta} & \frac{\delta x_{new}}{\delta v_x} & \frac{\delta x_{new}}{\delta v_y}\\ \frac{\delta y_{new}}{\delta x} & \frac{\delta y_{new}}{\delta y} & \frac{\delta y_{new}}{\delta \theta} & \frac{\delta y_{new}}{\delta v_x} & \frac{\delta y_{new}}{\delta v_y}\\ \frac{\delta \theta_{new}}{\delta x} & \frac{\delta \theta_{new}}{\delta y} & \frac{\delta \theta_{new}}{\delta \theta} & \frac{\delta \theta_{new}}{\delta v_x} & \frac{\delta \theta_{new}}{\delta v_y}\\ \frac{\delta v_{x_{new}}}{\delta x} & \frac{\delta v_{x_{new}}}{\delta y} & \frac{\delta v_{x_{new}}}{\delta \theta} & \frac{v_{x_{new}}}{\delta v_x} & \frac{v_{y_{new}}}{\delta v_y} \\ \frac{\delta v_{y_{new}}}{\delta x} & \frac{\delta v_{y_{new}}}{\delta y} & \frac{\delta v_{y_{new}}}{\delta \theta} & \frac{v_{y_{new}}}{\delta v_x} & \frac{v_{y_{new}}}{\delta v_y}\end{bmatrix}
$$

## Sensor Updates

### IMU Sensor
The IMU measures the angular rate around the Z-axs $\omega$ as well as the linear acceleration $a_x, a_y$.

$$z_{imu} = \left[\omega, a_x, a_y\right]$$

### Odometry
Odometry uses wheel encoders coupled with knowledge about the robot's physical layout to obtain an estimate of the robot's pose:

$$z_{odom} = \left[x_{odom}, y_{odom}, \theta_{odom}\right]$$