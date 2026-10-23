# Extended Kalman filter · Circular robot motion

Track a differential-drive robot moving at 0.5 m/s with a 0.3 rad/s yaw rate. The filter estimates position, heading, linear velocity, and angular velocity from noisy GPS and velocity measurements.

![Running EKF replay](../../docs/assets/assignment2.png)

*Actual Python replay, seed 7, 630 steps at 0.1 s. The robot follows the analytic circular path while the estimate fuses noisy measurements.*

**Python · NumPy · nonlinear kinematics · analytic Jacobian · ROS 1**

The [five-state EKF](assign2/src/ekf.py) integrates constant-turn-rate motion exactly over each step. A sinc-based expression and a small-angle series keep prediction and its Jacobian continuous when angular velocity approaches zero. GPS and velocity updates use Joseph-form covariance updates and wrap heading to `[-π, π)`.

The supplied simulator adds the same GPS noise sample to both axes. The default measurement covariance therefore includes that correlation. Its velocity noise values are treated as standard deviations because they are passed directly to `random.gauss`, despite the original variable comments calling them variances.

From the repository root:

```sh
python3 tools/simulate.py --assignment 2
python3 -m http.server 8765 --bind 127.0.0.1
```

Open <http://127.0.0.1:8765/tools/viewer.html?assignment=2>. Position RMSE is **0.063 m**, versus **0.331 m** for GPS and **0.117 m** for dead reckoning. Ground truth comes from an independent analytic circle, rather than the filter's prediction function.

| ROS topic | Role |
| --- | --- |
| `/odom` · `Odometry` | One-time initial state |
| `/measurement/gps` · `PointStamped` | Position measurement |
| `/measurement/linear_velocity` · `Float64` | Speed measurement |
| `/measurement/angular_velocity` · `Float64` | Yaw-rate measurement |
| `/ekf` · `Odometry` | Estimated state and covariance |

The ROS adapter consumes each latest measurement once at a 10 Hz fusion tick and discards samples older than 0.5 s. It is an approximate asynchronous interface, not a delayed-measurement smoother. Set `~gps_correlation` to zero for independent GPS axes. [ROS setup and verification scope](../../docs/SETUP.md#ros-1-course-workspace) · [Collection](../../README.md).
