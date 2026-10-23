# Kalman filter · GPS and heading fusion

Estimate a robot's planar position while it moves forward at 0.3 m/s. Noisy heading determines the predicted displacement, and GPS corrects accumulated position error.

![Running Kalman-filter replay](../../docs/assets/assignment1.png)

*Actual Python replay, seed 7, 600 steps at 0.1 s. Blue is the true robot, green the estimate, and amber the GPS samples.*

**Python · NumPy · linear Kalman filter · ROS 1 · TurtleBot3/RViz configuration**

The [implementation](assign1/src/kalman.py) uses a two-coordinate position state. Heading is an observed input, so the position-state transition is linear with a time-varying displacement. Prediction propagates speed and heading uncertainty into position covariance. The GPS update uses a linear solve and Joseph-form covariance update.

The supplied sensor script uses independent GPS errors with 0.3 m standard deviation and `abs(gauss(0, 0.01))` heading noise. The replay preserves that positive heading bias. Heading itself is not estimated by this position filter.

From the repository root:

```sh
python3 tools/simulate.py --assignment 1
python3 -m http.server 8765 --bind 127.0.0.1
```

Open <http://127.0.0.1:8765/tools/viewer.html?assignment=1>. This run gives position RMSE **0.049 m**, compared with **0.406 m** for GPS and **0.089 m** for dead reckoning. RMSE is the square root of the mean squared 2D position error over the whole replay.

| ROS input | Use |
| --- | --- |
| `/measurement/gps` · `PointStamped` | Position correction |
| `/measurement/yaw` · `PointStamped.point.z` | Heading for displacement prediction |
| `/kalman_est` · `Odometry` output | Estimated position, heading and covariance |

The ROS node initializes position from the first GPS sample. Each GPS callback advances the filter to that sample's timestamp and corrects once. A 10 Hz timer publishes the latest estimate. [ROS setup and limitations](../../docs/SETUP.md#ros-1-course-workspace) · [Collection](../../README.md).
