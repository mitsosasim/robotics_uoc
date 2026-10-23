# Particle filter · Drone localization on a height map

Estimate a drone's unknown 2D position in an 8 × 15 m room. Particles move with noisy displacement measurements and receive weights according to how well the map height matches the measured altitude minus downward range.

![Running particle-filter replay with the drone and map](../../docs/assets/assignment3.png)

*Actual Python replay, seed 7. The viewer shows 1,200 of the filter's 12,000 particles on the supplied world's measurement surface. Playback starts with an uncertain global position and follows convergence as the drone moves.*

**Python · NumPy · Monte Carlo localization · SDF map parsing · ROS `PoseArray`**

The [filter](assign3/src/particles.py) uses a uniform position prior, displacement prediction with process noise, log-space likelihoods, normalized weights, and effective-sample-size monitoring. Systematic resampling is triggered below half the particle count. Out-of-bounds particles are reinitialized inside the room. A weighted mean and covariance summarize the posterior, which can be multimodal during initial localization.

The supplied `create_height_map`, `get_displacement`, and `find_vertical_distance` helper bodies remain unchanged. The offline map builder reproduces the same calculation using the world's saved SDF state poses, which differ from some model-definition poses. The 0.01 m height grid encodes a decreasing surface around box centers, as required by the exercise, rather than flat physical box tops.

From the repository root:

```sh
python3 tools/simulate.py --assignment 3
python3 -m http.server 8765 --bind 127.0.0.1
```

Open <http://127.0.0.1:8765/tools/viewer.html?assignment=3>. On this 355-step route, the initial weighted-mean error is **4.353 m**, the final error **0.045 m**, and RMSE over the last 100 steps **0.195 m**. This is one seeded scenario, not a guarantee of global convergence for every route.

The ROS node gets measurements only through the provided helper functions and publishes `/particle_filter` as a `PoseArray`: frame `map`, particle height 1 m, and identity orientation. Keep the drone's yaw fixed, as the supplied displacement model does not track it. The helpers block waiting for simulation messages. [Gazebo dependencies and world-path setup](../../docs/SETUP.md#drone-world) · [Collection](../../README.md).
