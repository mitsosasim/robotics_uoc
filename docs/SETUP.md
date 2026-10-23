# Setup and validation

## Verified Python workflow

The self-contained replay requires Python 3.10+ and NumPy. It was run with Python 3.12 on Linux. It imports the filter classes directly from the same `kalman.py`, `ekf.py`, and `particles.py` files used by the ROS nodes. ROS imports are deferred or optional.

From the repository root:

```sh
python3 -m pip install -r requirements.txt
python3 -m unittest discover -s tests -v
python3 tools/simulate.py
python3 -m http.server 8765 --bind 127.0.0.1
```

Open <http://127.0.0.1:8765/tools/viewer.html>. `--assignment 1`, `2`, or `3` runs one exercise. `--seed` changes the reproducible random stream. Output JSON and the metrics summary go to `build/`. The viewer reads those generated files, renders robot/map geometry, and replays stored estimates. It does not implement a second filter in JavaScript.

The 10 numerical tests check a closed-form KF posterior, straight-line motion, invalid measurements, EKF Jacobians by finite differences, analytic noiseless circles and zero-turn motion, covariance symmetry/positivity, resampling frequencies, particle bounds, extreme likelihood normalization, exact equivalence to the supplied map helper, and seeded replay accuracy. The last test covers all three scenarios. CI runs the tests and all simulations.

The fixed seed-7 results are in [results.json](results.json). They use synthetic measurements and known truth. They are functional results for these scenarios, not timing benchmarks, statistical confidence bounds, hardware tests, or evidence that the ROS/Gazebo integration was exercised.

## Reproduce screenshots

After generating the replay JSON:

```sh
npm install --no-save --package-lock=false playwright@1.62.1
npx playwright install chromium --only-shell
node tools/capture.cjs
```

The capture script starts its own temporary loopback server, opens the real viewer, checks that timeline controls change the canvas, and writes `docs/assets/assignment1.png` through `assignment3.png`. It can use an existing Chromium binary through `CHROMIUM_EXECUTABLE_PATH`. Browser dependencies are optional and separate from filter execution. The screenshot metric cards describe the complete run, while the timeline changes the displayed frame and current error.

## ROS 1 course workspace

These interfaces target the supplied ROS 1/catkin environment. ROS, Gazebo Classic, RViz, and the robot packages were unavailable during verification. The commands below describe the integration path and remain unverified. Use an existing compatible course environment with Python 3 ROS bindings, `rospy`, `tf`, message packages, `gazebo_ros`, TurtleBot3 packages, and the SJTU drone package for assignment 3.

Create links to the three packages from the repository root:

```sh
mkdir -p "$HOME/475_ws/src"
ln -s "$PWD/assignments/01_kalman_filter/assign1" "$HOME/475_ws/src/assign1"
ln -s "$PWD/assignments/02_extended_kalman_filter/assign2" "$HOME/475_ws/src/assign2"
ln -s "$PWD/assignments/03_particle_filter/assign3" "$HOME/475_ws/src/assign3"
cd "$HOME/475_ws"
catkin_make
source devel/setup.bash
```

Run one assignment at a time because the sensor topic names overlap. In one terminal, launch `roslaunch assign1 burger.launch` or `roslaunch assign2 burger.launch`. In a second sourced terminal, run `rosrun assign1 kalman.py` or `rosrun assign2 ekf.py`. Their outputs match the supplied RViz subscriptions. TurtleBot3 defaults to the `burger` model. Assignment 2's supplied `Float64` publisher construction has been corrected to instantiate message objects.

## Drone world

The course instructions point to [tahsinkose/sjtu-drone](https://github.com/tahsinkose/sjtu-drone), whose upstream README targets an older ROS/Gazebo environment. The handout additionally calls for changing its `ignition/math4/ignition/math.hh` include to `ignition/math6/ignition/math.hh` for the course environment. No compatibility with an arbitrary current ROS/Gazebo release is claimed.

Clone/build that external package in the compatible workspace, following its requirements. It supplies the drone meshes, plugin, keyboard node, and `fly_indoor` room mesh. From this repository root, resolve the archived absolute mesh paths without editing the supplied world:

```sh
python3 tools/prepare_world.py --sjtu-path "$HOME/475_ws/src/sjtu-drone"
roslaunch assign3 drone.launch world_name:="$PWD/build/new_world.world"
```

In another sourced terminal, run `rosrun assign3 particles.py`. Use the supplied keyboard controller to take off and move in x/y while keeping yaw fixed. The 0.01 m measurement grid assumes the exercise's `[0,8) × [0,15)` domain. The saved SDF state has box positions inside that domain. General maps, negative coordinates, rotated obstacles, sensor outages, and arbitrary flight attitudes need additional modeling.

The particle filter uses the helper's synthetic displacement/range information. This is not localization from a real onboard range sensor. The offline replay uses independent uniform errors of ±0.05 m, consistent with the helper noise bounds, but it is not a simulation of the asynchronous helper calls or Gazebo dynamics.

## Report source

The two-page report uses pdfLaTeX. The Overleaf ZIP contains `main.tex` at its root and the local figure, and needs no shell escape or external conversion. To regenerate the figure, install Matplotlib separately and run `python3 tools/plot_report.py` after the simulations. Compile `docs/report/main.tex` from that directory with two passes of `pdflatex -interaction=nonstopmode -halt-on-error main.tex`. The figure compares the three active implementations on the same saved runs shown by the viewer.
