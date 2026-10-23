#!/usr/bin/env python3
"""Run the real filter classes on reproducible synthetic sensor streams."""
import argparse
import importlib.util
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ASSIGNMENTS = ['01_kalman_filter', '02_extended_kalman_filter', '03_particle_filter']


def load_filter(number):
    filename = ['kalman', 'ekf', 'particles'][number-1]
    p = ROOT/'assignments'/ASSIGNMENTS[number-1]/f'assign{number}'/'src'/f'{filename}.py'
    spec = importlib.util.spec_from_file_location(filename, p)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def rmse(a, b):
    return float(np.sqrt(np.mean(np.sum((np.asarray(a)-b)**2, axis=1))))


def run_kalman(seed=7):
    rng = np.random.default_rng(seed)
    cls = load_filter(1).KalmanFilter
    dt, n = 0.1, 600
    times = np.arange(1, n+1)*dt
    truth = np.column_stack([0.3*times, np.zeros(n)])
    gps = truth+rng.normal(0, 0.3, (n, 2))
    # The supplied navigation.py uses abs(gauss(0, .01)), not zero-mean noise.
    yaw = np.abs(rng.normal(0, 0.01, n))
    kf = cls(position=gps[0], covariance=np.eye(2)*0.3**2)
    dead, estimates, errors, frames = np.zeros(2), [], [], []
    dead_path = []
    for i in range(n):
        if i:
            kf.predict(yaw[i], dt)
            kf.update(gps[i])
        dead += 0.3*dt*np.array([np.cos(yaw[i]), np.sin(yaw[i])])
        dead_path.append(dead.copy())
        estimates.append(kf.x.copy())
        errors.append(float(np.linalg.norm(kf.x-truth[i])))
        if i % 10 == 0 or i == n-1:
            frames.append(dict(index=i, time=float(times[i]), position=kf.x.tolist(),
                               covariance=kf.P.tolist(), heading=float(yaw[i])))
    return dict(title='Kalman filter', subtitle='Straight-line robot · GPS + measured heading',
                seed=seed, dt=dt, times=times.tolist(), truth=truth.tolist(),
                measurements=gps.tolist(), estimate=np.array(estimates).tolist(),
                baseline=np.array(dead_path).tolist(), errors=errors, frames=frames,
                metrics=dict(gps_rmse_m=rmse(gps, truth), estimate_rmse_m=rmse(estimates, truth),
                             dead_reckoning_rmse_m=rmse(dead_path, truth)),
                bounds=[-0.8, 19, -1.5, 1.5])


def run_ekf(seed=7):
    rng = np.random.default_rng(seed)
    mod = load_filter(2)
    dt, n, v, w = 0.1, 630, 0.5, 0.3
    times = np.arange(1, n+1)*dt
    # Independent analytic oracle, not the EKF's motion function.
    truth = np.column_stack([v/w*np.sin(w*times), v/w*(1-np.cos(w*times))])
    noise = rng.normal(0, 0.25, (n, 1))
    gps = truth+noise  # Shared x/y noise matches supplied simulator.
    velocities = np.column_stack([v+rng.normal(0, .03, n), w+rng.normal(0, .03, n)])
    ekf = mod.ExtendedKalmanFilter()
    estimates, dead_path, frames, errors = [], [], [], []
    dead = np.zeros(3)
    for i in range(n):
        ekf.predict(dt)
        ekf.update_velocity(velocities[i, 0])
        ekf.update_velocity(velocities[i, 1], angular=True)
        ekf.update_gps(gps[i])
        # Plain dead reckoning from noisy speed/yaw-rate measurements.
        speed, rate = velocities[i]
        mid = dead[2]+rate*dt/2
        distance = speed*dt*np.sinc(rate*dt/(2*np.pi))
        dead += [distance*np.cos(mid), distance*np.sin(mid), rate*dt]
        dead_path.append(dead[:2].copy())
        estimates.append(ekf.x[:2].copy())
        errors.append(float(np.linalg.norm(ekf.x[:2]-truth[i])))
        if i % 10 == 0 or i == n-1:
            frames.append(dict(index=i, time=float(times[i]), position=ekf.x[:2].tolist(),
                               covariance=ekf.P[:2, :2].tolist(), heading=float(ekf.x[2])))
    return dict(title='Extended Kalman filter', subtitle='Circular robot motion · nonlinear five-state model',
                seed=seed, dt=dt, times=times.tolist(), truth=truth.tolist(),
                measurements=gps.tolist(), estimate=np.array(estimates).tolist(),
                baseline=np.array(dead_path).tolist(), errors=errors, frames=frames,
                metrics=dict(gps_rmse_m=rmse(gps, truth), estimate_rmse_m=rmse(estimates, truth),
                             dead_reckoning_rmse_m=rmse(dead_path, truth)),
                bounds=[-2.6, 2.6, -0.8, 4.3])


def run_particles(seed=7):
    rng = np.random.default_rng(seed)
    mod = load_filter(3)
    world = ROOT/'assignments'/ASSIGNMENTS[2]/'assign3/worlds/new_world.world'
    surface = mod.height_map_from_world(world)
    pf = mod.ParticleFilter(surface, seed=seed)
    waypoints = np.array([[1.48, 2.51], [2.48, .54], [4.49, 2.47], [5.51, 3.48],
                          [6.54, 4.48], [5.54, 6.48], [1.30, 10.48], [4.49, 2.47]])
    positions = [waypoints[0]]
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        steps = int(np.ceil(np.linalg.norm(b-a)/0.07))
        positions.extend(np.linspace(a, b, steps+1)[1:])
    truth = np.array(positions)
    estimates, errors, frames, observations, effective_counts = [], [], [], [], []
    for i, point in enumerate(truth):
        delta = point-truth[max(0, i-1)]
        # The helper supplies uniform +/- .05 m errors in displacement and range.
        measured_delta = delta+rng.uniform(-.05, .05, 2)
        height = pf.heights(point[None, :])[0]
        altitude = 2.0+rng.uniform(-.05, .05)
        range_to_surface = 2.0-height+rng.uniform(-.05, .05)
        mean, cov, effective = pf.step(measured_delta, range_to_surface, altitude)
        estimates.append(mean)
        errors.append(float(np.linalg.norm(mean-point)))
        observations.append(float(altitude-range_to_surface))
        effective_counts.append(effective)
        if i % 12 == 0 or i == len(truth)-1:
            # Weighted display sample; independent deterministic display RNG.
            indices = np.linspace(0, len(pf.particles)-1, 1200).astype(int)
            frames.append(dict(index=i, time=i*.1, position=mean.tolist(),
                               covariance=cov.tolist(), heading=0,
                               particles=pf.particles[indices].tolist(), effective_count=effective))
    tail = min(100, len(truth))
    return dict(title='Particle filter', subtitle='Drone localization · height-map likelihood + displacement',
                seed=seed, dt=.1, times=(np.arange(len(truth))*.1).tolist(),
                truth=truth.tolist(), estimate=np.array(estimates).tolist(),
                measurements=[], baseline=[], errors=errors, frames=frames,
                height_map=surface[::10, ::10].T.tolist(), observations=observations,
                effective_counts=effective_counts,
                metrics=dict(particles=len(pf.particles), initial_error_m=errors[0],
                             final_error_m=errors[-1], last_100_rmse_m=rmse(estimates[-tail:], truth[-tail:])),
                bounds=[0, 8, 0, 15])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assignment', choices=['all', '1', '2', '3'], default='all')
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--output', type=Path, default=ROOT/'build')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    summary = dict(environment='Python / NumPy synthetic replay; no ROS or Gazebo', seed=args.seed, assignments={})
    for number, run in enumerate([run_kalman, run_ekf, run_particles], 1):
        if args.assignment not in ['all', str(number)]:
            continue
        data = run(args.seed)
        (args.output/f'assignment{number}.json').write_text(json.dumps(data, allow_nan=False))
        summary['assignments'][str(number)] = data['metrics']
        print(data['title'], json.dumps(data['metrics']), flush=True)
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')


if __name__ == '__main__':
    main()
