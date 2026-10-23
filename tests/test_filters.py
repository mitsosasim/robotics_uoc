import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from simulate import load_filter, run_kalman, run_ekf, run_particles

kf_module, ekf_module, pf_module = [load_filter(i) for i in (1, 2, 3)]


class KalmanChecks(unittest.TestCase):
    def test_scalar_posterior_in_two_dimensions(self):
        f = kf_module.KalmanFilter(position=(0, 0), covariance=np.eye(2), gps_std=2)
        f.update([5, -5])
        np.testing.assert_allclose(f.x, [1, -1])
        np.testing.assert_allclose(f.P, .8*np.eye(2))

    def test_straight_line_and_heading_uncertainty(self):
        f = kf_module.KalmanFilter()
        for _ in range(100):
            f.predict(np.pi/2, .1)
        np.testing.assert_allclose(f.x, [0, 3], atol=1e-12)
        self.assertGreater(np.linalg.eigvalsh(f.P).min(), 0)

    def test_bad_measurement_rejected(self):
        with self.assertRaises(ValueError):
            kf_module.KalmanFilter().update([np.nan, 1])


class ExtendedKalmanChecks(unittest.TestCase):
    def test_jacobian_against_finite_differences(self):
        for w in [0, 1e-10, -1e-10, .3, -.3]:
            x = np.array([.4, -.2, .7, .5, w])
            _, analytic = ekf_module.motion_and_jacobian(x, .1)
            numeric = np.zeros((5, 5))
            for j in range(5):
                delta = np.eye(5)[j]*1e-6
                high, _ = ekf_module.motion_and_jacobian(x+delta, .1)
                low, _ = ekf_module.motion_and_jacobian(x-delta, .1)
                numeric[:, j] = (high-low)/2e-6
            np.testing.assert_allclose(analytic, numeric, atol=1e-8)

    def test_noiseless_circle_and_zero_turn(self):
        for w in [.3, -.3, 0]:
            f = ekf_module.ExtendedKalmanFilter(state=[0, 0, 0, .5, w])
            for _ in range(200):
                f.predict(.1)
            expected = [.5/w*np.sin(w*20), .5/w*(1-np.cos(w*20))] if w else [10, 0]
            np.testing.assert_allclose(f.x[:2], expected, atol=1e-10)

    def test_covariance_stays_symmetric_positive(self):
        f = ekf_module.ExtendedKalmanFilter()
        rng = np.random.default_rng(4)
        for _ in range(100):
            f.predict(.1)
            f.update_gps(f.x[:2]+rng.normal(0, .25))
            f.update_velocity(.5+rng.normal(0, .03))
            f.update_velocity(.3+rng.normal(0, .03), angular=True)
            np.testing.assert_allclose(f.P, f.P.T, atol=1e-12)
            self.assertGreaterEqual(np.linalg.eigvalsh(f.P).min(), -1e-12)


class ParticleChecks(unittest.TestCase):
    def test_weight_normalization_and_systematic_resampling(self):
        f = pf_module.ParticleFilter(np.zeros((80, 150)), .1, count=1000)
        f.weights[:] = 0
        f.weights[:2] = [.9, .1]
        original = f.particles.copy()
        f.resample()
        self.assertEqual(np.sum(np.all(f.particles == original[0], axis=1)), 900)
        self.assertEqual(np.sum(np.all(f.particles == original[1], axis=1)), 100)
        np.testing.assert_allclose(f.weights.sum(), 1)

    def test_bounds_and_extreme_likelihood(self):
        f = pf_module.ParticleFilter(np.zeros((80, 150)), .1)
        f.predict([100, -100])
        self.assertTrue(np.all(f.particles >= 0))
        self.assertTrue(np.all(f.particles < f.extent))
        f.update(-1000, 1000)
        self.assertTrue(np.isfinite(f.weights).all())
        self.assertAlmostEqual(f.weights.sum(), 1)

    def test_course_height_map_helper_is_replayed_exactly(self):
        import types
        import xml.etree.ElementTree as ET
        root = Path(__file__).resolve().parents[1]
        world = root/'assignments/03_particle_filter/assign3/worlds/new_world.world'
        tree = ET.parse(world)
        models = tree.findall('./world/model')
        saved = {m.get('name'): m.findtext('pose') for m in tree.findall('./world/state/model')}
        states = types.SimpleNamespace(name=[], pose=[])
        for m in models:
            if m.get('name', '').startswith('unit_box'):
                x, y = map(float, saved.get(m.get('name'), m.findtext('pose')).split()[:2])
                states.name.append(m.get('name'))
                states.pose.append(types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y)))
        old_ros = pf_module.rospy
        try:
            pf_module.rospy = types.SimpleNamespace(wait_for_message=lambda *a, **kw: states)
            pf_module.ModelStates = object
            from_helper = pf_module.create_height_map(.01)
        finally:
            pf_module.rospy = old_ros
        np.testing.assert_array_equal(from_helper, pf_module.height_map_from_world(world))


class ReplayChecks(unittest.TestCase):
    def test_seeded_replays_against_truth(self):
        for run in [run_kalman, run_ekf]:
            metrics = run(7)['metrics']
            self.assertLess(metrics['estimate_rmse_m'], metrics['gps_rmse_m'])
            self.assertLess(metrics['estimate_rmse_m'], .15)
        result = run_particles(7)
        self.assertLess(result['metrics']['last_100_rmse_m'], .5)
        self.assertLess(result['metrics']['final_error_m'], .5)


if __name__ == '__main__':
    unittest.main()
