#!/usr/bin/env python3
import math
import numpy as np
from copy import copy
import random

from random import uniform

try:
    import rospy
    from gazebo_msgs.msg import ModelStates
    from geometry_msgs.msg import Pose, PoseArray
except ImportError:
    rospy = None  # Pure filter and offline demo do not require ROS.


#  DO NOT MODIFY- use these functions as your measurements #
# ---------------------------------------------------------#
error = 0.05
dt = 0.1  # Approximate time difference between measurements


def create_height_map(resolution):
    states = rospy.wait_for_message(
        "/gazebo/model_states", ModelStates, timeout=None)
    index_of_interest = []
    indx = 0
    for i in states.name:
        if i[0:8] == 'unit_box':
            index_of_interest.append(indx)
        indx += 1

    box_poses = []
    for i in index_of_interest:
        x = states.pose[i].position.x
        y = states.pose[i].position.y
        box_poses.append([x, y])

    dim_x = 8
    dim_y = 15
    height_map = np.zeros((int(dim_x/resolution), int(dim_y/resolution)))

    for pose in box_poses:
        x_i = pose[0]
        y_i = pose[1]
        for i in range(int((x_i-0.5)/resolution), int((x_i+0.5)/resolution)):
            for j in range(int((y_i-0.5)/resolution), int((y_i+0.5)/resolution)):
                dist_cent_x = (i*resolution - abs(x_i))
                dist_cent_y = (j*resolution - abs(y_i))
                height_map[i, j] = 1.0 - \
                    math.sqrt(dist_cent_x**2+dist_cent_y**2)

    return height_map

# Returns (dx dy)
def get_displacement(x, y):
    curr_pos = rospy.wait_for_message("/drone/gt_pose", Pose, timeout=None)
    return curr_pos.position.x-x + uniform(-error, error), curr_pos.position.y-y + uniform(-error, error)

# Returns the distance from the closest vertical object and the height of the drone above ground.
def find_vertical_distance(height_map, resolution):
    curr_pos = rospy.wait_for_message("/drone/gt_pose", Pose, timeout=None)
    x = int(curr_pos.position.x/resolution)
    y = int(curr_pos.position.y/resolution)
    z = curr_pos.position.z
    return z-height_map[x, y] + uniform(-error, error), z + uniform(-error, error)
# ----------------------------------------------------------------------------------------------------------------


def height_map_from_world(world_path, resolution=0.01):
    """Replay the supplied map helper from SDF model poses, without Gazebo.

    Saved SDF state poses override model definition poses. The grid indexing
    and abs(center) convention match the provided helper exactly.
    """
    import xml.etree.ElementTree as ET
    tree = ET.parse(world_path)
    saved = {m.get('name'): m.findtext('pose') for m in tree.findall('./world/state/model')}
    height_map = np.zeros((int(8/resolution), int(15/resolution)))
    for model in tree.findall('./world/model'):
        if not model.get('name', '').startswith('unit_box'):
            continue
        x_i, y_i = map(float, saved.get(model.get('name'), model.findtext('pose')).split()[:2])
        for i in range(int((x_i-0.5)/resolution), int((x_i+0.5)/resolution)):
            for j in range(int((y_i-0.5)/resolution), int((y_i+0.5)/resolution)):
                height_map[i, j] = 1.0-math.sqrt((i*resolution-abs(x_i))**2+(j*resolution-abs(y_i))**2)
    return height_map


class ParticleFilter:
    def __init__(self, height_map, resolution=0.01, count=12000, seed=7,
                 motion_std=0.035, measurement_std=0.05):
        if count < 2 or resolution <= 0:
            raise ValueError('Need at least two particles and positive resolution')
        self.map = np.asarray(height_map)
        self.resolution = resolution
        self.extent = np.array(self.map.shape)*resolution
        self.rng = np.random.default_rng(seed)
        self.particles = self.rng.uniform([0, 0], self.extent, (count, 2))
        self.weights = np.full(count, 1/count)
        self.motion_std, self.measurement_std = motion_std, measurement_std

    def heights(self, positions):
        indices = np.floor(np.asarray(positions)/self.resolution).astype(int)
        indices = np.clip(indices, 0, np.array(self.map.shape)-1)
        return self.map[indices[:, 0], indices[:, 1]]

    def predict(self, displacement):
        delta = np.asarray(displacement)
        if delta.shape != (2,) or not np.isfinite(delta).all():
            raise ValueError('Displacement must have two finite components')
        self.particles += delta+self.rng.normal(0, self.motion_std, self.particles.shape)
        invalid = np.any((self.particles < 0) | (self.particles >= self.extent), axis=1)
        self.particles[invalid] = self.rng.uniform([0, 0], self.extent, (invalid.sum(), 2))

    def update(self, range_to_surface, altitude):
        if not np.isfinite([range_to_surface, altitude]).all():
            raise ValueError('Range and altitude must be finite')
        observed_height = altitude-range_to_surface
        residual = (observed_height-self.heights(self.particles))/self.measurement_std
        log_weights = np.log(np.maximum(self.weights, 1e-300))-0.5*residual**2
        log_weights -= log_weights.max()
        self.weights = np.exp(log_weights)
        self.weights /= self.weights.sum()
        return self.estimate()

    def estimate(self):
        mean = self.weights @ self.particles
        d = self.particles-mean
        covariance = (d*self.weights[:, None]).T @ d
        return mean.copy(), covariance

    def effective_count(self):
        return float(1/np.sum(self.weights**2))

    def resample(self):
        # Low-variance/systematic resampling: one random offset, N equal steps.
        n = len(self.weights)
        points = (self.rng.random()+np.arange(n))/n
        cdf = np.cumsum(self.weights)
        cdf[-1] = 1.0
        indices = np.searchsorted(cdf, points, side='right')
        self.particles = self.particles[indices].copy()
        self.weights.fill(1/n)

    def step(self, displacement, range_to_surface, altitude):
        self.predict(displacement)
        mean, covariance = self.update(range_to_surface, altitude)
        effective = self.effective_count()
        if effective < 0.5*len(self.weights):
            self.resample()
        return mean, covariance, effective


def main():
    if rospy is None:
        raise SystemExit('ROS is required for the node; use tools/simulate.py for the offline demo.')
    rospy.init_node('particle_filter')
    resolution = 0.01
    surface = create_height_map(resolution)
    pf = ParticleFilter(surface, resolution, count=rospy.get_param('~count', 12000),
                        seed=rospy.get_param('~seed', 7))
    pub = rospy.Publisher('/particle_filter', PoseArray, queue_size=1)
    rate = rospy.Rate(10)
    odom_x = odom_y = 0.0
    while not rospy.is_shutdown():
        # Only the three provided helper interfaces supply sensor information.
        dx, dy = get_displacement(odom_x, odom_y)
        odom_x, odom_y = odom_x+dx, odom_y+dy
        distance, altitude = find_vertical_distance(surface, resolution)
        mean, covariance, effective = pf.step([dx, dy], distance, altitude)
        msg = PoseArray()
        msg.header.frame_id, msg.header.stamp = 'map', rospy.Time.now()
        for x, y in pf.particles:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = float(x), float(y), 1.0
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        pub.publish(msg)
        rospy.loginfo_throttle(2, 'PF estimate (%.2f, %.2f), Neff=%.0f', mean[0], mean[1], effective)
        rate.sleep()


if __name__ == '__main__':
    main()
