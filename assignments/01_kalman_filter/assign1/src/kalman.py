#!/usr/bin/env python3
"""Position KF with measured heading as an input; ROS imports are optional."""
import math
import threading
import numpy as np


class KalmanFilter:
    def __init__(self, position=(0., 0.), covariance=None, speed=0.3,
                 gps_std=0.3, yaw_std=0.01, speed_std=0.03):
        self.x = np.asarray(position, dtype=float).copy()
        self.P = np.eye(2) if covariance is None else np.array(covariance, dtype=float)
        self.speed = speed
        self.R = np.eye(2) * gps_std**2
        self.yaw_std, self.speed_std = yaw_std, speed_std

    def predict(self, yaw, dt):
        if not np.isfinite([yaw, dt]).all() or dt <= 0:
            raise ValueError('Heading must be finite and dt positive')
        c, s = math.cos(yaw), math.sin(yaw)
        self.x += self.speed * dt * np.array([c, s])
        # Propagate uncertainty in measured heading and commanded speed.
        J = dt * np.array([[c, -self.speed*s], [s, self.speed*c]])
        self.P += J @ np.diag([self.speed_std**2, self.yaw_std**2]) @ J.T
        return self.x.copy()

    def update(self, gps):
        z = np.asarray(gps, dtype=float)
        if z.shape != (2,) or not np.isfinite(z).all():
            raise ValueError('GPS must contain two finite coordinates')
        K = np.linalg.solve((self.P + self.R).T, self.P.T).T
        self.x += K @ (z - self.x)
        A = np.eye(2) - K
        self.P = A @ self.P @ A.T + K @ self.R @ K.T  # Joseph form
        self.P = (self.P + self.P.T) / 2
        return self.x.copy()


def main():
    import rospy
    from geometry_msgs.msg import PointStamped
    from nav_msgs.msg import Odometry

    rospy.init_node('kalman_filter')
    kf = KalmanFilter(speed=rospy.get_param('~speed', 0.3))
    pub = rospy.Publisher('/kalman_est', Odometry, queue_size=10)
    lock = threading.RLock()
    state = dict(yaw=None, stamp=None, initialized=False)

    def advance(stamp):
        if state['stamp'] is not None:
            dt = (stamp-state['stamp']).to_sec()
            if dt < 0:
                return False
            if dt > 1:
                rospy.logwarn_throttle(5, 'Time gap: prediction skipped')
            elif dt > 0 and state['yaw'] is not None:
                kf.predict(state['yaw'], dt)
        state['stamp'] = stamp
        return True

    def gps(msg):
        if not np.isfinite([msg.point.x, msg.point.y]).all():
            return
        with lock:
            stamp = msg.header.stamp if msg.header.stamp.to_sec() else rospy.Time.now()
            if not advance(stamp):
                return
            if not state['initialized']:
                kf.x[:] = [msg.point.x, msg.point.y]
                kf.P[:] = kf.R
                state['initialized'] = True
            else:
                kf.update([msg.point.x, msg.point.y])

    def yaw(msg):
        if np.isfinite(msg.point.z):
            with lock:
                state['yaw'] = msg.point.z

    def publish(_):
        with lock:
            if not state['initialized'] or state['yaw'] is None:
                return
            now = state['stamp']
            msg = Odometry()
            msg.header.stamp, msg.header.frame_id = now, 'odom'
            msg.child_frame_id = 'kalman_base'
            msg.pose.pose.position.x, msg.pose.pose.position.y = kf.x.tolist()
            msg.pose.pose.orientation.z = math.sin(state['yaw']/2)
            msg.pose.pose.orientation.w = math.cos(state['yaw']/2)
            cov = np.diag([1e6]*6)
            cov[:2, :2] = kf.P
            cov[5, 5] = kf.yaw_std**2
            msg.pose.covariance = cov.ravel().tolist()
            msg.twist.twist.linear.x = kf.speed
            pub.publish(msg)

    rospy.Subscriber('/measurement/gps', PointStamped, gps, queue_size=10)
    rospy.Subscriber('/measurement/yaw', PointStamped, yaw, queue_size=10)
    rospy.Timer(rospy.Duration(0.1), publish)
    rospy.spin()


if __name__ == '__main__':
    main()
