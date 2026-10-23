#!/usr/bin/env python3
"""Five-state constant-turn-rate EKF with an analytic motion Jacobian."""
import math
import threading
import numpy as np


def wrap(angle):
    return (angle + np.pi) % (2*np.pi) - np.pi


def motion_and_jacobian(state, dt):
    """Exact arc integration, continuous at zero angular velocity."""
    x = np.array(state, dtype=float)
    theta, v, w = x[2:]
    u = w*dt/2
    if abs(u) < 1e-4:
        sinc = 1-u*u/6+u**4/120
        dsinc = -u/3+u**3/30-u**5/840
    else:
        sinc = math.sin(u)/u
        dsinc = (u*math.cos(u)-math.sin(u))/(u*u)
    scale, dscale = dt*sinc, dt*dt/2*dsinc
    c, s = math.cos(theta+u), math.sin(theta+u)
    dx, dy = v*scale*c, v*scale*s
    G = np.eye(5)
    G[0, 2], G[1, 2] = -dy, dx
    G[0, 3], G[1, 3] = scale*c, scale*s
    G[0, 4] = v*(dscale*c-scale*s*dt/2)
    G[1, 4] = v*(dscale*s+scale*c*dt/2)
    G[2, 4] = dt
    x[:3] += [dx, dy, w*dt]
    x[2] = wrap(x[2])
    return x, G


class ExtendedKalmanFilter:
    def __init__(self, state=(0., 0., 0., 0.5, 0.3), covariance=None):
        self.x = np.array(state, dtype=float)
        self.P = np.diag([0.25**2]*2+[0.05**2, 0.03**2, 0.03**2]) if covariance is None else np.array(covariance, dtype=float)

    def predict(self, dt):
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be finite and positive')
        theta = self.x[2]
        self.x, G = motion_and_jacobian(self.x, dt)
        L = np.array([[0.5*dt**2*math.cos(theta), 0],
                      [0.5*dt**2*math.sin(theta), 0],
                      [0, 0.5*dt**2], [dt, 0], [0, dt]])
        Q = L @ np.diag([0.08**2, 0.06**2]) @ L.T + np.eye(5)*1e-9*dt
        self.P = G @ self.P @ G.T + Q
        self.P = (self.P+self.P.T)/2
        return self.x.copy()

    def update(self, measurement, indices, covariance):
        z, R = np.atleast_1d(measurement).astype(float), np.atleast_2d(covariance).astype(float)
        if not np.isfinite(z).all():
            raise ValueError('Measurements must be finite')
        H = np.eye(5)[list(indices)]
        S = H @ self.P @ H.T + R
        K = np.linalg.solve(S.T, (self.P @ H.T).T).T
        self.x += K @ (z-H@self.x)
        self.x[2] = wrap(self.x[2])
        A = np.eye(5)-K@H
        self.P = A@self.P@A.T+K@R@K.T
        self.P = (self.P+self.P.T)/2
        return self.x.copy()

    def update_gps(self, xy, std=0.25, correlation=1.0):
        # Supplied simulator adds the same Gaussian sample to x and y.
        R = std**2*np.array([[1., correlation], [correlation, 1.]])+np.eye(2)*1e-9
        return self.update(xy, [0, 1], R)

    def update_velocity(self, value, angular=False, std=0.03):
        return self.update([value], [4 if angular else 3], [[std**2]])


def main():
    import rospy
    from geometry_msgs.msg import PointStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float64
    from tf.transformations import euler_from_quaternion

    rospy.init_node('extended_kalman_filter')
    ekf, lock = ExtendedKalmanFilter(), threading.RLock()
    state = dict(initialized=False, stamp=None, pending={})
    pub = rospy.Publisher('/ekf', Odometry, queue_size=10)

    def advance(stamp):
        if state['stamp'] is not None:
            dt = (stamp-state['stamp']).to_sec()
            if dt < 0:
                return False
            if 0 < dt <= 1:
                ekf.predict(dt)
            elif dt > 1:
                rospy.logwarn_throttle(5, 'Time gap: prediction skipped')
        state['stamp'] = stamp
        return True

    def initialize(msg):
        with lock:
            if state['initialized']:
                return
            q = msg.pose.pose.orientation
            theta = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            ekf.x[:] = [msg.pose.pose.position.x, msg.pose.pose.position.y, theta,
                        msg.twist.twist.linear.x, msg.twist.twist.angular.z]
            state.update(initialized=True, stamp=msg.header.stamp)

    def gps(msg):
        if not np.isfinite([msg.point.x, msg.point.y]).all():
            return
        with lock:
            state['pending']['gps'] = ([msg.point.x, msg.point.y], msg.header.stamp)

    def velocity(msg, angular):
        if not np.isfinite(msg.data):
            return
        with lock:
            state['pending']['angular' if angular else 'linear'] = (msg.data, rospy.Time.now())

    def publish(_):
        with lock:
            now = rospy.Time.now()
            if not state['initialized'] or not advance(now):
                return
            # Assimilate each new sample once at the 10 Hz fusion tick.
            for channel, (value, stamp) in list(state['pending'].items()):
                age = (now-stamp).to_sec()
                if age < 0:
                    continue
                del state['pending'][channel]
                if age > 0.5:
                    continue
                if channel == 'gps':
                    ekf.update_gps(value, correlation=rospy.get_param('~gps_correlation', 1.0))
                else:
                    ekf.update_velocity(value, angular=(channel == 'angular'))
            msg = Odometry()
            msg.header.stamp, msg.header.frame_id = now, 'odom'
            msg.child_frame_id = 'ekf_base'
            msg.pose.pose.position.x, msg.pose.pose.position.y = ekf.x[:2].tolist()
            msg.pose.pose.orientation.z = math.sin(ekf.x[2]/2)
            msg.pose.pose.orientation.w = math.cos(ekf.x[2]/2)
            cov = np.diag([1e6]*6)
            cov[np.ix_([0, 1, 5], [0, 1, 5])] = ekf.P[:3, :3]
            msg.pose.covariance = cov.ravel().tolist()
            msg.twist.twist.linear.x, msg.twist.twist.angular.z = ekf.x[3:].tolist()
            twist = np.diag([1e6]*6)
            twist[np.ix_([0, 5], [0, 5])] = ekf.P[3:, 3:]
            msg.twist.covariance = twist.ravel().tolist()
            pub.publish(msg)

    rospy.Subscriber('/odom', Odometry, initialize, queue_size=1)
    rospy.Subscriber('/measurement/gps', PointStamped, gps, queue_size=10)
    rospy.Subscriber('/measurement/linear_velocity', Float64, velocity, callback_args=False, queue_size=10)
    rospy.Subscriber('/measurement/angular_velocity', Float64, velocity, callback_args=True, queue_size=10)
    rospy.Timer(rospy.Duration(0.1), publish)
    rospy.spin()


if __name__ == '__main__':
    main()
