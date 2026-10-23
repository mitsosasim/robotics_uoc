#!/usr/bin/env python3
import rospy
import random
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

rospy.init_node('gps_simulator', anonymous=True)

gps_pub = rospy.Publisher("/measurement/gps", PointStamped, queue_size=1)
yaw_pub = rospy.Publisher("/measurement/yaw", PointStamped, queue_size=1)
cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=True)


def odom_callback(msg):

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    noise_x = random.gauss(0, 0.3)
    noise_y = random.gauss(0, 0.3)

    noisy_x = x + noise_x
    noisy_y = y + noise_y

    # Publish noisy GPS measurement
    gps_viz_msg = PointStamped()
    gps_viz_msg.header.stamp = rospy.Time.now()
    gps_viz_msg.header.frame_id = "odom"
    gps_viz_msg.point.x = noisy_x
    gps_viz_msg.point.y = noisy_y
    gps_viz_msg.point.z = 0
    gps_pub.publish(gps_viz_msg)

def imu_callback(msg):

    orientation_q = msg.orientation

    _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])


    # Publish noisy yaw measurement
    yaw_viz_msg = PointStamped()
    yaw_viz_msg.header.stamp = rospy.Time.now()
    yaw_viz_msg.header.frame_id = "odom"
    yaw_viz_msg.point.x = 0
    yaw_viz_msg.point.y = 0
    yaw_viz_msg.point.z = yaw + (abs(random.gauss(0, 0.01))) #0.01 rad
    yaw_pub.publish(yaw_viz_msg)

# Subscribers
rospy.Subscriber("/odom", Odometry, odom_callback)
rospy.Subscriber("/imu", Imu, imu_callback)

# Command velocity to move the robot
msg = Twist()
msg.linear.x = 0.3  # Move robot forward at 0.3 m/s
msg.linear.y = 0.
msg.linear.z = 0.
msg.angular.x = 0.
msg.angular.y = 0.
msg.angular.z = 0.

rate = rospy.Rate(10)  # 10 Hz
while cmd_pub.get_num_connections() == 0:
    rate.sleep()
cmd_pub.publish(msg)

rospy.spin()
