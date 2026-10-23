#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PointStamped
import random
import math

rospy.init_node('odometry_simulator', anonymous=True)

odom_pub = rospy.Publisher("/noisy_odom", Odometry, queue_size=1)

# Variables to store the robot's position and orientation
x = 0.0
y = 0.0
theta = 0.0  # Yaw angle (orientation)
linear_velocity = 0.3  # Constant speed in m/s

dt = 0.1  # 10 Hz update rate

def yaw_callback(msg):
    global theta
    theta = msg.point.z

def simulate_odometry():
    global x, y, theta

    dx = linear_velocity * math.cos(theta) * dt
    dy = linear_velocity * math.sin(theta) * dt

    x += dx
    y += dy

    # Create Odometry message to publish
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "odom"

    # Set the position and orientation
    odom_msg.pose.pose.position.x = x
    odom_msg.pose.pose.position.y = y
    odom_msg.pose.pose.position.z = 0

    odom_msg.pose.pose.orientation.x = 0
    odom_msg.pose.pose.orientation.y = 0
    odom_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
    odom_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

    odom_msg.twist.twist.linear.x = linear_velocity
    odom_msg.twist.twist.angular.z = 0

    odom_pub.publish(odom_msg)

def main():
    # Subscriber to receive yaw measurements
    rospy.Subscriber("/measurement/yaw", PointStamped, yaw_callback)
    rate = rospy.Rate(10)  # 10 Hz update rate

    while not rospy.is_shutdown():
        simulate_odometry()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
