cmake_minimum_required(VERSION 3.0.2)
project(assign2)
find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs nav_msgs sensor_msgs std_msgs tf gazebo_msgs)
catkin_package()
catkin_install_python(PROGRAMS src/ekf.py scripts/navigation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
