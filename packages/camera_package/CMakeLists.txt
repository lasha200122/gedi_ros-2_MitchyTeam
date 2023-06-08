cmake_minimum_required(VERSION 2.8.3)
project(camera_package)

find_package(catkin REQUIRED COMPONENTS
	rospy
	roscpp
  	duckietown_msgs
  	sensor_msgs
)

catkin_package()
