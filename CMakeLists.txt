cmake_minimum_required(VERSION 3.0.2)
project(mushr_gprm)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf2
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
