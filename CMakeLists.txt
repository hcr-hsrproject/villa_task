cmake_minimum_required(VERSION 2.8.3)
project(villa_task)

find_package(catkin REQUIRED COMPONENTS
  rospy
  villa_sound_localization
  villa_helpers
  villa_manipulation
  std_msgs
)


catkin_python_setup()

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
