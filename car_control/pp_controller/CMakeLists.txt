cmake_minimum_required(VERSION 3.0.2)
project(pp_controller)
set(CMAKE_CXX_STANDARD 14)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  common

)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES traj_generation
#  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport nav_msgs roscpp rospy std_msgs tf
#  DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME} src/pp_follower.cpp )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} )


