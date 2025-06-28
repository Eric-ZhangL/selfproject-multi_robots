cmake_minimum_required(VERSION 3.0.2)
project(record_path)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

find_package(Threads REQUIRED)
generate_messages(DEPENDENCIES
  std_msgs
)
#find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
#link_directories(${PCL_LIBRARY_DIRS})

add_executable(key_op src/key_op.cpp)
add_dependencies(key_op ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(key_op ${catkin_LIBRARIES})

add_executable(record src/record.cpp)
add_dependencies(record ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(record ${catkin_LIBRARIES})
