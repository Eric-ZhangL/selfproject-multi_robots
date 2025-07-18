cmake_minimum_required(VERSION 3.0.2)
#SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")
SET(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
project(gps)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  serial
  message_generation
  dynamic_reconfigure
  tf
  rospy
  common
  gpx
)

find_package(Threads REQUIRED)


add_message_files(
   FILES
   MyGPS_msg.msg
 )


generate_messages(DEPENDENCIES
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gpxdata
  CATKIN_DEPENDS roscpp std_msgs message_runtime common
#  DEPENDS system_lib
)


include_directories(
 include 
 ${catkin_INCLUDE_DIRS}
)


add_executable(gps_base
  src/gps_base.cpp
)
add_dependencies(gps_base ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_base
  ${catkin_LIBRARIES}
)

add_executable(gps_pro
  src/gps_pro.cpp
)
add_dependencies(gps_pro ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_pro
  ${catkin_LIBRARIES}
)

add_executable(gps_base_backup
  src/gps_base_backup.cpp
)
add_dependencies(gps_base_backup ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_base_backup
  ${catkin_LIBRARIES}
)

add_executable(gps_pro_backup
  src/gps_pro_backup.cpp
)
add_dependencies(gps_pro_backup ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_pro_backup
  ${catkin_LIBRARIES}
)





