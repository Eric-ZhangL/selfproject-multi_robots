cmake_minimum_required(VERSION 3.0.2)
project(imu)


find_package(catkin REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )


# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
)



add_executable(imu
  src/imu.cpp
)
add_dependencies(imu ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu
  ${catkin_LIBRARIES}
)
