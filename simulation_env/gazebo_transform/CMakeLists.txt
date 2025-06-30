cmake_minimum_required(VERSION 3.0.2)
project(gazebo_transform)

#add_compile_options(-std=c++11)
#add_compile_options(-std=gnu++11)
#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE Release)
#set(BUILD_STATIC_LIBS ON)
#set(BUILD_SHARED_LIBS OFF)



find_package(catkin REQUIRED COMPONENTS
#pcl_ros
rospy
#ugv_bit
roscpp
tf
geometry_msgs
#message_generation

# autoware_msgs

#sensor_msgs
)

find_package(Threads REQUIRED)
#find_package(PCL REQUIRED)

catkin_package(
CATKIN_DEPENDS
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
)


include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(gazebo_transform src/sub_pub.cpp)
target_link_libraries(gazebo_transform ${catkin_LIBRARIES} Threads::Threads)
add_executable(gazebo_transform_backup src/sub_pub.cpp)
target_link_libraries(gazebo_transform_backup ${catkin_LIBRARIES} Threads::Threads)
#add_executable(test_node src/test.cpp)
#target_link_libraries(test_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} Threads::Threads)
