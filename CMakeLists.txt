cmake_minimum_required(VERSION 3.9)
project(rviz_display_interface)

set(CMAKE_CXX_STANDARD 11)
set(SGBOT_INCLUDE_DIR /root/catkin_ws/src/libsgbot/include/)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        geometry_msgs
        std_msgs
        )
find_package(Eigen3 REQUIRED)

include_directories(EIGEN3_INCLUDE_DIR)
#link_directories(${catkin_LIBRARY_DIRS})
catkin_package(
    CATKIN_DEPENDS
        roscpp
        geometry_msgs
)
include_directories(${catkin_INCLUDE_DIRS} ${SGBOT_INCLUDE_DIR})

add_executable(rviz_display_interface main.cpp DisplayInterface.cpp)
target_link_libraries(rviz_display_interface ${catkin_LIBRARIES} sgbot)