cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_teleop)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs cv_bridge)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES opencv
  CATKIN_DEPENDS roscpp geometry_msgs cv_bridge
#  DEPENDS
)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(turtlebot_teleop src/optical_flow.cpp)
target_link_libraries(turtlebot_teleop ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

