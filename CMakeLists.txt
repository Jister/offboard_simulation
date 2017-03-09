cmake_minimum_required(VERSION 2.8.3)
project(offboard_simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(offboard_node src/offb_node.cpp)

target_link_libraries(offboard_node
  ${catkin_LIBRARIES}
)

link_directories(lib)

add_executable(prp src/read_xl.cpp)

target_link_libraries(prp
  ${catkin_LIBRARIES}
  libxl.so
)

