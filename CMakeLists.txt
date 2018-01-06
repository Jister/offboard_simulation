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


#add_executable(offboard_node src/offb_node.cpp)

#target_link_libraries(offboard_node
#  ${catkin_LIBRARIES}
#)

add_executable(offb_test src/test.cpp)
target_link_libraries(offb_test
  ${catkin_LIBRARIES}
)

link_directories(lib)

add_executable(prp src/read_xl.cpp)

target_link_libraries(prp
  ${catkin_LIBRARIES}
  libxl.so
)

add_executable(fly_xl src/fly_xl.cpp)

target_link_libraries(fly_xl
  ${catkin_LIBRARIES}
  libxl.so
)

add_executable(obstacle src/obstacle_avoid.cpp)

target_link_libraries(obstacle
  ${catkin_LIBRARIES}
)

add_executable(fly_obstacle src/fly_obstacle.cpp)

target_link_libraries(fly_obstacle
  ${catkin_LIBRARIES}
)


add_executable(off_test src/off_test.cpp)

target_link_libraries(off_test
  ${catkin_LIBRARIES}
)

add_executable(att_interp src/att_interp.cpp)

target_link_libraries(att_interp
  ${catkin_LIBRARIES}
  libxl.so
)
