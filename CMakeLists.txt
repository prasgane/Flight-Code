cmake_minimum_required(VERSION 2.8.3)
project(flight_code)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(include)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(flight_node
              src/flight_code_node.cpp
              src/flight_code.cpp)

target_link_libraries(flight_node
                ${catkin_LIBRARIES}
                )
