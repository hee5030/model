cmake_minimum_required(VERSION 3.0.2)
project(ip200_core)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  nav_msgs
  ip200_msgs
)

catkin_package(
 LIBRARIES ip200_core
 CATKIN_DEPENDS roscpp std_msgs tf nav_msgs
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

add_executable(ip200_driver src/ip200_driver.cpp)
target_link_libraries(ip200_driver ${catkin_LIBRARIES} )
