cmake_minimum_required(VERSION 3.0.2)
project(ip200_motor_driver)

find_package(catkin REQUIRED COMPONENTS
  can_msgs
  roscpp
  std_msgs
  std_srvs
  ip200_msgs
)

catkin_package(
 LIBRARIES ip200_motor_driver
 CATKIN_DEPENDS roscpp std_msgs can_msgs 
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

add_executable(ip200_motor_driver src/can_handler.cpp src/motor_driver.cpp)
target_link_libraries(ip200_motor_driver ${catkin_LIBRARIES} )
