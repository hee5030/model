cmake_minimum_required(VERSION 3.0.2)
project(ip200_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    geometry_msgs
    std_msgs
)

add_message_files(
    FILES
    RobotMotor.msg
    ArucoMarker.msg
    ArucoMarkerVector.msg
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS
    geometry_msgs
    std_msgs
)