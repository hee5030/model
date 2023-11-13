# model



## Dependencies
* https://github.com/ros-planning/navigation
* https://github.com/mats-robotics/yolov5_ros
* https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy

* pip install opencv-python==4.6.0.66
* pip install opencv-contrib-python==4.7.0.72

## Add packages
* ros 세팅방법은 아래와 같이 명령어를 terminal에서 차례대로 입력합니다.
* sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
* sudo apt install curl # if you haven't already installed curl
* curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
* sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
* sudo apt-get update && sudo apt-get upgrade -y
* sudo apt-get install ros-noetic-desktop-full
* sudo apt-get install ros-kinetic-rqt*
* sudo apt-get install python-rosdep
* sudo rosdep init
* rosdep update
* sudo apt-get install python-rosinstall
* source /opt/ros/kinetic/setup.bash
* mkdir -p ~/catkin_ws/src
* cd ~/catkin_ws/src
* catkin_init_workspace
* cd ~/catkin_ws/
* catkin_make
*
* --추가로 로봇을 움직이기 위해 필요한 패키지들--
* sudo apt install ros-noetic-can-msgs
* sudo apt install ros-noetic-map-server
* sudo apt insatll ros-noetic-tf2-sensor-msgs
* sudo apt install ros-noetic-move-base-msgs
* sudo apt install ros-noetic-socketcan-bridge
