# ip200_ros
ROS packages for AI:DL ip200_ros


# 디펜던시
- ros-canopen
- socketcan

# 설치

## PCAN 설치

### 1 pcan-linux-installation

드라이버 설치
https://www.peak-system.com/
Home: PEAK-System  Support -> Download -> Linux 에서 드라이버 Linux PCAN Driver: Overview  설치로 이동


### 2 터미널을 사용해 HOME 디렉터리에서 압축을 해제한다. 


~~~
mv peak-linux-driver-8.13.0.tar.gz ~/
cd
tar xvzf peak-linux-driver-8.13.0.tar.gz
~~~ 


### 3 의존 라이브러리를 설치한다.
~~~ 
sudo apt install -y libpopt-dev libelf-dev gcc-multilib can-utils
~~~

### 4 설치

~~~ 
cd peak-linux-driver-8.13.0
make clean
make NET=NETDEV_SUPPORT
sudo make install
~~~


### 5 설치 확인

~~~
sudo modprobe pcan
sudo ip link set can0 type can
sudo ip link set can0 up type can bitrate 125000 # configure the CAN bitrate, f.e. 125000 bit/s

./driver/lspcan --all       # should print "pcanusb32" and pcan version
tree /dev/pcan-usb          # should show a pcan-usb device
ip -a link                  # should print some "can0: ..." messages
ip -details link show can0  # should print some details about "can0" net device
~~~ 

## ip200 로봇 셋업

### 0 의존라이브러리 설치
~~~
sudo apt update
sudo apt install ros-$ROS_DISTRO-canopen-*
sudo apt install ros-$ROS_DISTRO-socketcan-*
sudo apt install ros-$ROS_DISTRO-navigation
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard
sudo apt install ros-$ROS_DISTRO-teleop-twist-joy
~~~

### 1 git 레포지터리 클론 
~~~
cd ~/catkin_ws/src
git clone https://github.com/AIDL-LAB/IP200_ROS

catkin_make
~~~

### 2 실행

2.1.CAN 포트 설정
~~~
sudo modprobe pcan     #PEAK vendor driver
sudo ip link set can0 up type can bitrate 500000  # Initialize NIC
~~~
2.2.CAN 브릿지 설정, motor_driver, 로봇 드라이버 실행

~~~
roslaunch ip200_core ip200_core.launch
~~~
2.3.로봇 조종 패키지 실행
로봇의 PC와 컨트롤러(PS4 듀얼쇼크)가 블루투스로 연결된 상태로 실행한다.

~~~
roslaunch ip200_teleop teleop.launch 
~~~

