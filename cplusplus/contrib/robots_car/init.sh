#!/bin/bash

set -e

current_path=$(cd `dirname $0`; pwd)

# 1.配置realsense
[ ! -e $current_path/librealsense/build ] && mkdir $current_path/librealsense/build
cd $current_path/librealsense/build && rm -rf ./*
apt-get install -y libudev-dev pkg-config libgtk-3-dev libusb-1.0-0-dev libglfw3-dev libssl-dev ros-melodic-realsense2-camera ros-melodic-tf2-sensor-msgs can-utils
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
cmake ../ -DEBUG_EXAMPLES=true
make -j12 && make install

# 2.安装依赖
cd $current_path
rosdep install --from-path src --ignore-src -y -r

# 3.compile
catkin_make
rosdep install --from-path src --ignore-src -y -r

# 4.config env
echo "source ~/waic/devel/setup.bash" >> ~/.bashrc
echo "export ROS_IP=192.168.1.10" >> ~/.bashrc
echo 'export ROS_HOSTNAME=${ROS_IP}' >> ~/.bashrc
echo 'export ROS_MASTER_URI=http://${ROS_IP}:11311' >> ~/.bashrc

