#!/bin/bash

echo "Starting the system ..."
sleep 5s
gnome-terminal -t "start_robot_base" -x bash -c "cd ~/waic && roslaunch run_noah_base.launch;exec bash"
sleep 15s
gnome-terminal -t "start_navigation" -x bash -c "source /opt/ros/melodic/setup.bash && source ~/waic/devel/setup.bash && export ROS_HOSTNAME=192.168.1.10 && export ROS_MASTER_URI=http://192.168.1.10:11311 && cd ~/waic && roslaunch run_noah_navigation.launch;exec bash"

gnome-terminal -t "start_rviz" -x bash -c "source /opt/ros/melodic/setup.bash && source ~/waic/devel/setup.bash && export ROS_HOSTNAME=192.168.1.10 && export ROS_MASTER_URI=http://192.168.1.10:11311 && cd ~/waic && rviz -d rviz.rviz;exec bash"
gnome-terminal -t "start_nav" -x bash -c "source /opt/ros/melodic/setup.bash && source ~/waic/devel/setup.bash && export ROS_HOSTNAME=192.168.1.10 && export ROS_MASTER_URI=http://192.168.1.10:11311 && python ~/waic/nav.py;exec bash"

echo "OK ..."
