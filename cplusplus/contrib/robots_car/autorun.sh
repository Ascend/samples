#!/bin/bash

echo "Starting the system ..."
sleep 5s
gnome-terminal -t "start_robot_base" -x bash -c "source /opt/ros/melodic/setup.bash && source ~/waic/devel/setup.bash && export ROS_HOSTNAME=192.168.1.10 && export ROS_MASTER_URI=http://192.168.1.10:11311 && cd ~/waic && roslaunch run_noah_base.launch;exec bash"

sleep 20s
gnome-terminal -t "start_navigation" -x bash -c "source /opt/ros/melodic/setup.bash && source ~/waic/devel/setup.bash && export ROS_HOSTNAME=192.168.1.10 && export ROS_MASTER_URI=http://192.168.1.10:11311 && cd ~/waic && roslaunch run_noah_navigation.launch;exec bash"
gnome-terminal -t "start_atlas_detect" -x bash -c "echo 'bash autostart_detect.sh' | sshpass -p "Mind@123" ssh HwHiAiUser@192.168.1.111;exec bash"
gnome-terminal -t "start_atlas_text" -x bash -c "echo 'bash autostart_text.sh' | sshpass -p "Mind@123" ssh HwHiAiUser@192.168.1.111;exec bash"
gnome-terminal -t "start_publish_res" -x bash -c "source /opt/ros/melodic/setup.bash && source ~/waic/devel/setup.bash && export ROS_HOSTNAME=192.168.1.10 && export ROS_MASTER_URI=http://192.168.1.10:11311 && python ~/waic/detector_node_v2.py;exec bash"

sleep 15s

gnome-terminal -t "start_rviz" -x bash -c "source /opt/ros/melodic/setup.bash && source ~/waic/devel/setup.bash && export ROS_HOSTNAME=192.168.1.10 && export ROS_MASTER_URI=http://192.168.1.10:11311 && cd ~/waic && rviz -d rviz.rviz;exec bash"
gnome-terminal -t "start_nav" -x bash -c "source /opt/ros/melodic/setup.bash && source ~/waic/devel/setup.bash && export ROS_HOSTNAME=192.168.1.10 && export ROS_MASTER_URI=http://192.168.1.10:11311 && python ~/waic/nav.py;exec bash"

echo "OK ..."
