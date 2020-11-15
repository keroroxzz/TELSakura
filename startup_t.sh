#!/bin/sh
gnome-terminal --working-directory='/home/nano' -x bash -c "source /opt/ros/melodic/setup.bash; source /home/nano/catkin_ws/devel/setup.bash; source /home/nano/.bashrc; /home/nano/catkin_ws/src/SAKURA/TELSakura/startup.sh; exec bash"

