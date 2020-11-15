#!/bin/sh
#export PATH="/opt/ros/kinetic/bin:/home/rtu/bin:/home/rtu/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin:/usr/local/cuda/bin:/home/rtu/tools/arduino-1.8.13:/opt/ros/kinetic/lib:/opt/ros/kinetic/share"
#export PYTHONPATH="/home/rtu/catkin_ws/devel/lib/python2.7/dist-packages:/opt/ros/kinetic/lib/python2.7/dist-packages"
#export ROS_PACKAGE_PATH="/home/rtu/catkin_ws/src:/opt/ros/kinetic/share"
#export ROS_ROOT="/opt/ros/kinetic/share/ros"
#export ROS_ETC_DIR="/opt/ros/kinetic/etc/ros/"
#export ROSLISP_PACKAGE_DIRECTORIES="/home/rtu/catkin_ws/devel/share/common-lisp/"

echo "$PATH"

echo "Start up! Wait 5 secs..."
#sleep 1
echo "Start!"

ros_add=192.168.1.123

count=0

#waiting for the wifi for 10 secs
while [ $count -le 9 ]
do
    echo "Waiting for wifi...$count"
    net="$(ifconfig | grep 192.168.1.123)"

    if [ -n "$net" ]; then
        count=10
    else
        count=$(($count+1))
    fi

    sleep 1
done

#pick the proper uri and hostname
if [ -n "$net" ]; then
    export ROS_MASTER_URI=http://${ros_add}:11311
    export ROS_HOSTNAME=${ros_add}
else
    export ROS_MASTER_URI=http://localhost:11311
    export ROS_HOSTNAME=localhost
fi

roslaunch sakura_mission awake.launch
