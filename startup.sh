#!/bin/bash

ros_add=192.168.0.193 

count=0

#waiting for the wifi for 10 secs
while [ $count -le 9 ]
do
    echo "Waiting for wifi...$count"
    net="$(ifconfig | grep $ros_add)"

    if [ -n "$net" ]; then
        count=10
    else
        count=$(($count+1))
    fi

    sleep 1
done

echo "nano" | sudo -S sh -c 'echo 100 > /sys/devices/pwm-fan/target_pwm'

#pick the proper uri and hostname
if [ -n "$net" ]; then
    export ROS_MASTER_URI=http://${ros_add}:11311
    export ROS_HOSTNAME=${ros_add}
    roslaunch sakura_mission awake_control.launch
else
    export ROS_MASTER_URI=http://localhost:11311
    export ROS_HOSTNAME=localhost
    roslaunch sakura_mission awake_auto.launch
fi


