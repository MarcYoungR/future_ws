#!/bin/bash
# 该脚本用于启动mavros和fastlio，同时修改imu频率

#export DISPLAY=:0

#xhost +

gnome-terminal --tab -- bash -c "roslaunch livox_ros_driver2 msg_MID360.launch; exec bash"

sleep 3

gnome-terminal --tab -- bash -c "roslaunch mavros px4.launch; exec bash"

sleep 3

gnome-terminal --tab -- bash -c "rosrun mavros mavcmd long 511 31 10000 0 0 0 0 0; exec bash"

sleep 3

gnome-terminal --tab -- bash -c "roslaunch fast_lio mapping_mid360.launch; exec bash"
