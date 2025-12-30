#!/bin/bash
# 该脚本用于启动mavros和fastlio

export DISPLAY=:0

xhost +

gnome-terminal --tab -- bash -c "roslaunch livox_ros_driver2 msg_MID360.launch; exec bash"

sleep 1

gnome-terminal --tab -- bash -c "source ../../../devel/setup.bash; roslaunch fast_lio mavros_with_fastlio_px4ctrl.launch; exec bash"

sleep 3

gnome-terminal --tab -- bash -c "rosrun mavros mavcmd long 511 31 10000 0 0 0 0 0; exec bash"