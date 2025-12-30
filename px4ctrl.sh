#!/bin/bash
# 该脚本用于启动mavros和fastlio，同时修改imu频率

#export DISPLAY=:0

#xhost +

gnome-terminal --tab -- bash -c "source /home/rose/Px4ctrl/devel/setup.bash; roslaunch px4ctrl run_ctrl.launch; exec bash"
