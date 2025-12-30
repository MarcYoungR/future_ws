1. roslaunch livox_ros_driver2 msg_MID360.launch   启动livox驱动
2. roslaunch mavros px4.launch   启动mavros
3. rosrun mavros mavcmd long 511 31 10000 0 0 0 0 0   修改imu频率
4. 在future_ws文件夹下，roslaunch fast_lio mapping_mid360.launch   启动fast_lio
5. 在Px4ctrl文件夹下，roslaunch px4ctrl run_ctrl.launch   启动px4控制器
6. roslaunch mission_planner click_mission.launch   启动SUPER Planner（click_mission.launch为有rviz可视化）
7. 运行roslaunch obstacle_detection realsense_detection.launch \
camera_type:=mono \
color_topic:=/camera/color/image_raw \
camera_info_topic:=/camera/color/camera_info \
depth_topic:=/camera/depth/image_rect_raw \
depth_timeout:=0.5 \
capture_dir:=captures
图像检测节点
8. 在/home/rose/future_ws/src/SUPER/super_planner/scripts 下运行打点飞行程序
运行完此节点，马上切换offboard并解锁。


- 注意SUPER Planner的参数修改的是click_smooth_ros1.yaml
- 注意Px4ctrl的参数修改的是ctrl_param_fpv.yaml

