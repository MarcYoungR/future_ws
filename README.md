Autonomous UAV System (FAST-LIO + Px4Ctrl + SUPER)
1. 项目简介 | Project Overview
本项目是一个集成了定位、控制与规划的全自主无人机系统。系统架构主要包含以下三个核心模块：

定位与建图 (Localization & Mapping): 采用 FAST-LIO2 方案，基于 Livox MID360 激光雷达提供高精度的里程计与环境点云。

底层控制 (Control): 采用高飞老师团队开源的 Px4Ctrl 控制器，提供稳定的轨迹跟踪控制。

避障与规划 (Planning): 采用 MARS 实验室的 SUPER Planner，实现动态环境下的避障与路径规划。

交互方式： 用户只需通过 Rviz 或代码逻辑向系统发送 /goal 目标点话题，无人机即可自动规划路径并飞行至目标位置。

2. 硬件与驱动准备 | Prerequisites
在启动系统前，请确保以下硬件驱动已正确安装并调试完毕：

2.1 Livox MID360 驱动
需要预先安装 livox_ros_driver2。

注意： 请参考本项目文档中的 [雷达部署部分] 进行网络配置和 SDK 编译，确保能够正常读取点云数据。

2.2 Mavros 通信
确保机载电脑与飞控（Pixhawk/PX4）通过串口或 USB 连接正常。

确保 mavros 包已安装，且波特率与飞控设置一致。

3. 参数配置指南 | Configuration
在使用 FAST-LIO 进行定位前，必须根据你的无人机实际结构修改配置文件。

FAST-LIO 配置文件
文件路径： src/FAST_LIO/config/mid360.yaml (假设使用的是 mid360 的配置)

修改内容 (Extrinsic Parameter)： 你需要修改雷达（LiDAR）相对于飞控 IMU 的外参（旋转矩阵 common.R_L_I 和 平移向量 common.t_L_I）。

如果雷达安装位置发生变化，务必重新测量并更新这两个参数，否则会导致里程计漂移或控制发散。

其他模块参数
SUPER Planner: 修改 src/SUPER/super_planner/config/click_smooth_ros1.yaml 以调整规划器的速度、加速度限制及避障策略。

Px4Ctrl: 修改 src/Px4ctrl/config/ctrl_param_fpv.yaml 以调整 PID 增益及动力学参数。

4. 启动流程 | Quick Start
请按照以下顺序在终端中启动各个模块。

Step 1: 启动雷达驱动
Bash

roslaunch livox_ros_driver2 msg_MID360.launch
检查：确保有点云数据输出。

Step 2: 启动 Mavros 通信
Bash

roslaunch mavros px4.launch
检查：确保能连接到 FCU (Flight Control Unit)。

Step 3: 修改 IMU 频率 (关键步骤)
为了满足 FAST-LIO 对高频 IMU 数据的需求，需要通过 mavcmd 修改飞控的 IMU 发送频率。

Bash

rosrun mavros mavcmd long 511 31 10000 0 0 0 0
说明：此命令将 IMU 频率设置为高频模式。

Step 4: 启动 FAST-LIO 里程计
Bash

# 请确保在 workspace 根目录下运行
roslaunch fast_lio mapping_mid360.launch
注意：启动前请确认上述提到的 config 外参已配置正确。

Step 5: 启动 Px4Ctrl 控制器
Bash

roslaunch px4ctrl run_ctrl.launch
参数文件：ctrl_param_fpv.yaml

Step 6: 启动 SUPER Planner
Bash

roslaunch mission_planner click_mission.launch
说明：此命令包含 Rviz 可视化界面。 参数文件：click_smooth_ros1.yaml

Step 7: 执行自动飞行任务
进入脚本目录并运行打点飞行程序：

Bash

cd src/SUPER/super_planner/scripts
python3 flight_script.py  # 假设你的脚本叫这个名字，请替换为实际文件名
操作提示： 运行完此节点后，请立即切入 Offboard 模式并 解锁 (Arm) 无人机，无人机将响应规划器的指令开始执行任务。

5. 常见问题 | FAQ
无人机无法切入 Offboard 模式？

检查 Mavros 是否有数据流（rostopic echo /mavros/state）。

检查 Px4Ctrl 是否正常接收到了里程计信息（~odom 话题）。

定位漂移？

重点检查 FAST-LIO 的外参配置以及 IMU 的频率是否成功修改。
