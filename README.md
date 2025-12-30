Autonomous UAV System (FAST-LIO + Px4Ctrl + SUPER)


# 1. 项目简介


本项目是一个集成了定位、控制与规划的全自主无人机系统。系统架构主要包含以下三个核心模块：

定位与建图:采用FAST-LIO2方案[hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO)，基于Livox MID360激光雷达提供高精度的里程计与环境点云。

底层控制:采用高飞老师团队开源的Px4Ctrl控制器[ZJU-FAST-Lab/Fast-Drone-250](https://github.com/ZJU-FAST-Lab/Fast-Drone-250)，提供稳定的轨迹跟踪控制。

避障与规划:采用MARS实验室的SUPER Planner[hku-mars/SUPER](https://github.com/hku-mars/SUPER)，实现动态环境下的避障与路径规划。

只需通过 Rviz 或代码逻辑向系统发送 /goal 目标点话题，无人机即可自动规划路径并飞行至目标位置。



# 2. 硬件与驱动准备

在启动系统前，请确保以下硬件驱动已正确安装并调试完毕：

## 2.1 Livox MID360 驱动
需要预先安装 livox_ros_driver2。

注意： 请参考本项目文档中的 [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2),进行网络配置和 SDK 编译，确保能够正常读取点云数据。

## 2.2 Mavros 通信
确保机载电脑与飞控（Pixhawk/PX4）通过串口或 USB 连接正常。

确保mavros包已安装，且波特率与飞控设置一致。

# 3. 参数配置指南
   
在使用 FAST-LIO 进行定位前，必须根据你的无人机实际结构修改配置文件。

FAST-LIO 配置文件
文件路径：src/FAST_LIO/config/mid360.yaml

修改内容：需要修改雷达（LiDAR）相对于飞控 IMU 的外参。

如果雷达安装位置发生变化，务必重新测量并更新这两个参数，否则会导致里程计漂移或控制发散。

其他模块参数

SUPER Planner: 修改 src/SUPER/super_planner/config/click_smooth_ros1.yaml 以调整规划器的速度、加速度限制及避障策略。


Px4Ctrl: 修改 src/Px4ctrl/config/ctrl_param_fpv.yaml 以调整 PID 增益及动力学参数。

## 4. 启动流程

请严格按照以下顺序在不同的终端窗口中启动各个模块。

### Step 1: 启动传感器与飞控通信
首先启动雷达驱动与 Mavros。

```bash
# 1. 启动 Livox MID360 驱动
roslaunch livox_ros_driver2 msg_MID360.launch

# 2. 启动 Mavros (连接飞控)
roslaunch mavros px4.launch

# 3. 修改 IMU 频率 (关键步骤)
# 说明: 此命令将飞控 IMU 频率设置为高频模式，以满足 FAST-LIO 的需求
rosrun mavros mavcmd long 511 31 10000 0 0 0 0
```

### Step 2: 启动 FAST-LIO 里程计
```Bash

# 请确保在 workspace 根目录下运行
roslaunch fast_lio mapping_mid360.launch
注意：启动前请确认上述提到的 config 外参已配置正确。

```

### Step 3: 启动 Px4Ctrl 控制器

```Bash

roslaunch px4ctrl run_ctrl.launch
# 参数文件：ctrl_param_fpv.yaml
```
### Step 4: 启动 SUPER Planner
```Bash

roslaunch mission_planner click_mission.launch
# 参数文件：click_smooth_ros1.yaml
```
### Step 5: 执行自动飞行任务
进入脚本目录并运行打点飞行程序：

```Bash

cd src/SUPER/super_planner/scripts
python3 auto_mission.py  # 假设你的脚本叫这个名字，请替换为实际文件名
操作提示： 运行完此节点后，请立即切入Offboard模式并 解锁(Arm)无人机，无人机将响应规划器的指令开始执行任务。
```

