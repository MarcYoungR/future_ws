#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest

class AutoMissionExecutor:
    def __init__(self):
        rospy.init_node('auto_mission_executor')

        # --- 配置参数 ---
        self.target_frame = "world"   # 需与 super_planner 的地图坐标系一致
        self.takeoff_height =1.0     # 起飞高度 (m)
        self.wait_time = 15.0         # 每个点的强制停留时间 (s)

        # 定义 6 个任务航点 [x, y, z]
        self.waypoints = [
            [1.0,  0.0,  1.0],   # 第 1 点
            [1.0,  1.0,  1.0],   # 第 2 点
            [1.0,  -1.0,  1.0],   # 第 3 点
            [1.0, 0.0,  1.0],   # 第 5 点
            [0.0,  0.0,  1.0],    # 第 6 点 (最后一点)
            [0.0,  0.0,  0.1]
            
        ]

        # --- ROS 接口 ---
        self.current_state = State()
        self.state_sub = rospy.Subscriber("mavros/state", State, self.state_cb)
        
        # 发布给 Super Planner 的目标话题
        #
        self.goal_pub = rospy.Publisher("/goal", PoseStamped, queue_size=10)

        # MAVROS 服务
        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

        # 等待服务连接
        rospy.loginfo("正在等待 MAVROS 服务...")
        try:
            rospy.wait_for_service('mavros/cmd/arming', timeout=5)
            rospy.wait_for_service('mavros/set_mode', timeout=5)
        except rospy.ROSException:
            rospy.logerr("连接 MAVROS 服务超时，请检查仿真或飞控连接！")

    def state_cb(self, msg):
        self.current_state = msg

    def publish_goal(self, x, y, z):
        """发布 PoseStamped 给 Super Planner"""
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.target_frame
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.w = 1.0 # 默认朝向
        
        self.goal_pub.publish(goal)
        # 连续发几次以防丢包 (Super Planner 收到一次即可触发规划)
        for _ in range(3):
            self.goal_pub.publish(goal)
            rospy.sleep(0.05)

    def try_arm_and_offboard(self):
        """尝试解锁并切换到 OFFBOARD 模式"""
        # 1. 先发 100 个起飞点，确保 Super Planner 开始输出控制流
        # 如果没有控制流，PX4 会拒绝切入 OFFBOARD
        rospy.loginfo(">>> [初始化] 正在发布起飞点，以激活规划器控制流...")
        start_pos = [0, 0, self.takeoff_height]
        for _ in range(50): 
            if rospy.is_shutdown(): return False
            self.publish_goal(start_pos[0], start_pos[1], start_pos[2])
            rospy.sleep(0.05)

        # 2. 请求解锁
        if not self.current_state.armed:
            rospy.loginfo(">>> 正在请求解锁 (Arming)...")
            try:
                self.arming_client(CommandBoolRequest(value=True))
            except rospy.ServiceException as e:
                rospy.logerr(f"解锁请求失败: {e}")

        rospy.sleep(1.0)

        # 3. 请求 OFFBOARD
        if self.current_state.mode != "OFFBOARD":
            rospy.loginfo(">>> 正在请求 OFFBOARD 模式...")
            try:
                # 再次发送目标点确保控制流不断
                self.publish_goal(start_pos[0], start_pos[1], start_pos[2])
                self.set_mode_client(SetModeRequest(custom_mode="OFFBOARD"))
            except rospy.ServiceException as e:
                rospy.logerr(f"切换模式请求失败: {e}")
        
        return True

    def run(self):
        # 等待连接
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.loginfo_throttle(2.0, "等待飞控心跳...")
            rospy.sleep(0.1)

        # --- 阶段 1: 自动起飞逻辑 ---
        # 这里的逻辑是：发布起飞高度 -> 解锁 -> 切模式 -> 等待起飞完成(20s)
        if self.try_arm_and_offboard():
            rospy.loginfo(f">>> [起飞] 目标高度 {self.takeoff_height}m. 等待 {self.wait_time} 秒...")
            # 这里的“等待”即为起飞过程
            rospy.sleep(self.wait_time)
        else:
            rospy.logerr("初始化失败，退出任务。")
            return

        # --- 阶段 2: 执行 6 个航点 ---
        rospy.loginfo(">>> [任务开始] 执行 6 点巡航...")
        
        for i, pt in enumerate(self.waypoints):
            if rospy.is_shutdown(): break
            
            # 发布目标
            rospy.loginfo(f">>> [航点 {i+1}/6] 前往: {pt}")
            self.publish_goal(pt[0], pt[1], pt[2])
            
            # 强制等待 20 秒
            rospy.loginfo(f"    ... 保持 15 秒 ...")
            rospy.sleep(self.wait_time)

        # --- 阶段 3: 降落逻辑 (发布高度 0) ---
        last_pt = self.waypoints[-1]
        rospy.loginfo(f">>> [降落] 在最后一个点 {last_pt[:2]} 发布高度 0.0m")
        
        # 持续发布 0 高度，直到脚本结束或手动停止
        # Super Planner 收到 z=0 会规划一条撞地轨迹，PX4 检测到触地后会自动加锁(Disarm)
        land_goal = [last_pt[0], last_pt[1], -0.5] # 设置为略低于地面(-0.2)以确保压实地面
        
        start_land = rospy.Time.now()
        while not rospy.is_shutdown():
            self.publish_goal(land_goal[0], land_goal[1], land_goal[2])
            rospy.loginfo_throttle(5.0, ">>> 正在降落中 (发布 z=-0.5)...")
            
            # 如果已经加锁（降落完成），则退出
            if not self.current_state.armed and (rospy.Time.now() - start_land).to_sec() > 5.0:
                rospy.loginfo(">>> 检测到无人机已加锁 (Disarmed)，任务完成。")
                break
            
            rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        executor = AutoMissionExecutor()
        executor.run()
    except rospy.ROSInterruptException:
        pass
