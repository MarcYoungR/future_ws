#ifndef MISSION_FSM_HPP
#define MISSION_FSM_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <cmath>
#include <XmlRpcValue.h> // 解析 YAML 列表

namespace search_mission {

enum MissionState {
    IDLE,               // 等待解锁 + Offboard
    TAKE_OFF,           // 起飞
    EXECUTE_WAYPOINTS,  // 执行航点序列
    SEARCHING_ARUCO,    // 末端搜索
    VISUAL_SERVO_LAND,  // 视觉伺服降落
    FORCE_LAND,         // 强制盲降
    FINISHED            // 任务结束
};

class MissionFSM {
public:
    MissionFSM(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~MissionFSM() = default;

private:
    // --- ROS 句柄 ---
    ros::NodeHandle nh_, pnh_;
    ros::Publisher goal_pub_;
    ros::Subscriber odom_sub_, state_sub_, aruco_sub_;
    ros::Timer fsm_timer_;

    // --- 状态变量 ---
    MissionState current_state_;
    Eigen::Vector3d cur_pos_;
    double cur_yaw_;
    
    bool has_odom_;
    bool is_armed_;
    std::string current_mavros_mode_;
    std::string last_mavros_mode_;

    // --- 视觉变量 ---
    bool aruco_detected_;
    Eigen::Vector3d aruco_rel_pos_; // 相对位置
    ros::Time last_aruco_stamp_;

    // --- 任务数据 ---
    std::vector<Eigen::Vector4d> waypoints_; // x, y, z, yaw(rad)
    int cur_wp_index_;
    std::vector<Eigen::Vector3d> search_pattern_;
    int cur_search_index_;
    ros::Time state_start_time_;

    // --- 参数配置结构体 ---
    struct Config {
        std::string goal_topic;
        std::string odom_topic;
        std::string aruco_topic;
        std::string state_topic;

        double takeoff_height;
        double reach_tol_xyz;
        double reach_tol_yaw_rad; // 内部存弧度
        double wp_timeout;
        
        double landing_search_timeout;
        double search_radius;
        double descend_vel;
        double land_kp_xy;
        
        // 降落判定参数
        double ground_height;
        double land_height_threshold;
        double final_land_setpoint;
    } cfg_;

    // --- 核心函数 ---
    bool loadParameters();
    void fsmCallback(const ros::TimerEvent &e);
    
    // 回调函数
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
    void stateCallback(const mavros_msgs::StateConstPtr &msg);
    void arucoCallback(const geometry_msgs::PoseStampedConstPtr &msg);

    // 辅助工具
    void publishGoal(const Eigen::Vector3d &pos, double yaw);
    bool checkGoalReached(const Eigen::Vector3d &target_pos, double target_yaw);
    void generateSearchPattern(const Eigen::Vector3d &center);
    void changeState(MissionState next_state);
    double normalizeAngle(double angle);
};

} // namespace search_mission

#endif