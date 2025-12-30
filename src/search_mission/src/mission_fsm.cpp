#include "search_mission/mission_fsm.hpp"

namespace search_mission {

MissionFSM::MissionFSM(ros::NodeHandle &nh, ros::NodeHandle &pnh) 
    : nh_(nh), pnh_(pnh), current_state_(IDLE), has_odom_(false), 
      is_armed_(false), aruco_detected_(false), cur_wp_index_(0), cur_search_index_(0) {
    
    // 1. 加载参数
    if (!loadParameters()) {
        ROS_ERROR("[SearchMission] Parameter loading failed, node shutting down!");
        ros::shutdown();
        return;
    }

    // 2. 初始化 ROS 接口
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(cfg_.goal_topic, 10);
    odom_sub_ = nh_.subscribe(cfg_.odom_topic, 10, &MissionFSM::odomCallback, this);
    state_sub_ = nh_.subscribe(cfg_.state_topic, 10, &MissionFSM::stateCallback, this);
    aruco_sub_ = nh_.subscribe(cfg_.aruco_topic, 10, &MissionFSM::arucoCallback, this);

    // 3. 启动 10Hz 状态机定时器
    fsm_timer_ = nh_.createTimer(ros::Duration(0.1), &MissionFSM::fsmCallback, this);

    ROS_INFO("\033[1;32m[SearchMission] Initialized. Waiting for ARM and [OFFBOARD] mode...\033[0m");
    ROS_INFO("[SearchMission] Ground height set to: %.2f m", cfg_.ground_height);
}

bool MissionFSM::loadParameters() {
    // 话题名称
    pnh_.param<std::string>("goal_topic", cfg_.goal_topic, "/planning/click_goal");
    pnh_.param<std::string>("odom_topic", cfg_.odom_topic, "/mavros/local_position/odom");
    pnh_.param<std::string>("aruco_topic", cfg_.aruco_topic, "/aruco_single/pose");
    pnh_.param<std::string>("state_topic", cfg_.state_topic, "/mavros/state");

    // 任务参数
    pnh_.param("takeoff_height", cfg_.takeoff_height, 1.0);
    pnh_.param("reach_tol_xyz", cfg_.reach_tol_xyz, 0.2);
    double yaw_deg;
    pnh_.param("reach_tol_yaw_deg", yaw_deg, 30.0);
    cfg_.reach_tol_yaw_rad = yaw_deg * M_PI / 180.0;
    pnh_.param("wp_timeout", cfg_.wp_timeout, 30.0);

    // 降落与搜寻参数
    pnh_.param("landing_search_timeout", cfg_.landing_search_timeout, 45.0);
    pnh_.param("search_radius", cfg_.search_radius, 1.0);
    pnh_.param("descend_vel", cfg_.descend_vel, 0.2);
    pnh_.param("land_kp_xy", cfg_.land_kp_xy, 0.6);
    pnh_.param("ground_height", cfg_.ground_height, -0.3);
    pnh_.param("land_height_threshold", cfg_.land_height_threshold, -0.2);
    pnh_.param("final_land_setpoint", cfg_.final_land_setpoint, -0.8);

    // 加载航点列表
    XmlRpc::XmlRpcValue waypoints_xml;
    if (pnh_.getParam("waypoints", waypoints_xml)) {
        if (waypoints_xml.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR("YAML parameter 'waypoints' must be a list!");
            return false;
        }
        for (int i = 0; i < waypoints_xml.size(); ++i) {
            XmlRpc::XmlRpcValue pt = waypoints_xml[i];
            if (pt.size() != 4) {
                ROS_ERROR("Waypoint #%d format error, must be [x, y, z, yaw_deg]", i);
                return false;
            }
            double x = static_cast<double>(pt[0]);
            double y = static_cast<double>(pt[1]);
            double z = static_cast<double>(pt[2]);
            double yaw = static_cast<double>(pt[3]) * M_PI / 180.0;
            waypoints_.push_back(Eigen::Vector4d(x, y, z, yaw));
        }
    } else {
        ROS_WARN("No 'waypoints' found in YAML, mission will only execute Takeoff!");
    }
    return true;
}

void MissionFSM::fsmCallback(const ros::TimerEvent &e) {
    if (!has_odom_) {
        ROS_WARN_THROTTLE(2.0, "[SearchMission] Waiting for Odom data...");
        return;
    }

    // 检查 Aruco 超时
    if ((ros::Time::now() - last_aruco_stamp_).toSec() > 0.5) {
        aruco_detected_ = false;
    }

    switch (current_state_) {
        case IDLE:
            // 触发条件：已解锁 + 切入 OFFBOARD
            if (is_armed_ && current_mavros_mode_ == "OFFBOARD" && last_mavros_mode_ != "OFFBOARD") {
                ROS_INFO("\033[1;33m[Mission] Triggered! Taking off to %.2fm\033[0m", cfg_.takeoff_height);
                changeState(TAKE_OFF);
            }
            break;

        case TAKE_OFF: {
            Eigen::Vector3d target(cur_pos_.x(), cur_pos_.y(), cfg_.takeoff_height);
            publishGoal(target, cur_yaw_);
            // 检查起飞是否完成
            if (std::abs(cur_pos_.z() - cfg_.takeoff_height) < cfg_.reach_tol_xyz) {
                ROS_INFO("[Mission] Takeoff complete. Heading to the first waypoint.");
                cur_wp_index_ = 0;
                changeState(EXECUTE_WAYPOINTS);
            }
            break;
        }

        case EXECUTE_WAYPOINTS: {
            if (waypoints_.empty() || cur_wp_index_ >= waypoints_.size()) {
                ROS_INFO("[Mission] All waypoints reached. Starting landing search.");
                generateSearchPattern(cur_pos_);
                changeState(SEARCHING_ARUCO);
                break;
            }

            Eigen::Vector4d wp = waypoints_[cur_wp_index_];
            publishGoal(wp.head<3>(), wp(3));

            bool reached = checkGoalReached(wp.head<3>(), wp(3));
            bool timeout = (ros::Time::now() - state_start_time_).toSec() > cfg_.wp_timeout;

            if (reached || timeout) {
                if (timeout) ROS_WARN("[Mission] Waypoint %d timeout, skipping!", cur_wp_index_);
                else ROS_INFO("[Mission] Waypoint %d reached.", cur_wp_index_);
                
                cur_wp_index_++;
                state_start_time_ = ros::Time::now();
            }
            break;
        }

        case SEARCHING_ARUCO: {
            if (aruco_detected_) {
                ROS_INFO("\033[1;32m[Mission] Aruco detected! Switching to Visual Servo Landing.\033[0m");
                changeState(VISUAL_SERVO_LAND);
                break;
            }
            if ((ros::Time::now() - state_start_time_).toSec() > cfg_.landing_search_timeout) {
                ROS_ERROR("[Mission] Search timeout. Executing FORCE BLIND LANDING.");
                changeState(FORCE_LAND);
                break;
            }

            // 沿搜索图案飞行
            if (cur_search_index_ < search_pattern_.size()) {
                publishGoal(search_pattern_[cur_search_index_], cur_yaw_);
                if ((cur_pos_ - search_pattern_[cur_search_index_]).norm() < 0.3) {
                    cur_search_index_++;
                }
            } else {
                cur_search_index_ = 0; // 循环搜索
            }
            break;
        }

        case VISUAL_SERVO_LAND: {
            // 1. 检查高度是否足够低以执行压地 (穿地逻辑)
            if (cur_pos_.z() < cfg_.land_height_threshold) {
                ROS_WARN_THROTTLE(1.0, "[Mission] Low altitude (%.2fm). Executing final PRESS-DOWN...", cur_pos_.z());
                Eigen::Vector3d press_down(cur_pos_.x(), cur_pos_.y(), cfg_.final_land_setpoint);
                publishGoal(press_down, cur_yaw_);
                return; // 保持在此状态直到上锁
            }

            // 2. 目标丢失处理
            if (!aruco_detected_) {
                ROS_WARN_THROTTLE(0.5, "[Mission] Visual lost during landing. Descending vertically...");
                Eigen::Vector3d blind_des = cur_pos_;
                blind_des.z() -= cfg_.descend_vel * 0.1;
                publishGoal(blind_des, cur_yaw_);
                return;
            }

            // 3. 视觉伺服控制 (机体系 -> 世界系)
            double err_x = aruco_rel_pos_.x(); 
            double err_y = aruco_rel_pos_.y();
            double move_x = err_x * cos(cur_yaw_) - err_y * sin(cur_yaw_);
            double move_y = err_x * sin(cur_yaw_) + err_y * cos(cur_yaw_);

            Eigen::Vector3d land_target;
            land_target.x() = cur_pos_.x() + cfg_.land_kp_xy * move_x;
            land_target.y() = cur_pos_.y() + cfg_.land_kp_xy * move_y;
            land_target.z() = cur_pos_.z() - cfg_.descend_vel * 0.1;

            publishGoal(land_target, cur_yaw_);
            break;
        }

        case FORCE_LAND: {
            if (cur_pos_.z() < cfg_.land_height_threshold) {
                Eigen::Vector3d press_down(cur_pos_.x(), cur_pos_.y(), cfg_.final_land_setpoint);
                publishGoal(press_down, cur_yaw_);
            } else {
                Eigen::Vector3d des = cur_pos_;
                des.z() -= cfg_.descend_vel * 0.1;
                publishGoal(des, cur_yaw_);
            }
            break;
        }

        case FINISHED:
            break;
    }
    last_mavros_mode_ = current_mavros_mode_;
}

// --- 辅助函数 ---
bool MissionFSM::checkGoalReached(const Eigen::Vector3d &target, double target_yaw) {
    double d = (cur_pos_ - target).norm();
    double ang = std::abs(normalizeAngle(cur_yaw_ - target_yaw));
    return (d < cfg_.reach_tol_xyz && ang < cfg_.reach_tol_yaw_rad);
}

void MissionFSM::publishGoal(const Eigen::Vector3d &pos, double yaw) {
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = pos.x();
    msg.pose.position.y = pos.y();
    msg.pose.position.z = pos.z();
    msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    goal_pub_.publish(msg);
}

void MissionFSM::generateSearchPattern(const Eigen::Vector3d &c) {
    search_pattern_.clear();
    double r = cfg_.search_radius;
    // 十字搜索图案
    search_pattern_.push_back(c);
    search_pattern_.push_back(c + Eigen::Vector3d(r, 0, 0));
    search_pattern_.push_back(c + Eigen::Vector3d(-r, 0, 0));
    search_pattern_.push_back(c + Eigen::Vector3d(0, r, 0));
    search_pattern_.push_back(c + Eigen::Vector3d(0, -r, 0));
    cur_search_index_ = 0;
}

void MissionFSM::changeState(MissionState next) {
    current_state_ = next;
    state_start_time_ = ros::Time::now();
}

double MissionFSM::normalizeAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

void MissionFSM::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
    has_odom_ = true;
    cur_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    cur_yaw_ = tf::getYaw(msg->pose.pose.orientation);
}

void MissionFSM::stateCallback(const mavros_msgs::StateConstPtr &msg) {
    current_mavros_mode_ = msg->mode;
    is_armed_ = msg->armed;
}

void MissionFSM::arucoCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
    aruco_detected_ = true;
    last_aruco_stamp_ = ros::Time::now();
    aruco_rel_pos_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
}

} // namespace

int main(int argc, char **argv) {
    ros::init(argc, argv, "search_mission_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    search_mission::MissionFSM fsm(nh, pnh);
    ros::spin();
    return 0;
}
