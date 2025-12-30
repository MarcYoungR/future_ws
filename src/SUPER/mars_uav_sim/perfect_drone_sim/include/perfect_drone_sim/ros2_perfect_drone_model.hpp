#ifndef _PERFECT_DRONE_SIM_HPP_
#define _PERFECT_DRONE_SIM_HPP_

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mars_quadrotor_msgs/msg/position_command.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "string"
#include "Eigen/Dense"
#include "nav_msgs/msg/path.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <marsim_render/marsim_render.hpp>
#include "pcl_conversions/pcl_conversions.h"
#include "perfect_drone_sim/config.hpp"
#include "tf2_ros/transform_broadcaster.h"


typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 3, 3> Mat33;

typedef Eigen::Matrix<double, 3, 3> StatePVA;
typedef Eigen::Matrix<double, 3, 4> StatePVAJ;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DynamicMat;
typedef Eigen::MatrixX4d MatX4;
typedef std::pair<double, Vec3> TimePosPair;

namespace perfect_drone {
    class PerfectDrone : public rclcpp::Node {
        std::shared_ptr<tf2_ros::TransformBroadcaster> br_map_ego_;

        Config cfg_;
        std::shared_ptr<marsim::MarsimRender> render_ptr_;
        double sys_start_t;
        rclcpp::TimerBase::SharedPtr odom_pub_timer_;
        rclcpp::TimerBase::SharedPtr global_pc_pub_timer_;
        rclcpp::TimerBase::SharedPtr local_pc_pub_timer_;
        rclcpp::Subscription<mars_quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_sub_;
        rclcpp::CallbackGroup::SharedPtr odom_timer_cbk_group, global_pc_pub_cbk_group, local_pc_pub_cbk_group,
                cmd_sub_cbk_group;


        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_pc_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_pc_pub_;
        Vec3 position_, velocity_;
        double yaw_;
        Eigen::Quaterniond q_;
        std::string mesh_resource_;

        nav_msgs::msg::Odometry odom_;
        nav_msgs::msg::Path path_;


    public:
        PerfectDrone(): Node("perfect_tracking") {
            // TODO: The current implementation uses a lenient QoS configuration for message transmission.
            const rclcpp::QoS qos(rclcpp::QoS(100)
                                          .best_effort()
                                          .keep_last(100)
                                          .durability_volatile());

#define CONFIG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "config/"+(name)))
            std::string dft_cfg_path = CONFIG_FILE_DIR("lidar_sim.yaml");
            std::string cfg_path, cfg_name;
            this->declare_parameter<std::string>("config_name", dft_cfg_path);

            if(this->get_parameter("config_name", cfg_name)){
                cfg_path = CONFIG_FILE_DIR(cfg_name);
                RCLCPP_WARN(this->get_logger(), " -- [MissionPlanner] Load config by file name: %s", cfg_path.c_str());
            }

            cfg_ = Config(cfg_path);

            // 创建 TransformBroadcaster

            br_map_ego_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);


            // 初始化 render_ptr_
            render_ptr_ = std::make_shared<marsim::MarsimRender>(cfg_path);

            // 订阅命令
            rclcpp::SubscriptionOptions so;
            cmd_sub_cbk_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            so.callback_group = cmd_sub_cbk_group;
            cmd_sub_ = this->create_subscription<mars_quadrotor_msgs::msg::PositionCommand>(
                    "/planning/pos_cmd", qos, std::bind(&PerfectDrone::cmdCallback, this, std::placeholders::_1),
                    so
            );

            // 发布 Odometry 消息
            odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/lidar_slam/odom", qos);

            // 发布 Pose 消息
            pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/lidar_slam/pose", qos);

            // 发布 Robot Marker
            robot_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("robot", qos);

            // 发布 Path
            path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", qos);

            // 发布 PointCloud2 消息
            local_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", qos);

            global_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_pc", qos);


            position_ = cfg_.init_pos;
            velocity_.setZero();
            yaw_ = cfg_.init_yaw;
            mesh_resource_ = cfg_.mesh_resource;
            q_ = Eigen::AngleAxisd(yaw_, Vec3::UnitZ());
            odom_.header.frame_id = "world";
            path_.poses.clear();
            path_.header.frame_id = "world";
            path_.header.stamp = this->get_clock()->now();


            odom_timer_cbk_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            odom_pub_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(10),
                    std::bind(&PerfectDrone::publishOdom, this),
                    odom_timer_cbk_group
            );

            global_pc_pub_cbk_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            global_pc_pub_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(1),
                    std::bind(&PerfectDrone::publishGlobalPC, this),
                    global_pc_pub_cbk_group
            );

            const int publish_dt_ms = static_cast<int>((1.0 / cfg_.sensing_rate) * 1000);
            local_pc_pub_cbk_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            local_pc_pub_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(publish_dt_ms),
                    std::bind(&PerfectDrone::publishPC, this),
                    local_pc_pub_cbk_group
            );

            sys_start_t = this->get_clock()->now().seconds();
        }

        double getSensingRate() {
            return cfg_.sensing_rate;
        }


        void publishPC() {
            pcl::PointCloud<marsim::PointType>::Ptr local_map(new pcl::PointCloud<marsim::PointType>);
            const auto cur_t = this->get_clock()->now().seconds();
            render_ptr_->renderOnceInWorld(position_.cast<float>(), q_.cast<float>(), cur_t, local_map);
            sensor_msgs::msg::PointCloud2 pc_msg;
            pcl::toROSMsg(*local_map, pc_msg);
            pc_msg.header.frame_id = "world";
            pc_msg.header.stamp = this->get_clock()->now();
            std::cout << "Publish local map size: " << local_map->size() << std::endl;
            local_pc_pub_->publish(pc_msg);
        }

        ~PerfectDrone() {}

    private:
        void cmdCallback(const mars_quadrotor_msgs::msg::PositionCommand::SharedPtr msg) {
            Vec3 pos(msg->position.x, msg->position.y, msg->position.z);
            Vec3 vel(msg->velocity.x, msg->velocity.y, msg->velocity.z);
            Vec3 acc(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z);
            double yaw = msg->yaw;
            updateFlatness(pos, vel, acc, yaw);
        }


        void publishGlobalPC() {
            static int last_sub_num = 0;
            // update sub num
            int sub_num = this->count_subscribers("/global_pc");
            double cur_t = this->get_clock()->now().seconds() - sys_start_t;
            if (sub_num > 0 && last_sub_num != sub_num || (cur_t > 5.0 && cur_t < 5.1)) {
                pcl::PointCloud<marsim::PointType>::Ptr global_map(new pcl::PointCloud<marsim::PointType>);
                render_ptr_->getGlobalMap(global_map);
                sensor_msgs::msg::PointCloud2 pc_msg;
                pcl::toROSMsg(*global_map, pc_msg);
                pc_msg.header.frame_id = "world";
                pc_msg.header.stamp = this->get_clock()->now();
                global_pc_pub_->publish(pc_msg);
                std::cout << "Publish global map size: " << global_map->size() << std::endl;
            }
            last_sub_num = sub_num;
        }

        void publishOdom() {
            odom_.pose.pose.position.x = position_.x();
            odom_.pose.pose.position.y = position_.y();
            odom_.pose.pose.position.z = position_.z();

            odom_.pose.pose.orientation.x = q_.x();
            odom_.pose.pose.orientation.y = q_.y();
            odom_.pose.pose.orientation.z = q_.z();
            odom_.pose.pose.orientation.w = q_.w();

            odom_.twist.twist.linear.x = velocity_.x();
            odom_.twist.twist.linear.y = velocity_.y();
            odom_.twist.twist.linear.z = velocity_.z();

            odom_.header.stamp = this->get_clock()->now();


            odom_pub_->publish(odom_);

            geometry_msgs::msg::PoseStamped pose;
            pose.pose = odom_.pose.pose;
            pose.header = odom_.header;
            pose_pub_->publish(pose);

            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped.header.stamp = odom_.header.stamp;
            transformStamped.header.frame_id = "world";
            transformStamped.child_frame_id = "perfect_drone";
            transformStamped.transform.translation.x = odom_.pose.pose.position.x;
            transformStamped.transform.translation.y = odom_.pose.pose.position.y;
            transformStamped.transform.translation.z = odom_.pose.pose.position.z;
            transformStamped.transform.rotation.x = odom_.pose.pose.orientation.x;
            transformStamped.transform.rotation.y = odom_.pose.pose.orientation.y;
            transformStamped.transform.rotation.z = odom_.pose.pose.orientation.z;
            transformStamped.transform.rotation.w = odom_.pose.pose.orientation.w;

            // 发布变换
            br_map_ego_->sendTransform(transformStamped);

            visualization_msgs::msg::Marker meshROS;
            meshROS.header.frame_id = "world";
            meshROS.header.stamp = odom_.header.stamp;
            meshROS.ns = "mesh";
            meshROS.id = 0;
            meshROS.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
            meshROS.action = visualization_msgs::msg::Marker::ADD;
            meshROS.pose.position = odom_.pose.pose.position;
            meshROS.pose.orientation = odom_.pose.pose.orientation;
            meshROS.scale.x = 1;
            meshROS.scale.y = 1;
            meshROS.scale.z = 1;
            meshROS.mesh_resource = mesh_resource_;
            meshROS.mesh_use_embedded_materials = true;
            meshROS.color.a = 1.0;
            meshROS.color.r = 0.0;
            meshROS.color.g = 0.0;
            meshROS.color.b = 0.0;
            robot_pub_->publish(meshROS);
            static int slow_down = 0;
            if (slow_down++ % 10 == 0) {
                if ((position_.head(2) - Vec3(0, -50, 1.5).head(2)).norm() < 1) {
                    path_.poses.clear();
                    path_.poses.reserve(10000);
                }
                path_.poses.push_back(pose);
                path_.header = odom_.header;
                path_pub_->publish(path_);
            }
        }

        void updateFlatness(const Vec3& pos, const Vec3& vel,
                            const Vec3& acc, const double yaw) {
            Vec3 gravity_ = 9.80 * Eigen::Vector3d(0, 0, 1);
            position_ = pos;
            velocity_ = vel;
            double a_T = (gravity_ + acc).norm();
            Eigen::Vector3d xB, yB, zB;
            Eigen::Vector3d xC(cos(yaw), sin(yaw), 0);

            zB = (gravity_ + acc).normalized();
            yB = ((zB).cross(xC)).normalized();
            xB = yB.cross(zB);
            Eigen::Matrix3d R;
            R << xB, yB, zB;
            q_ = Eigen::Quaterniond(R);
        }
    };
}


#endif