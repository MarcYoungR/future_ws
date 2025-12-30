#ifndef _PERFECT_DRONE_SIM_HPP_
#define _PERFECT_DRONE_SIM_HPP_

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "nav_msgs/Odometry.h"
#include "tf2_ros/transform_broadcaster.h"
#include "string"
#include "Eigen/Dense"
#include "nav_msgs/Path.h"
#include <sensor_msgs/PointCloud2.h>
#include "marsim_render/marsim_render.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "perfect_drone_sim/config.hpp"


typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 3, 3> Mat33;

typedef Eigen::Matrix<double, 3, 3> StatePVA;
typedef Eigen::Matrix<double, 3, 4> StatePVAJ;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DynamicMat;
typedef Eigen::MatrixX4d MatX4;
typedef std::pair<double, Vec3> TimePosPair;

namespace perfect_drone {
    class PerfectDrone {
        marsim::MarsimRender::Ptr render_ptr_;
        Config cfg_;

    public:
        PerfectDrone(const ros::NodeHandle &n) : nh_(n) {
#define CONFIG_FILE_DIR(name) (string(string(ROOT_DIR) + "config/"+name))
            std::string dft_cfg_path = CONFIG_FILE_DIR("click.yaml");
            std::string cfg_path, cfg_name;
            if (nh_.param("config_path", cfg_path, dft_cfg_path)) {
                cout << " -- [Fsm-Test] Load config from: " << cfg_path << endl;
            } else if (nh_.param("config_name", cfg_name, dft_cfg_path)) {
                cfg_path = CONFIG_FILE_DIR(cfg_name);
                cout << " -- [Fsm-Test] Load config by file name: " << cfg_name << endl;
            }
            cfg_ = Config(cfg_path);
            render_ptr_ = std::make_shared<marsim::MarsimRender>(cfg_path);
            cmd_sub_ = nh_.subscribe("/planning/pos_cmd", 100, &PerfectDrone::cmdCallback, this);
            odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/lidar_slam/odom", 100);
            pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/lidar_slam/pose", 100);
            robot_pub_ = nh_.advertise<visualization_msgs::Marker>("robot", 100);
            path_pub_ = nh_.advertise<nav_msgs::Path>("path", 100);
            local_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);
            global_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/global_pc", 100);
            vel_pub_ = nh_.advertise<visualization_msgs::Marker>("vel_text", 100);

            position_ = cfg_.init_pos;
            velocity_.setZero();
            yaw_ = cfg_.init_yaw;
            mesh_resource_ = cfg_.mesh_resource;
            q_ = Eigen::AngleAxisd(yaw_, Vec3::UnitZ());
            odom_.header.frame_id = "world";
            odom_pub_timer_ = nh_.createTimer(ros::Duration(0.01), &PerfectDrone::publishOdom, this);

            global_pc_pub_timer_ = nh_.createTimer(ros::Duration(0.001), &PerfectDrone::publishGlobalPC, this);
            path_.poses.clear();
            path_.header.frame_id = "world";
            path_.header.stamp = ros::Time::now();
        }

        double getSensingRate() {
            return cfg_.sensing_rate;
        }

        void visualizeText(const ros::Publisher &pub,
                           const std::string &ns,
                           const std::string &text,
                           const Vec3 &position,
                           const double &size,
                           const int &id
        ) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.ns = ns.c_str();
            if (id >= 0) {
                marker.id = id;
            } else {
                static int id = 0;
                marker.id = id++;
            }
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.scale.z = size;
            marker.color.a = 1.0;
            marker.color.r = 0.1;
            marker.color.g = 0.1;
            marker.color.b = 0.1;
            marker.text = text;
            marker.pose.position.x = position.x();
            marker.pose.position.y = position.y();
            marker.pose.position.z = position.z()+5.0;
            marker.pose.orientation.w = 1.0;
            pub.publish(marker);
        }


        void publishPC() {
            pcl::PointCloud<marsim::PointType>::Ptr local_map(new pcl::PointCloud<marsim::PointType>);
            render_ptr_->renderOnceInWorld(position_.cast<float>(), q_.cast<float>(), ros::Time::now().toSec(),
                                           local_map);
            sensor_msgs::PointCloud2 pc_msg;
            pcl::toROSMsg(*local_map, pc_msg);
            pc_msg.header.frame_id = "world";
            pc_msg.header.stamp = ros::Time::now();
            std::cout << "Publish local map size: " << local_map->size() << std::endl;
            local_pc_pub_.publish(pc_msg);
        }

        ~PerfectDrone() {}

    private:
        nav_msgs::Path path_;
        ros::Subscriber cmd_sub_;
        ros::Publisher odom_pub_, robot_pub_, pose_pub_, path_pub_, global_pc_pub_, local_pc_pub_, vel_pub_;
        ros::Timer odom_pub_timer_;
        ros::Timer pc_pub_timer_;
        ros::Timer global_pc_pub_timer_;
        ros::NodeHandle nh_;
        Vec3 position_, velocity_;
        double yaw_;
        Eigen::Quaterniond q_;
        nav_msgs::Odometry odom_;
        std::string mesh_resource_;

        void cmdCallback(const quadrotor_msgs::PositionCommandConstPtr &msg) {
            Vec3 pos(msg->position.x, msg->position.y, msg->position.z);
            Vec3 vel(msg->velocity.x, msg->velocity.y, msg->velocity.z);
            Vec3 acc(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z);
            double yaw = msg->yaw;
            updateFlatness(pos, vel, acc, yaw);
        }


        void publishGlobalPC(const ros::TimerEvent &e) {
            static int last_sub_num = 0;
            // update sub num
            int sub_num = global_pc_pub_.getNumSubscribers();
            if (sub_num > 0 && last_sub_num != sub_num) {
                ros::Duration(0.5).sleep();
                pcl::PointCloud<marsim::PointType>::Ptr global_map(new pcl::PointCloud<marsim::PointType>);
                render_ptr_->getGlobalMap(global_map);
                sensor_msgs::PointCloud2 pc_msg;
                pcl::toROSMsg(*global_map, pc_msg);
                pc_msg.header.frame_id = "world";
                pc_msg.header.stamp = ros::Time::now();
                global_pc_pub_.publish(pc_msg);
                std::cout << "Publish global map" << std::endl;
            }
            last_sub_num = sub_num;
        }

        void publishOdom(const ros::TimerEvent &e) {
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
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(3) << velocity_.norm();  // 设置两位小数
            visualizeText(vel_pub_, "vel",
                          "Speed: " + oss.str() + " m/s",
                          position_, 3.0, 0);

            odom_.header.stamp = ros::Time::now();


            odom_pub_.publish(odom_);

            geometry_msgs::PoseStamped pose;
            pose.pose = odom_.pose.pose;
            pose.header = odom_.header;
            pose_pub_.publish(pose);

            static tf2_ros::TransformBroadcaster br_map_ego;
            geometry_msgs::TransformStamped transformStamped;
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
            br_map_ego.sendTransform(transformStamped);

            visualization_msgs::Marker meshROS;
            meshROS.header.frame_id = "world";
            meshROS.header.stamp = odom_.header.stamp;
            meshROS.ns = "mesh";
            meshROS.id = 0;
            meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
            meshROS.action = visualization_msgs::Marker::ADD;
            meshROS.pose.position = odom_.pose.pose.position;
            meshROS.pose.orientation = odom_.pose.pose.orientation;
            meshROS.scale.x = 1;
            meshROS.scale.y = 1;
            meshROS.scale.z = 1;
            meshROS.mesh_resource = mesh_resource_;
            meshROS.mesh_use_embedded_materials = true;
            meshROS.color.a = 1.0;
            meshROS.color.r = 1.0;
            meshROS.color.g = 1.0;
            meshROS.color.b = 1.0;
            robot_pub_.publish(meshROS);
            static int slow_down = 0;
            if (slow_down++ % 10 == 0) {
                if ((position_.head(2) - Vec3(0, -50, 1.5).head(2)).norm() < 1) {
                    path_.poses.clear();
                    path_.poses.reserve(10000);
                }
                path_.poses.push_back(pose);
                path_.header = odom_.header;
                path_pub_.publish(path_);
            }
        }

        void updateFlatness(const Vec3 &pos, const Vec3 &vel,
                            const Vec3 &acc, const double yaw) {
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
