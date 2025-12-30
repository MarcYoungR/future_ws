//
// Created by yunfan on 2022/3/26.
//

#ifndef MISSION_PLANNER_CONFIG
#define MISSION_PLANNER_CONFIG

#include "vector"
#include "string"
#include "iostream"
#include <iomanip>
#include <fstream>
#include <utils/eigen_alias.hpp>
#include <utils/yaml_loader.hpp>
#include <utils/color_text.hpp>
namespace mission_planner {
    using namespace super_utils;
    using namespace color_text;
    using namespace std;
    enum TriggerType {
        RVIZ_CLICK = 0,
        MAVROS_RC = 1,
        TARGET_ODOM = 2
    };

    enum CmdType {
        PATH = 0,
        WAYPOINT = 1
    };

    class MissionConfig {
    public:
        // Bool Params

        int start_trigger_type;
        int cmd_type;
        double start_program_delay;
        string path_pub_topic, goal_pub_topic, odom_topic, waypoints_file_name;
        vec_E<Vec3f> waypoints;
        vector<double> switch_dis_vec;
        double switch_dis;
        double odom_timeout;
        double publish_dt;

        double str2double(string s) {
            double d;
            stringstream ss;
            ss << s;
            ss >> setprecision(16) >> d;
            ss.clear();
            return d;
        }

        void LoadWaypoint(string file_name) {
            ifstream theFile(file_name);
            std::string line;
            Vec3f log;
            while (std::getline(theFile, line)) {
                std::vector<std::string> result;
                std::istringstream iss(line);
                for (std::string s; iss >> s;) {
                    result.push_back(s);
                }
                for (size_t i = 0; i < result.size() - 1; i++) {
                    log(i) = str2double(result[i]);
                }
                switch_dis_vec.push_back(str2double(result[result.size() - 1]));
                waypoints.push_back(log);
            }


            cout << GREEN << " -- [MISSION] Load " << waypoints.size() << " waypoints." << RESET << endl;
            for (int i = 0; i < waypoints.size(); i++) {
                cout << BLUE << " -- [MISSION] Waypoint " << i << " at (" << waypoints[i].x() << ", "
                     << waypoints[i].y() << ", " << waypoints[i].z() << ")" << " Switch dis = " << switch_dis_vec[i]
                     << RESET << endl;
            }
        }
        
        MissionConfig() {};

        MissionConfig(const std::string & cfg_path) {
            yaml_loader::YamlLoader loader(cfg_path);

            loader.LoadParam("start_trigger_type", start_trigger_type, 1);
            loader.LoadParam("start_program_delay", start_program_delay, 1.0);
            loader.LoadParam("cmd_type", cmd_type, 1);
            loader.LoadParam("switch_dis", switch_dis, 1.0);
            loader.LoadParam("odom_timeout", odom_timeout, 0.1);
            loader.LoadParam("publish_dt", publish_dt, 1.0);
            loader.LoadParam("goal_pub_topic", goal_pub_topic, string("/planner/goal"));
            loader.LoadParam("odom_topic", odom_topic, string("/lidar_slam/odom"));
//            loader.LoadParam("path_pub_topic", path_pub_topic, string("/planner/path_cmd"));
//            loader.LoadParam("waypoints_file_name", waypoints_file_name, string("a_working_waypoints.txt"));

        }

    };
}
#endif //PLANNER_CONFIG_HPP
