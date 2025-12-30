//
// Created by yunfan on 2022/3/26.
//

#ifndef PERFECT_DRONE_CONFIG_HPP
#define PERFECT_DRONE_CONFIG_HPP


#include "cstring"
#include "vector"
#include "iostream"
#include "utils/yaml_loader.hpp"

namespace perfect_drone {
    using std::cout;
    using std::endl;
    using std::string;
    using std::vector;

    class Config {
    public:
        std::string mesh_resource;
        Eigen::Vector3d init_pos;
        double init_yaw;
        double sensing_rate;

        Config() = default;

        Config(const std::string &cfg_path) {
            yaml_loader::YamlLoader loader(cfg_path);
            loader.LoadParam("mesh_resource", mesh_resource, std::string("package://perfect_drone_sim/meshes/f250.dae"),
                             false);
            loader.LoadParam("init_position/x", init_pos.x(), 0.0);
            loader.LoadParam("init_position/y", init_pos.y(), 0.0);
            loader.LoadParam("init_position/z", init_pos.z(), 1.5);
            loader.LoadParam("init_yaw", init_yaw, 0.0);
            loader.LoadParam("sensing_rate", sensing_rate, 10.0);
        }
    };
}

#endif
