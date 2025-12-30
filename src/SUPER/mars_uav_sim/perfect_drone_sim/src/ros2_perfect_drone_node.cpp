#include "perfect_drone_sim/ros2_perfect_drone_model.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<perfect_drone::PerfectDrone>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
