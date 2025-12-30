#include "perfect_drone_sim/ros1_perfect_drone_model.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "perfect_tracking");
  ros::NodeHandle n("~");
  perfect_drone::PerfectDrone dp(n);
  ros::AsyncSpinner spinner(0);
  spinner.start();

  ros::Rate lprt(dp.getSensingRate());
  while(ros::ok()) {
    dp.publishPC();
    lprt.sleep();
  }

  ros::waitForShutdown();
  return 0;
}

