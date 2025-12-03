#ifndef TRAJECTORY_GENERATION_TEST_H
#define TRAJECTORY_GENERATION_TEST_H

#include <mrs_uav_testing/test_generic.h>

class TrajectoryGenerationTest : public mrs_uav_testing::TestGeneric {

public:
  TrajectoryGenerationTest();

  std::tuple<bool, std::string> checkPathFlythrough(const std::vector<Eigen::Vector4d> &waypoints);

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh_;
};

TrajectoryGenerationTest::TrajectoryGenerationTest() : mrs_uav_testing::TestGeneric(){};

std::tuple<bool, std::string> TrajectoryGenerationTest::checkPathFlythrough(const std::vector<Eigen::Vector4d> &waypoints) {

  unsigned long current_idx = 0;

  while (true) {

    if (!rclcpp::ok()) {
      return {false, "terminated form outside"};
    }

    if (uh_->isAtPosition(waypoints[current_idx][0], waypoints[current_idx][1], waypoints[current_idx][2], waypoints[current_idx][3], 2.0)) {
      RCLCPP_INFO(node_->get_logger(), "reached waypoint %lu", current_idx);
      current_idx++;
    }

    if (current_idx == waypoints.size()) {
      RCLCPP_INFO(node_->get_logger(), "reached last waypoint");
      return {true, "waypoints passed"};
    }

    sleep(0.01);
  }
}

#endif // TRAJECTORY_GENERATION_TEST_H
