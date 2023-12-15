#ifndef TRAJECTORY_GENERATION_TEST_H
#define TRAJECTORY_GENERATION_TEST_H

#include <mrs_uav_testing/test_generic.h>

class TrajectoryGenerationTest : public mrs_uav_testing::TestGeneric {

public:
  std::tuple<bool, std::string> checkPathFlythrough(const std::vector<Eigen::Vector4d> &waypoints);
};

std::tuple<bool, std::string> TrajectoryGenerationTest::checkPathFlythrough(const std::vector<Eigen::Vector4d> &waypoints) {

  unsigned long current_idx = 0;

  while (true) {

    if (!ros::ok()) {
      return {false, "terminated form outside"};
    }

    if (this->isAtPosition(waypoints[current_idx][0], waypoints[current_idx][1], waypoints[current_idx][2], waypoints[current_idx][3], 0.5)) {
      current_idx++;
    }

    if (current_idx == waypoints.size()) {
      return {true, "waypoints passed"};
    }

    sleep(0.01);
  }
}

#endif  // TRAJECTORY_GENERATION_TEST_H
