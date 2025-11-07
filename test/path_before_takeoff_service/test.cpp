#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <trajectory_generation_test.h>

using namespace std::chrono_literals;

class Tester : public TrajectoryGenerationTest {

public:
  bool test(void);
};

bool Tester::test(void) {

  {
    auto [uhopt, message] = getUAVHandler("uav1");

    if (!uhopt) {
      RCLCPP_ERROR(node_->get_logger(), "Failed obtain handler for '%s': '%s'", "uav1", message.c_str());
      return false;
    }

    uh_ = uhopt.value();
  }

  // | --------------------- define the path -------------------- |

  std::vector<Eigen::Vector4d> points;

  points.push_back(Eigen::Vector4d(-5, -5, 5, 1));
  points.push_back(Eigen::Vector4d(-5, 5, 5, 2));
  points.push_back(Eigen::Vector4d(5, -5, 5, 3));
  points.push_back(Eigen::Vector4d(5, 5, 5, 4));

  // | ---------------- prepare the path message ---------------- |

  mrs_msgs::msg::Path path;

  path.fly_now     = true;
  path.use_heading = true;

  for (Eigen::Vector4d point : points) {

    mrs_msgs::msg::Reference reference;
    reference.position.x = point[0];
    reference.position.y = point[1];
    reference.position.z = point[2];
    reference.heading    = point[3];

    path.points.push_back(reference);
  }

  // | ---------------- wait for ready to takeoff --------------- |

  while (true) {

    if (!rclcpp::ok()) {
      return false;
    }

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "waiting for the MRS UAV System");

    if (uh_->mrsSystemReady()) {
      RCLCPP_INFO(node_->get_logger(), "MRS UAV System is ready");
      break;
    }

    sleep(0.01);
  }

  // TODO we are not waiting for trajectory generation diagnostics to be ready
  sleep(3.0);

  // | -------------------- call the service -------------------- |

  {
    auto [success, message] = uh_->setPathSrv(path);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "goto failed with message: '%s'", message.c_str());
      return false;
    }
  }

  // | ---------- add the first waypoint after takeoff ---------- |

  {
    auto uav_pos = uh_->sh_uav_state_.getMsg()->pose.position;

    double heading = mrs_lib::AttitudeConverter(uh_->sh_uav_state_.getMsg()->pose.orientation).getHeading();

    mrs_msgs::msg::Reference reference;
    reference.position.x = uav_pos.x;
    reference.position.y = uav_pos.y;
    reference.position.z = uav_pos.z + 3.0;
    reference.heading    = heading;

    path.points.insert(path.points.begin(), reference);
  }

  // | ------------------------- takeoff ------------------------ |

  {
    auto [success, message] = uh_->takeoff();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "takeoff failed with message: '%s'", message.c_str());
      return false;
    }
  }

  // | ----------------------- goto start ----------------------- |

  {
    auto [success, message] = uh_->gotoTrajectoryStart();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "goto trajectory start failed with message: '%s'", message.c_str());
      return false;
    }
  }

  // | ---------------- start trajectory tracking --------------- |

  {
    auto [success, message] = uh_->startTrajectoryTracking();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "start trajectory tracking failed with message: '%s'", message.c_str());
      return false;
    }
  }

  // | --------------- check if we pass each point -------------- |

  {
    auto [success, message] = this->checkPathFlythrough(points);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "path flythrough failed: '%s'", message.c_str());
      return false;
    }
  }

  // | ---------- wait and check if we are still flying --------- |

  sleep(5.0);

  if (uh_->isFlyingNormally()) {
    return true;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "not flying normally");
    return false;
  }

  RCLCPP_ERROR(node_->get_logger(), "reached the end of the test methd without assertion");

  return false;
}

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  bool test_result = true;

  Tester tester;

  test_result &= tester.test();

  tester.sleep(2.0);

  std::cout << "Test: reporting test results" << std::endl;

  tester.reportTestResult(test_result);

  tester.join();
}
