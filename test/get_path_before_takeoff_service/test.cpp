#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <get_path_test.h>

using namespace std::chrono_literals;

class Tester : public GetPathTest {

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

  // | -------------------- call the service -------------------- |

  std::optional<mrs_msgs::msg::TrajectoryReference> trajectory;
  std::optional<Eigen::VectorXd>                    waypoint_idxs;

  {
    std::string message;

    std::tie(trajectory, waypoint_idxs, message) = uh_->getPathSrv(path);

    if (!trajectory) {
      RCLCPP_ERROR(node_->get_logger(), "goto failed with message: '%s'", message.c_str());
      return false;
    }
  }

  // | ------------------ check the trajectory ------------------ |

  {
    bool trajectory_is_fine = this->checkTrajectory(*trajectory, path, false);

    bool waypoints_are_fine = this->checkWaypintIdxs(*waypoint_idxs, path);

    if (!trajectory_is_fine || !waypoints_are_fine) {
      RCLCPP_ERROR(node_->get_logger(), "trajectory check failed");
      return false;
    } else {
      return true;
    }
  }

  RCLCPP_ERROR(node_->get_logger(), "reached the end of the test without assertion");

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
