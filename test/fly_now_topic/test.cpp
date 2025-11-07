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

  // | -------------------- activate the UAV -------------------- |

  {
    auto [success, message] = uh_->activateMidAir();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "midair activation failed with message: '%s'", message.c_str());
      return false;
    }
  }

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

  // | -------------------- publish the path -------------------- |

  {
    auto [success, message] = uh_->setPathTopic(path);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "goto failed with message: '%s'", message.c_str());
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
