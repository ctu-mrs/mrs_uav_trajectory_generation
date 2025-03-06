#include <gtest/gtest.h>

// include the generic test customized for this package
#include <get_path_test.h>

class Tester : public GetPathTest {

public:
  bool test();
};

bool Tester::test() {

  {
    auto [uhopt, message] = getUAVHandler(_uav_name_);

    if (!uhopt) {
      ROS_ERROR("[%s]: Failed obtain handler for '%s': '%s'", ros::this_node::getName().c_str(), _uav_name_.c_str(), message.c_str());
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

  mrs_msgs::Path path;

  path.fly_now     = true;
  path.use_heading = true;

  for (Eigen::Vector4d point : points) {

    mrs_msgs::Reference reference;
    reference.position.x = point[0];
    reference.position.y = point[1];
    reference.position.z = point[2];
    reference.heading    = point[3];

    path.points.push_back(reference);
  }

  // | ---------------- wait for ready to takeoff --------------- |

  while (true) {

    if (!ros::ok()) {
      return false;
    }

    ROS_INFO_THROTTLE(1.0, "[%s]: waiting for the MRS UAV System", name_.c_str());

    if (uh_->mrsSystemReady()) {
      ROS_INFO("[%s]: MRS UAV System is ready", name_.c_str());
      break;
    }

    sleep(0.01);
  }

  // | -------------------- call the service -------------------- |

  std::optional<mrs_msgs::TrajectoryReference> trajectory;
  std::optional<Eigen::VectorXd>               waypoint_idxs;

  {
    std::string message;

    std::tie(trajectory, waypoint_idxs, message) = uh_->getPathSrv(path);

    if (!trajectory) {
      ROS_ERROR("[%s]: goto failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | ------------------ check the trajectory ------------------ |

  {
    bool trajectory_is_fine = this->checkTrajectory(*trajectory, path, false);

    bool waypoints_are_fine = this->checkWaypintIdxs(*waypoint_idxs, path);

    if (!trajectory_is_fine || !waypoints_are_fine) {
      ROS_ERROR("[%s]: trajectory check failed", ros::this_node::getName().c_str());
      return false;
    } else {
      return true;
    }
  }

  ROS_ERROR("[%s]: reached the end of the test without assertion", ros::this_node::getName().c_str());

  return false;
}


TEST(TESTSuite, test) {

  Tester tester;

  bool result = tester.test();

  if (result) {
    GTEST_SUCCEED();
  } else {
    GTEST_FAIL();
  }
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  ros::init(argc, argv, "test");

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
