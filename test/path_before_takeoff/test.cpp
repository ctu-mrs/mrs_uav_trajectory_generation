#include <gtest/gtest.h>

// include the generic test customized for this package
#include <trajectory_generation_test.h>

class Tester : public TrajectoryGenerationTest {

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

  // TODO we are not waiting for trajectory generaiton diagnostics to be ready
  sleep(10.0);

  // | -------------------- call the service -------------------- |

  {
    auto [success, message] = uh_->setPathSrv(path);

    if (!success) {
      ROS_ERROR("[%s]: goto failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | ---------- add the first waypoint after takeoff ---------- |

  {
    auto uav_pos = uh_->sh_uav_state_.getMsg()->pose.position;

    double heading = mrs_lib::AttitudeConverter(uh_->sh_uav_state_.getMsg()->pose.orientation).getHeading();

    mrs_msgs::Reference reference;
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
      ROS_ERROR("[%s]: takeoff failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | ----------------------- goto start ----------------------- |

  {
    auto [success, message] = uh_->gotoTrajectoryStart();

    if (!success) {
      ROS_ERROR("[%s]: goto trajectory start failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | ---------------- start trajectory tracking --------------- |

  {
    auto [success, message] = uh_->startTrajectoryTracking();

    if (!success) {
      ROS_ERROR("[%s]: start trajectory tracking failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | --------------- check if we pass each point -------------- |

  {
    auto [success, message] = this->checkPathFlythrough(points);

    if (!success) {
      ROS_ERROR("[%s]: path flythrough failed: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | ---------- wait and check if we are still flying --------- |

  sleep(5.0);

  if (uh_->isFlyingNormally()) {
    return true;
  } else {
    ROS_ERROR("[%s]: not flying normally", ros::this_node::getName().c_str());
    return false;
  }

  ROS_ERROR("[%s]: reached the end of the test methd without assertion", ros::this_node::getName().c_str());

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
