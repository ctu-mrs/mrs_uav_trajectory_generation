#ifndef GET_PATH_TEST_H
#define GET_PATH_TEST_H

#include <mrs_uav_testing/test_generic.h>

#include <mrs_msgs/TrajectoryReference.h>

using sradians = mrs_lib::geometry::sradians;

#define POS_TOLERANCE 0.5
#define HDG_TOLERANCE 0.2

class GetPathTest : public mrs_uav_testing::TestGeneric {

public:
  bool checkTrajectory(const mrs_msgs::TrajectoryReference& trajectory, const mrs_msgs::Path& path, bool starting_from_current_pos);
};

bool GetPathTest::checkTrajectory(const mrs_msgs::TrajectoryReference& trajectory, const mrs_msgs::Path& path, bool starting_from_current_pos) {

  if (starting_from_current_pos) {

    auto pos = this->getTrackerCmd()->position;
    auto hdg = this->getTrackerCmd()->heading;

    if (std::hypot(pos.x - trajectory.points[0].position.x, pos.y - trajectory.points[0].position.y, pos.z - trajectory.points[0].position.z) > POS_TOLERANCE) {
      ROS_ERROR("[%s]: trajectories differ at the translation of the initial condition", ros::this_node::getName().c_str());
      return false;
    }

    if (path.use_heading && abs(sradians::diff(trajectory.points[0].heading, hdg)) > HDG_TOLERANCE) {
      ROS_ERROR("[%s]: trajectories differ at the heading of the initial condition", ros::this_node::getName().c_str());
      return false;
    }
  }

  size_t waypoint_idx = 0;

  for (size_t i = 0; i < trajectory.points.size(); i++) {

    double points_dist = std::hypot(path.points[waypoint_idx].position.x - trajectory.points[i].position.x,
                                    path.points[waypoint_idx].position.y - trajectory.points[i].position.y,
                                    path.points[waypoint_idx].position.z - trajectory.points[i].position.z);

    double hdg_dist = 0;

    if (path.use_heading) {
      hdg_dist = abs(sradians::diff(path.points[waypoint_idx].heading, trajectory.points[i].heading));
    }

    if (points_dist < POS_TOLERANCE && hdg_dist < HDG_TOLERANCE) {
      waypoint_idx++;
    }
  }

  if (waypoint_idx == path.points.size()) {
    return true;
  }

  return false;
}

#endif  // GET_PATH_TEST_H
