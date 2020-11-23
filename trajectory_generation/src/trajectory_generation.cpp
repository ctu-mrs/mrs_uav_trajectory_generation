/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_msgs/TrajectoryReferenceSrv.h>
#include <mrs_msgs/TrajectoryReference.h>

#include <std_srvs/Trigger.h>

#include <stdlib.h>

#include <mrs_msgs/DynamicsConstraints.h>
#include <mrs_msgs/Path.h>
#include <mrs_msgs/PathSrv.h>
#include <mrs_msgs/PositionCommand.h>

#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

#include <mav_msgs/eigen_mav_msgs.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/mutex.h>

#include <mutex>

#include <dynamic_reconfigure/server.h>
#include <trajectory_generation/trajectory_generationConfig.h>

//}

/* using //{ */

using vec2_t = mrs_lib::geometry::vec_t<2>;
using vec3_t = mrs_lib::geometry::vec_t<3>;
using mat3_t = Eigen::Matrix3Xd;

using radians  = mrs_lib::geometry::radians;
using sradians = mrs_lib::geometry::sradians;

//}

namespace trajectory_generation
{

/* class TrajectoryGeneration //{ */
class TrajectoryGeneration : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool is_initialized_ = false;

  double randd(const double from, const double to);

  bool               callbackTest(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer service_server_test_;

  bool               callbackPathSrv(mrs_msgs::PathSrv::Request& req, mrs_msgs::PathSrv::Response& res);
  ros::ServiceServer service_server_path_;

  void                          callbackConstraints(const mrs_msgs::DynamicsConstraintsConstPtr& msg);
  ros::Subscriber               subscriber_constraints_;
  bool                          got_constraints_ = false;
  mrs_msgs::DynamicsConstraints constraints_;
  std::mutex                    mutex_constraints_;

  void                      callbackPositionCmd(const mrs_msgs::PositionCommandConstPtr& msg);
  ros::Subscriber           subscriber_position_cmd_;
  bool                      got_position_cmd_ = false;
  mrs_msgs::PositionCommand position_cmd_;
  std::mutex                mutex_position_cmd_;

  double _path_min_waypoint_distance_;

  double _sampling_dt_;

  Eigen::MatrixXd _yaml_path_;
  bool            fly_now_;
  bool            use_heading_;
  std::string     frame_id_;

  bool stop_at_waypoints_;

  bool   _noise_enabled_;
  double _noise_max_;

  void            callbackPath(const mrs_msgs::PathConstPtr& msg);
  ros::Subscriber subscriber_path_;

  ros::ServiceClient service_client_trajectory_reference_;

  // | ------------------ trajectory validation ----------------- |

  /**
   * @brief validates samples of a trajectory agains a path of waypoints
   *
   * @param trajectory
   * @param segments
   *
   * @return <success, traj_fail_idx, path_fail_segment>
   */
  std::tuple<bool, int, int> validateTrajectory(const mav_msgs::EigenTrajectoryPoint::Vector& trajectory, const std::vector<Eigen::Vector4d>& waypoints);

  std::optional<mav_msgs::EigenTrajectoryPoint::Vector> findTrajectory(const std::vector<Eigen::Vector4d>& waypoints,
                                                                       const mrs_msgs::PositionCommand&    initial_state);

  mrs_msgs::TrajectoryReference getTrajectoryReference(const mav_msgs::EigenTrajectoryPoint::Vector& trajectory, const ros::Time& stamp);

  double distFromSegment(const vec3_t& point, const vec3_t& seg1, const vec3_t& seg2);

  double _trajectory_max_segment_deviation_;

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                                     mutex_drs_;
  typedef trajectory_generation::trajectory_generationConfig DrsParams_t;
  typedef dynamic_reconfigure::Server<DrsParams_t>           Drs_t;
  boost::shared_ptr<Drs_t>                                   drs_;
  void                                                       callbackDrs(trajectory_generation::trajectory_generationConfig& params, uint32_t level);
  DrsParams_t                                                params_;
  std::mutex                                                 mutex_params_;
};
//}

/* onInit() //{ */

void TrajectoryGeneration::onInit() {

  /* obtain node handle */
  ros::NodeHandle nh_("~");

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ----------------------- subscribers ---------------------- |

  subscriber_constraints_  = nh_.subscribe("constraints_in", 1, &TrajectoryGeneration::callbackConstraints, this, ros::TransportHints().tcpNoDelay());
  subscriber_position_cmd_ = nh_.subscribe("position_cmd_in", 1, &TrajectoryGeneration::callbackPositionCmd, this, ros::TransportHints().tcpNoDelay());
  subscriber_path_         = nh_.subscribe("path_in", 1, &TrajectoryGeneration::callbackPath, this, ros::TransportHints().tcpNoDelay());

  // | --------------------- service servers -------------------- |

  service_server_test_ = nh_.advertiseService("test_in", &TrajectoryGeneration::callbackTest, this);
  service_server_path_ = nh_.advertiseService("path_in", &TrajectoryGeneration::callbackPathSrv, this);

  service_client_trajectory_reference_ = nh_.serviceClient<mrs_msgs::TrajectoryReferenceSrv>("trajectory_reference_out");

  // | ----------------------- parameters ----------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "TrajectoryGeneration");

  _yaml_path_ = param_loader.loadMatrixDynamic2("path", -1, 4);
  param_loader.loadParam("fly_now", fly_now_);
  param_loader.loadParam("frame_id", frame_id_);
  param_loader.loadParam("use_heading", use_heading_);
  param_loader.loadParam("stop_at_waypoints", stop_at_waypoints_);

  param_loader.loadParam("add_noise/enabled", _noise_enabled_);
  param_loader.loadParam("add_noise/max", _noise_max_);

  param_loader.loadParam("path_min_waypoint_distance", _path_min_waypoint_distance_);

  param_loader.loadParam("sampling_dt", _sampling_dt_);

  param_loader.loadParam("trajectory_max_segment_deviation", _trajectory_max_segment_deviation_);

  // | --------------------- service clients -------------------- |

  param_loader.loadParam("time_penalty", params_.time_penalty);
  param_loader.loadParam("soft_constraints_enabled", params_.soft_constraints_enabled);
  param_loader.loadParam("soft_constraints_weight", params_.soft_constraints_weight);
  param_loader.loadParam("time_allocation", params_.time_allocation);
  param_loader.loadParam("equality_constraint_tolerance", params_.equality_constraint_tolerance);
  param_loader.loadParam("inequality_constraint_tolerance", params_.inequality_constraint_tolerance);
  param_loader.loadParam("max_iterations", params_.max_iterations);
  param_loader.loadParam("derivative_to_optimize", params_.derivative_to_optimize);


  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ControlManager]: could not load all parameters!");
    ros::shutdown();
  }

  // | --------------- dynamic reconfigure server --------------- |

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(params_);
  Drs_t::CallbackType f = boost::bind(&TrajectoryGeneration::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | --------------------- finish the init -------------------- |

  ROS_INFO_ONCE("[TrajectoryGeneration]: initialized");

  is_initialized_ = true;
}

//}

/* //{ randd() */

double TrajectoryGeneration::randd(const double from, const double to) {

  if (!_noise_enabled_) {
    return 0;
  }

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return (to - from) * zero_to_one + from;
}

//}

/* distFromSegment() //{ */

double TrajectoryGeneration::distFromSegment(const vec3_t& point, const vec3_t& seg1, const vec3_t& seg2) {

  vec3_t segment_vector = seg2 - seg1;
  double segment_len    = segment_vector.norm();

  vec3_t segment_vector_norm = segment_vector;
  segment_vector_norm.normalize();

  double point_coordinate = segment_vector_norm.dot(point - seg1);

  if (point_coordinate < 0) {
    return (point - seg1).norm();
  } else if (point_coordinate > segment_len) {
    return (point - seg2).norm();
  } else {

    mat3_t segment_projector = segment_vector_norm * segment_vector_norm.transpose();
    vec3_t projection        = seg1 + segment_projector * (point - seg1);

    return (point - projection).norm();
  }
}

//}

/* validateTrajectory() //{ */

std::tuple<bool, int, int> TrajectoryGeneration::validateTrajectory(const mav_msgs::EigenTrajectoryPoint::Vector& trajectory,
                                                                    const std::vector<Eigen::Vector4d>&           waypoints) {

  int waypoint_idx = 0;

  for (size_t i = 0; i < trajectory.size(); i++) {

    // the trajectory sample
    double sample_x = trajectory[i].position_W[0];
    double sample_y = trajectory[i].position_W[1];
    double sample_z = trajectory[i].position_W[2];
    vec3_t sample   = vec3_t(sample_x, sample_y, sample_z);

    // segment start
    double segment_start_x = waypoints.at(waypoint_idx)[0];
    double segment_start_y = waypoints.at(waypoint_idx)[1];
    double segment_start_z = waypoints.at(waypoint_idx)[2];
    vec3_t segment_start   = vec3_t(segment_start_x, segment_start_y, segment_start_z);

    // segment end
    double segment_end_x = waypoints.at(waypoint_idx + 1)[0];
    double segment_end_y = waypoints.at(waypoint_idx + 1)[1];
    double segment_end_z = waypoints.at(waypoint_idx + 1)[2];
    vec3_t segment_end   = vec3_t(segment_end_x, segment_end_y, segment_end_z);

    double distance_from_segment = distFromSegment(sample, segment_start, segment_end);

    // try the next segment
    if ((waypoint_idx + 2) < int(waypoints.size())) {

      double distance_from_next_segment = std::numeric_limits<float>::max();

      double next_segment_end_x = waypoints.at(waypoint_idx + 2)[0];
      double next_segment_end_y = waypoints.at(waypoint_idx + 2)[1];
      double next_segment_end_z = waypoints.at(waypoint_idx + 2)[2];
      vec3_t next_segment_end   = vec3_t(next_segment_end_x, next_segment_end_y, next_segment_end_z);

      distance_from_next_segment = distFromSegment(sample, segment_end, next_segment_end);

      if (distance_from_next_segment < distance_from_segment) {

        waypoint_idx++;
        ROS_INFO("[TrajectoryGeneration]: waypoint++ %d", waypoint_idx);
        continue;
      }
    }

    if (distance_from_segment > _trajectory_max_segment_deviation_) {
      return std::tuple(false, i, waypoint_idx);
    }
  }

  return std::tuple(true, trajectory.size(), waypoint_idx);
}

//}

/* findTrajectory() //{ */

std::optional<mav_msgs::EigenTrajectoryPoint::Vector> TrajectoryGeneration::findTrajectory(const std::vector<Eigen::Vector4d>& waypoints,
                                                                                           const mrs_msgs::PositionCommand&    initial_state) {

  ROS_INFO("[TrajectoryGeneration]: planning");

  mutex_params_.lock();
  auto params = params_;
  mutex_params_.unlock();

  mutex_constraints_.lock();
  auto constraints = constraints_;
  mutex_constraints_.unlock();

  mav_trajectory_generation::NonlinearOptimizationParameters parameters;

  parameters.f_rel                  = 0.05;
  parameters.x_rel                  = 0.1;
  parameters.time_penalty           = params.time_penalty;
  parameters.use_soft_constraints   = params.soft_constraints_enabled;
  parameters.soft_constraint_weight = params.soft_constraints_weight;
  parameters.time_alloc_method      = static_cast<mav_trajectory_generation::NonlinearOptimizationParameters::TimeAllocMethod>(params.time_allocation);
  if (params.time_allocation == 2) {
    parameters.algorithm = nlopt::LD_LBFGS;
  }
  parameters.initial_stepsize_rel            = 0.1;
  parameters.inequality_constraint_tolerance = params.inequality_constraint_tolerance;
  parameters.equality_constraint_tolerance   = params.equality_constraint_tolerance;
  parameters.max_iterations                  = params.max_iterations;

  mav_trajectory_generation::Vertex::Vector vertices;
  const int                                 dimension = 4;

  int derivative_to_optimize;
  switch (params.derivative_to_optimize) {
    case 0: {
      derivative_to_optimize = mav_trajectory_generation::derivative_order::ACCELERATION;
      break;
    }
    case 1: {
      derivative_to_optimize = mav_trajectory_generation::derivative_order::JERK;
      break;
    }
    case 2: {
      derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
      break;
    }
  }

  // | --------------- add constraints to vertices -------------- |

  double last_heading = initial_state.heading;

  ROS_INFO("[TrajectoryGeneration]: processing waypoints");

  for (size_t i = 0; i < waypoints.size(); i++) {

    double x       = waypoints.at(i)[0];
    double y       = waypoints.at(i)[1];
    double z       = waypoints.at(i)[2];
    double heading = sradians::unwrap(waypoints.at(i)[3], last_heading);
    last_heading   = heading;

    mav_trajectory_generation::Vertex vertex(dimension);

    if (i == (waypoints.size() - 1)) {  // the last point
      vertex.makeStartOrEnd(Eigen::Vector4d(x, y, z, heading), derivative_to_optimize);
    } else {  // mid points
      vertex.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector4d(x, y, z, heading));
      if (stop_at_waypoints_) {
        vertex.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector4d(0, 0, 0, 0));
      }
    }

    vertices.push_back(vertex);
  }

  // | ---------------- compute the segment times --------------- |

  ROS_INFO("[TrajectoryGeneration]: computing segment times");

  const double v_max = constraints.horizontal_speed;
  const double a_max = constraints.horizontal_acceleration;
  const double j_max = constraints.horizontal_jerk;

  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, v_max, a_max, j_max);

  // | --------- create an optimizer object and solve it -------- |

  const int                                                     N = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::JERK, j_max);
  opt.optimize();

  // | ------------- obtain the polynomial segments ------------- |

  mav_trajectory_generation::Segment::Vector segments;
  opt.getPolynomialOptimizationRef().getSegments(&segments);

  // | --------------- create the trajectory class -------------- |

  mav_trajectory_generation::Trajectory trajectory;
  opt.getTrajectory(&trajectory);

  mav_msgs::EigenTrajectoryPoint::Vector states;
  bool                                   success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, _sampling_dt_, &states);

  if (success) {
    return std::optional(states);
  } else {
    return {};
  }
}

//}

/* getTrajectoryReference() //{ */

mrs_msgs::TrajectoryReference TrajectoryGeneration::getTrajectoryReference(const mav_msgs::EigenTrajectoryPoint::Vector& trajectory, const ros::Time& stamp) {

  mrs_msgs::TrajectoryReference msg;

  msg.header.frame_id = frame_id_;
  msg.header.stamp    = stamp;
  msg.fly_now         = fly_now_;
  msg.use_heading     = use_heading_;
  msg.dt              = _sampling_dt_;

  for (size_t it = 0; it < trajectory.size(); it++) {

    mrs_msgs::Reference point;
    point.heading    = 0;
    point.position.x = trajectory[it].position_W[0];
    point.position.y = trajectory[it].position_W[1];
    point.position.z = trajectory[it].position_W[2];
    point.heading    = trajectory[it].getYaw();

    msg.points.push_back(point);
  }

  return msg;
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackTest() //{ */

bool TrajectoryGeneration::callbackTest([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  /* preconditions //{ */

  if (!got_constraints_) {
    std::stringstream ss;
    ss << "missing constraints";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[TrajectoryGeneration]: " << ss.str());
    res.success = false;
    res.message = ss.str();
  }

  if (!got_position_cmd_) {
    std::stringstream ss;
    ss << "missing position cmd";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[TrajectoryGeneration]: " << ss.str());
    res.success = false;
    res.message = ss.str();
  }

  //}

  auto position_cmd = mrs_lib::get_mutexed(mutex_position_cmd_, position_cmd_);

  /* copy the waypoints //{ */

  std::vector<Eigen::Vector4d> waypoints;

  // add current position to the beginning
  waypoints.push_back(Eigen::Vector4d(position_cmd.position.x, position_cmd.position.y, position_cmd.position.z, position_cmd.heading));

  for (int i = 0; i < _yaml_path_.rows(); i++) {

    Eigen::Vector4d last = waypoints.back();

    double distance_from_last = mrs_lib::geometry::dist(vec3_t(_yaml_path_(i, 0), _yaml_path_(i, 1), _yaml_path_(i, 2)), vec3_t(last[0], last[1], last[2]));

    if (distance_from_last < _path_min_waypoint_distance_) {
      ROS_INFO("[TrajectoryGeneration]: skipping vertex, too close to the previous one");
      continue;
    }

    double x       = _yaml_path_(i, 0);
    double y       = _yaml_path_(i, 1);
    double z       = _yaml_path_(i, 2);
    double heading = _yaml_path_(i, 3);

    waypoints.push_back(Eigen::Vector4d(x, y, z, heading));
  }

  //}

  bool safe = false;
  int  traj_idx;
  int  way_idx;

  mav_msgs::EigenTrajectoryPoint::Vector trajectory;

  for (int k = 0; k < 30; k++) {

    for (size_t it = 0; it < waypoints.size(); it++) {
      ROS_INFO("[TrajectoryGeneration]: waypoint %d, x %.2f, y %.2f z %.2f hdg %.2f", int(it), waypoints.at(it)[0], waypoints.at(it)[1], waypoints.at(it)[2],
               waypoints.at(it)[3]);
    }

    auto result = findTrajectory(waypoints, position_cmd);

    if (result) {
      trajectory = result.value();
    } else {
      ROS_ERROR("[TrajectoryGeneration]: failed to find trajectory");
      return true;
    }

    std::tie(safe, traj_idx, way_idx) = validateTrajectory(trajectory, waypoints);

    if (!safe) {

      Eigen::Vector4d midpoint;
      midpoint[0] = (waypoints[way_idx][0] + waypoints[way_idx + 1][0]) / 2.0;
      midpoint[1] = (waypoints[way_idx][1] + waypoints[way_idx + 1][1]) / 2.0;
      midpoint[2] = (waypoints[way_idx][2] + waypoints[way_idx + 1][2]) / 2.0;
      midpoint[3] = radians::interp(waypoints[way_idx][3], waypoints[way_idx + 1][3], 0.5);

      ROS_INFO("[TrajectoryGeneration]: inserting midpoint between %d and %d", way_idx, way_idx + 1);

      waypoints.insert(waypoints.begin() + way_idx + 1, midpoint);

    } else {
      safe = true;
      break;
    }
  }

  /* ROS_INFO("[TrajectoryGeneration]: safe: %d, traj id: %d, waypoint: %d", safe, traj_idx, way_idx); */

  mrs_msgs::TrajectoryReferenceSrv srv;

  srv.request.trajectory = getTrajectoryReference(trajectory, position_cmd.header.stamp);

  service_client_trajectory_reference_.call(srv);

  return true;
}

//}

/* callbackPath() //{ */

void TrajectoryGeneration::callbackPath(const mrs_msgs::PathConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  if (!got_constraints_) {
    std::stringstream ss;
    ss << "missing constraints";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[TrajectoryGeneration]: " << ss.str());
    return;
  }

  if (!got_position_cmd_) {
    std::stringstream ss;
    ss << "missing position cmd";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[TrajectoryGeneration]: " << ss.str());
    return;
  }

  ROS_INFO("[TrajectoryGeneration]: got trajectory");
}

//}

/* callbackPathSrv() //{ */

bool TrajectoryGeneration::callbackPathSrv(mrs_msgs::PathSrv::Request& req, mrs_msgs::PathSrv::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  if (!got_constraints_) {
    std::stringstream ss;
    ss << "missing constraints";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[TrajectoryGeneration]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  if (!got_position_cmd_) {
    std::stringstream ss;
    ss << "missing position cmd";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[TrajectoryGeneration]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  ROS_INFO("[TrajectoryGeneration]: got trajectory");

  res.message = "set";
  res.success = true;
  return true;
}

//}

/* callbackConstraints() //{ */

void TrajectoryGeneration::callbackConstraints(const mrs_msgs::DynamicsConstraintsConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[TrajectoryGeneration]: got constraints");

  got_constraints_ = true;

  mutex_constraints_.lock();
  constraints_ = *msg;
  mutex_constraints_.unlock();
}

//}

/* callbackPositionCmd() //{ */

void TrajectoryGeneration::callbackPositionCmd(const mrs_msgs::PositionCommandConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[TrajectoryGeneration]: got position cmd");

  got_position_cmd_ = true;

  mutex_position_cmd_.lock();
  position_cmd_ = *msg;
  mutex_position_cmd_.unlock();
}

//}

/* //{ callbackDrs() */

void TrajectoryGeneration::callbackDrs(trajectory_generation::trajectory_generationConfig& params, [[maybe_unused]] uint32_t level) {

  mutex_params_.lock();
  params_ = params;
  mutex_params_.unlock();

  if (params.test) {
    params.test = false;

    drs_->updateConfig(params_);

    std_srvs::Trigger::Request  req;
    std_srvs::Trigger::Response res;

    callbackTest(req, res);
  }

  ROS_INFO("[TrajectoryGeneration]: DRS updated");
}

//}

}  // namespace trajectory_generation

/* every nodelet must export its class as nodelet plugin */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(trajectory_generation::TrajectoryGeneration, nodelet::Nodelet);
