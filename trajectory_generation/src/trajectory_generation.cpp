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
#include <mrs_lib/batch_visualizer.h>
#include <mrs_lib/scope_timer.h>

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

/* defines //{ */

typedef struct
{
  Eigen::Vector4d coords;
  bool            stop_at;
} Waypoint_t;

//}

namespace trajectory_generation
{

/* class TrajectoryGeneration //{ */
class TrajectoryGeneration : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool        is_initialized_ = false;
  std::string _uav_name_;

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

  mrs_lib::BatchVisualizer bw_original_;
  mrs_lib::BatchVisualizer bw_final_;

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

  bool optimize(const std::vector<Waypoint_t>& waypoints_in);

  // | ------------------ trajectory validation ----------------- |

  /**
   * @brief validates samples of a trajectory agains a path of waypoints
   *
   * @param trajectory
   * @param segments
   *
   * @return <success, traj_fail_idx, path_fail_segment>
   */
  std::tuple<bool, int, std::vector<bool>, double> validateTrajectory(const mav_msgs::EigenTrajectoryPoint::Vector& trajectory,
                                                                      const std::vector<Waypoint_t>&                waypoints);

  std::optional<mav_msgs::EigenTrajectoryPoint::Vector> findTrajectory(const std::vector<Waypoint_t>&   waypoints,
                                                                       const mrs_msgs::PositionCommand& initial_state);

  mrs_msgs::TrajectoryReference getTrajectoryReference(const mav_msgs::EigenTrajectoryPoint::Vector& trajectory, const ros::Time& stamp);

  Waypoint_t interpolatePoint(const Waypoint_t& a, const Waypoint_t& b, const double& coeff);

  double distFromSegment(const vec3_t& point, const vec3_t& seg1, const vec3_t& seg2);

  bool   _trajectory_max_segment_deviation_enabled_;
  double _trajectory_max_segment_deviation_;
  int    _trajectory_max_segment_deviation_max_iterations_;
  bool   _max_deviation_first_segment_;

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
  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("fly_now", fly_now_);
  param_loader.loadParam("frame_id", frame_id_);
  param_loader.loadParam("use_heading", use_heading_);
  param_loader.loadParam("stop_at_waypoints", stop_at_waypoints_);

  param_loader.loadParam("add_noise/enabled", _noise_enabled_);
  param_loader.loadParam("add_noise/max", _noise_max_);

  param_loader.loadParam("path_min_waypoint_distance", _path_min_waypoint_distance_);

  param_loader.loadParam("sampling_dt", _sampling_dt_);

  param_loader.loadParam("check_trajectory_deviation/enabled", _trajectory_max_segment_deviation_enabled_);
  param_loader.loadParam("check_trajectory_deviation/max_deviation", _trajectory_max_segment_deviation_);
  param_loader.loadParam("check_trajectory_deviation/first_segment", _max_deviation_first_segment_);
  param_loader.loadParam("check_trajectory_deviation/max_iterations", _trajectory_max_segment_deviation_max_iterations_);

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

  // | -------------------- batch visualizer -------------------- |

  bw_original_ = mrs_lib::BatchVisualizer(nh_, "markers/original", _uav_name_ + "/gps_origin");

  bw_original_.clearBuffers();
  bw_original_.clearVisuals();

  bw_final_ = mrs_lib::BatchVisualizer(nh_, "markers/final", _uav_name_ + "/gps_origin");

  bw_final_.clearBuffers();
  bw_final_.clearVisuals();

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

std::tuple<bool, int, std::vector<bool>, double> TrajectoryGeneration::validateTrajectory(const mav_msgs::EigenTrajectoryPoint::Vector& trajectory,
                                                                                          const std::vector<Waypoint_t>&                waypoints) {

  // prepare the output

  std::vector<bool> segments;
  for (size_t i = 0; i < waypoints.size() - 1; i++) {
    segments.push_back(true);
  }

  int waypoint_idx = 0;

  bool   is_safe       = true;
  double max_deviation = 0;

  for (size_t i = 0; i < trajectory.size() - 1; i++) {

    // the trajectory sample
    double sample_x = trajectory[i].position_W[0];
    double sample_y = trajectory[i].position_W[1];
    double sample_z = trajectory[i].position_W[2];
    vec3_t sample   = vec3_t(sample_x, sample_y, sample_z);

    // next sample
    double next_sample_x = trajectory[i + 1].position_W[0];
    double next_sample_y = trajectory[i + 1].position_W[1];
    double next_sample_z = trajectory[i + 1].position_W[2];
    vec3_t next_sample   = vec3_t(next_sample_x, next_sample_y, next_sample_z);

    // segment start
    double segment_start_x = waypoints.at(waypoint_idx).coords[0];
    double segment_start_y = waypoints.at(waypoint_idx).coords[1];
    double segment_start_z = waypoints.at(waypoint_idx).coords[2];
    vec3_t segment_start   = vec3_t(segment_start_x, segment_start_y, segment_start_z);

    // segment end
    double segment_end_x = waypoints.at(waypoint_idx + 1).coords[0];
    double segment_end_y = waypoints.at(waypoint_idx + 1).coords[1];
    double segment_end_z = waypoints.at(waypoint_idx + 1).coords[2];
    vec3_t segment_end   = vec3_t(segment_end_x, segment_end_y, segment_end_z);

    double distance_from_segment = distFromSegment(sample, segment_start, segment_end);

    double segment_end_dist = distFromSegment(segment_end, sample, next_sample);

    if (waypoint_idx > 0 || _max_deviation_first_segment_) {

      if (distance_from_segment > max_deviation) {
        max_deviation = distance_from_segment;
      }

      if (distance_from_segment > _trajectory_max_segment_deviation_) {
        segments.at(waypoint_idx) = false;
        is_safe                   = false;
      }
    }

    if (segment_end_dist < 0.05 && waypoint_idx < (int(waypoints.size()) - 2)) {
      waypoint_idx++;
    }
  }

  return std::tuple(is_safe, trajectory.size(), segments, max_deviation);
}

//}

/* findTrajectory() //{ */

std::optional<mav_msgs::EigenTrajectoryPoint::Vector> TrajectoryGeneration::findTrajectory(const std::vector<Waypoint_t>&   waypoints,
                                                                                           const mrs_msgs::PositionCommand& initial_state) {

  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer("findTrajectory()");

  ROS_INFO("[TrajectoryGeneration]: planning");

  mutex_params_.lock();
  auto params = params_;
  mutex_params_.unlock();

  mutex_constraints_.lock();
  auto constraints = constraints_;
  mutex_constraints_.unlock();

  // optimizer

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

  for (size_t i = 0; i < waypoints.size(); i++) {

    double x       = waypoints.at(i).coords[0];
    double y       = waypoints.at(i).coords[1];
    double z       = waypoints.at(i).coords[2];
    double heading = sradians::unwrap(waypoints.at(i).coords[3], last_heading);
    last_heading   = heading;

    mav_trajectory_generation::Vertex vertex(dimension);

    if (i == 0) {
      vertex.makeStartOrEnd(Eigen::Vector4d(x, y, z, heading), derivative_to_optimize);

      vertex.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector4d(x, y, z, heading));

      vertex.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                           Eigen::Vector4d(initial_state.velocity.x, initial_state.velocity.y, initial_state.velocity.z, initial_state.heading_rate));

      vertex.addConstraint(
          mav_trajectory_generation::derivative_order::ACCELERATION,
          Eigen::Vector4d(initial_state.acceleration.x, initial_state.acceleration.y, initial_state.acceleration.z, initial_state.heading_acceleration));

      vertex.addConstraint(mav_trajectory_generation::derivative_order::JERK,
                           Eigen::Vector4d(initial_state.jerk.x, initial_state.jerk.y, initial_state.jerk.z, initial_state.heading_jerk));
    } else if (i == (waypoints.size() - 1)) {  // the last point
      vertex.makeStartOrEnd(Eigen::Vector4d(x, y, z, heading), derivative_to_optimize);
    } else {  // mid points
      vertex.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector4d(x, y, z, heading));
      if (waypoints.at(i).stop_at) {
        vertex.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector4d(0, 0, 0, 0));
      }
    }

    vertices.push_back(vertex);
  }

  // | ---------------- compute the segment times --------------- |

  const double v_max = constraints.horizontal_speed;
  const double a_max = constraints.horizontal_acceleration;
  const double j_max = constraints.horizontal_jerk;

  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, v_max, a_max, j_max);

  /* for (int i = 0; i < int(segment_times.size()); i++) { */
  /*   ROS_INFO("[TrajectoryGeneration]: segment time %d = %.2f s", i, segment_times.at(i)); */
  /* } */

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

/* interpolatePoint() //{ */

Waypoint_t TrajectoryGeneration::interpolatePoint(const Waypoint_t& a, const Waypoint_t& b, const double& coeff) {

  Waypoint_t      out;
  Eigen::Vector4d diff = b.coords - a.coords;

  out.coords[0] = a.coords[0] + coeff * diff[0];
  out.coords[1] = a.coords[1] + coeff * diff[1];
  out.coords[2] = a.coords[2] + coeff * diff[2];
  out.coords[3] = radians::interp(a.coords[3], b.coords[3], coeff);

  out.stop_at = false;

  return out;
}

//}

/* optimize() //{ */

bool TrajectoryGeneration::optimize(const std::vector<Waypoint_t>& waypoints_in) {

  auto position_cmd = mrs_lib::get_mutexed(mutex_position_cmd_, position_cmd_);

  // visualizer

  bw_original_.clearBuffers();
  bw_original_.clearVisuals();

  bw_final_.clearBuffers();
  bw_final_.clearVisuals();

  /* copy the waypoints //{ */

  std::vector<Waypoint_t> waypoints;

  // add current position to the beginning
  Waypoint_t waypoint;
  waypoint.coords  = Eigen::Vector4d(position_cmd.position.x, position_cmd.position.y, position_cmd.position.z, position_cmd.heading);
  waypoint.stop_at = false;
  waypoints.push_back(waypoint);

  bw_original_.setPointsScale(0.8);
  bw_final_.setPointsScale(0.5);

  bw_original_.addPoint(vec3_t(position_cmd.position.x, position_cmd.position.y, position_cmd.position.z), 1.0, 0.0, 0.0, 1.0);

  for (int i = 0; i < int(waypoints_in.size()); i++) {

    Eigen::Vector4d last = waypoints.back().coords;

    double distance_from_last = mrs_lib::geometry::dist(vec3_t(_yaml_path_(i, 0), _yaml_path_(i, 1), _yaml_path_(i, 2)), vec3_t(last[0], last[1], last[2]));

    if (distance_from_last < _path_min_waypoint_distance_) {
      ROS_INFO("[TrajectoryGeneration]: skipping vertex, too close to the previous one");
      continue;
    }

    double x       = waypoints_in.at(i).coords[0] + randd(-_noise_max_, _noise_max_);
    double y       = waypoints_in.at(i).coords[1] + randd(-_noise_max_, _noise_max_);
    double z       = waypoints_in.at(i).coords[2] + randd(-_noise_max_, _noise_max_);
    double heading = waypoints_in.at(i).coords[3];

    bw_original_.addPoint(vec3_t(x, y, z), 1.0, 0.0, 0.0, 1.0);

    Waypoint_t wp;
    wp.coords  = Eigen::Vector4d(x, y, z, heading);
    wp.stop_at = waypoints_in.at(i).stop_at;
    waypoints.push_back(wp);
  }

  //}

  bool              safe = false;
  int               traj_idx;
  std::vector<bool> segment_safeness;
  double            max_deviation;

  mav_msgs::EigenTrajectoryPoint::Vector trajectory;

  auto result = findTrajectory(waypoints, position_cmd);

  if (result) {
    trajectory = result.value();
  } else {
    ROS_ERROR("[TrajectoryGeneration]: failed to find trajectory");
    return true;
  }

  for (int k = 0; k < _trajectory_max_segment_deviation_max_iterations_; k++) {

    ROS_INFO("[TrajectoryGeneration]: revalidation cycle #%d", k);

    std::tie(safe, traj_idx, segment_safeness, max_deviation) = validateTrajectory(trajectory, waypoints);

    if (_trajectory_max_segment_deviation_enabled_ && !safe) {

      ROS_INFO("[TrajectoryGeneration]: not safe, max deviation %.2f m", max_deviation);

      std::vector<Waypoint_t>::iterator waypoint = waypoints.begin();
      std::vector<bool>::iterator       safeness = segment_safeness.begin();

      for (; waypoint < waypoints.end() - 1; waypoint++) {

        if (!(*safeness)) {

          if (waypoint > waypoints.begin() || _max_deviation_first_segment_) {
            Waypoint_t midpoint2 = interpolatePoint(*waypoint, *(waypoint + 1), 0.5);
            waypoint             = waypoints.insert(waypoint + 1, midpoint2);
          }
        }

        safeness++;
      }

      auto result = findTrajectory(waypoints, position_cmd);

      if (result) {
        trajectory = result.value();
      } else {
        ROS_ERROR("[TrajectoryGeneration]: failed to find trajectory");
        return true;
      }

    } else {
      ROS_INFO("[TrajectoryGeneration]: trajectory is finally safe (%.2f)", max_deviation);
      safe = true;
      break;
    }
  }

  std::tie(safe, traj_idx, segment_safeness, max_deviation) = validateTrajectory(trajectory, waypoints);

  ROS_INFO("[TrajectoryGeneration]: final max deviation %.2f m", max_deviation);

  for (int i = 0; i < int(waypoints.size()); i++) {
    bw_final_.addPoint(vec3_t(waypoints.at(i).coords[0], waypoints.at(i).coords[1], waypoints.at(i).coords[2]), 0.0, 1.0, 0.0, 1.0);
  }

  mrs_msgs::TrajectoryReferenceSrv srv;

  srv.request.trajectory = getTrajectoryReference(trajectory, _max_deviation_first_segment_ ? ros::Time::now() : position_cmd.header.stamp);

  service_client_trajectory_reference_.call(srv);

  bw_original_.publish();
  bw_final_.publish();
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackTest() //{ */

bool TrajectoryGeneration::callbackTest([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer("callbackTest()");

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

  std::vector<Waypoint_t> waypoints;

  for (int i = 0; i < _yaml_path_.rows(); i++) {

    double x       = _yaml_path_(i, 0) + randd(-_noise_max_, _noise_max_);
    double y       = _yaml_path_(i, 1) + randd(-_noise_max_, _noise_max_);
    double z       = _yaml_path_(i, 2) + randd(-_noise_max_, _noise_max_);
    double heading = _yaml_path_(i, 3);

    Waypoint_t waypoint;
    waypoint.coords  = Eigen::Vector4d(x, y, z, heading);
    waypoint.stop_at = stop_at_waypoints_;

    waypoints.push_back(waypoint);
  }

  bool success = optimize(waypoints);

  res.success = success;
  res.message = "dog";
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

  std::vector<Waypoint_t> waypoints;

  for (size_t i = 0; i < msg->points.size(); i++) {

    double x       = msg->points[i].position.x;
    double y       = msg->points[i].position.y;
    double z       = msg->points[i].position.z;
    double heading = msg->points[i].heading;

    Waypoint_t wp;
    wp.coords  = Eigen::Vector4d(x, y, z, heading);
    wp.stop_at = stop_at_waypoints_;

    waypoints.push_back(wp);
  }

  fly_now_     = msg->fly_now;
  use_heading_ = msg->use_heading;
  frame_id_    = msg->header.frame_id;

  optimize(waypoints);
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

  std::vector<Waypoint_t> waypoints;

  for (size_t i = 0; i < req.path.points.size(); i++) {

    double x       = req.path.points[i].position.x;
    double y       = req.path.points[i].position.y;
    double z       = req.path.points[i].position.z;
    double heading = req.path.points[i].heading;

    Waypoint_t wp;
    wp.coords  = Eigen::Vector4d(x, y, z, heading);
    wp.stop_at = stop_at_waypoints_;

    waypoints.push_back(wp);
  }

  fly_now_     = req.path.fly_now;
  use_heading_ = req.path.use_heading;
  frame_id_    = req.path.header.frame_id;

  bool success = optimize(waypoints);

  res.message = success;
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
