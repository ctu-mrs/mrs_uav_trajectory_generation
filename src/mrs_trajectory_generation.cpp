/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_msgs/TrajectoryReferenceSrv.h>
#include <mrs_msgs/TrajectoryReference.h>

#include <std_srvs/Trigger.h>

#include <geometry_msgs/Vector3Stamped.h>

#include <mrs_msgs/DynamicsConstraints.h>
#include <mrs_msgs/Path.h>
#include <mrs_msgs/PathSrv.h>
#include <mrs_msgs/GetPathSrv.h>
#include <mrs_msgs/PositionCommand.h>
#include <mrs_msgs/MpcPredictionFullState.h>
#include <mrs_msgs/Reference.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>

#include <eth_trajectory_generation/impl/polynomial_optimization_nonlinear_impl.h>
#include <eth_trajectory_generation/trajectory.h>
#include <eth_trajectory_generation/trajectory_sampling.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/batch_visualizer.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/utils.h>
#include <mrs_lib/scope_timer.h>

#include <dynamic_reconfigure/server.h>
#include <mrs_uav_trajectory_generation/drsConfig.h>

#include <future>

//}

/* using //{ */

using vec2_t = mrs_lib::geometry::vec_t<2>;
using vec3_t = mrs_lib::geometry::vec_t<3>;
using mat3_t = Eigen::Matrix3Xd;

using radians  = mrs_lib::geometry::radians;
using sradians = mrs_lib::geometry::sradians;

//}

/* defines //{ */

#define FUTURIZATION_EXEC_TIME_FACTOR 0.25       // [0, 1]
#define FUTURIZATION_FIRST_WAYPOINT_FACTOR 0.50  // [0, 1]
#define OVERTIME_SAFETY_FACTOR 0.95              // [0, 1]
#define OVERTIME_SAFETY_OFFSET 0.01              // [s]
#define NLOPT_EXEC_TIME_FACTOR 0.95              // [0, 1]

typedef struct
{
  Eigen::Vector4d coords;
  bool            stop_at;
} Waypoint_t;

//}

namespace mrs_uav_trajectory_generation
{

/* class MrsTrajectoryGeneration //{ */
class MrsTrajectoryGeneration : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool is_initialized_ = false;

  // | ----------------------- parameters ----------------------- |

  // path for debuggins
  Eigen::MatrixXd _yaml_path_;

  double _sampling_dt_;

  double _max_trajectory_len_factor_;
  double _min_trajectory_len_factor_;

  int _n_attempts_;

  double _min_waypoint_distance_;

  bool   _fallback_sampling_enabled_;
  double _fallback_sampling_speed_factor_;
  double _fallback_sampling_accel_factor_;
  double _fallback_sampling_stopping_time_;

  std::string _uav_name_;

  bool   _trajectory_max_segment_deviation_enabled_;
  double _trajectory_max_segment_deviation_;
  int    _trajectory_max_segment_deviation_max_iterations_;

  bool   _path_straightener_enabled_;
  double _path_straightener_max_deviation_;
  double _path_straightener_max_hdg_deviation_;

  // | -------- variable parameters (come with the path) -------- |

  std::string frame_id_;
  bool        fly_now_                              = false;
  bool        use_heading_                          = false;
  bool        stop_at_waypoints_                    = false;
  bool        override_constraints_                 = false;
  bool        loop_                                 = false;
  double      override_max_velocity_horizontal_     = false;
  double      override_max_velocity_vertical_       = false;
  double      override_max_acceleration_horizontal_ = false;
  double      override_max_acceleration_vertical_   = false;

  // | -------------- variable parameters (deduced) ------------- |

  double     max_execution_time_ = 0;
  std::mutex mutex_max_execution_time_;

  bool max_deviation_first_segment_;

  // | -------------------- the transformer  -------------------- |

  std::shared_ptr<mrs_lib::Transformer> transformer_;

  // service client for input
  bool               callbackPathSrv(mrs_msgs::PathSrv::Request& req, mrs_msgs::PathSrv::Response& res);
  ros::ServiceServer service_server_path_;

  // service client for returning result to the user
  bool               callbackGetPathSrv(mrs_msgs::GetPathSrv::Request& req, mrs_msgs::GetPathSrv::Response& res);
  ros::ServiceServer service_server_get_path_;

  // subscriber for input
  void            callbackPath(const mrs_msgs::PathConstPtr& msg);
  ros::Subscriber subscriber_path_;

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

  void                             callbackPredictionFullState(const mrs_msgs::MpcPredictionFullStateConstPtr& msg);
  ros::Subscriber                  subscriber_prediction_full_state_;
  bool                             got_prediction_full_state_ = false;
  mrs_msgs::MpcPredictionFullState prediction_full_state_;
  std::mutex                       mutex_prediction_full_state_;

  void                                callbackControlManagerDiag(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg);
  ros::Subscriber                     subscriber_control_manager_diag_;
  bool                                got_control_manager_diag_ = false;
  mrs_msgs::ControlManagerDiagnostics control_manager_diag_;
  std::mutex                          mutex_control_manager_diag_;

  // service client for publishing trajectory out
  ros::ServiceClient service_client_trajectory_reference_;

  // solve the whole problem
  std::tuple<bool, std::string, mrs_msgs::TrajectoryReference> optimize(const std::vector<Waypoint_t>& waypoints_in, const std_msgs::Header& waypoints_stamp,
                                                                        const mrs_msgs::PositionCommand&        initial_condition,
                                                                        const mrs_msgs::MpcPredictionFullState& current_prediction, bool fallback_sampling,
                                                                        const bool relax_heading);

  // batch vizualizer
  mrs_lib::BatchVisualizer bw_original_;
  mrs_lib::BatchVisualizer bw_final_;

  // transforming PositionCommand
  std::optional<mrs_msgs::PositionCommand> transformPositionCmd(const mrs_msgs::PositionCommand& position_cmd, const std::string& target_frame);

  // | ------------------ trajectory validation ----------------- |

  /**
   * @brief validates samples of a trajectory agains a path of waypoints
   *
   * @param trajectory
   * @param segments
   *
   * @return <success, traj_fail_idx, path_fail_segment>
   */
  std::tuple<bool, int, std::vector<bool>, double> validateTrajectorySpatial(const eth_mav_msgs::EigenTrajectoryPoint::Vector& trajectory,
                                                                             const std::vector<Waypoint_t>&                    waypoints);

  std::vector<int> getTrajectorySegmenCenterIdxs(const eth_mav_msgs::EigenTrajectoryPoint::Vector& trajectory, const std::vector<Waypoint_t>& waypoints);

  std::optional<eth_mav_msgs::EigenTrajectoryPoint::Vector> findTrajectory(const std::vector<Waypoint_t>&   waypoints,
                                                                           const mrs_msgs::PositionCommand& initial_state, const double& sampling_dt,
                                                                           const bool& relax_heading);

  std::optional<eth_mav_msgs::EigenTrajectoryPoint::Vector>              findTrajectoryFallback(const std::vector<Waypoint_t>&   waypoints,
                                                                                                const mrs_msgs::PositionCommand& initial_state, const double& sampling_dt,
                                                                                                const bool& relax_heading);
  std::future<std::optional<eth_mav_msgs::EigenTrajectoryPoint::Vector>> future_trajectory_result_;
  std::atomic<bool>                                                      running_async_planning_ = false;

  std::optional<eth_mav_msgs::EigenTrajectoryPoint::Vector> findTrajectoryAsync(const std::vector<Waypoint_t>&   waypoints,
                                                                                const mrs_msgs::PositionCommand& initial_state, const double& sampling_dt,
                                                                                const bool& relax_heading);

  std::vector<Waypoint_t> preprocessPath(const std::vector<Waypoint_t>& waypoints_in);

  mrs_msgs::TrajectoryReference getTrajectoryReference(const eth_mav_msgs::EigenTrajectoryPoint::Vector& trajectory, const ros::Time& stamp,
                                                       const double& sampling_dt);

  Waypoint_t interpolatePoint(const Waypoint_t& a, const Waypoint_t& b, const double& coeff);

  bool checkNaN(const Waypoint_t& a);

  double distFromSegment(const vec3_t& point, const vec3_t& seg1, const vec3_t& seg2);

  bool trajectorySrv(const mrs_msgs::TrajectoryReference& msg);

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                           mutex_drs_;
  typedef mrs_uav_trajectory_generation::drsConfig DrsParams_t;
  typedef dynamic_reconfigure::Server<DrsParams_t> Drs_t;
  boost::shared_ptr<Drs_t>                         drs_;
  void                                             callbackDrs(mrs_uav_trajectory_generation::drsConfig& params, uint32_t level);
  DrsParams_t                                      params_;
  std::mutex                                       mutex_params_;

  // | ------------ Republisher for the desired path ------------ |

  ros::Publisher publisher_original_path_;

  // | ------------- measuring the time of execution ------------ |

  ros::Time  start_time_total_;
  std::mutex mutex_start_time_total_;
  bool       overtime(void);
  double     timeLeft(void);
};

//}

/* onInit() //{ */

void MrsTrajectoryGeneration::onInit() {

  /* obtain node handle */
  ros::NodeHandle nh_("~");

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ----------------------- publishers ----------------------- |

  publisher_original_path_ = nh_.advertise<mrs_msgs::Path>("original_path_out", 1);

  // | ----------------------- subscribers ---------------------- |

  subscriber_constraints_  = nh_.subscribe("constraints_in", 1, &MrsTrajectoryGeneration::callbackConstraints, this, ros::TransportHints().tcpNoDelay());
  subscriber_position_cmd_ = nh_.subscribe("position_cmd_in", 1, &MrsTrajectoryGeneration::callbackPositionCmd, this, ros::TransportHints().tcpNoDelay());
  subscriber_prediction_full_state_ =
      nh_.subscribe("prediction_full_state_in", 1, &MrsTrajectoryGeneration::callbackPredictionFullState, this, ros::TransportHints().tcpNoDelay());
  subscriber_control_manager_diag_ =
      nh_.subscribe("control_manager_diag_in", 1, &MrsTrajectoryGeneration::callbackControlManagerDiag, this, ros::TransportHints().tcpNoDelay());
  subscriber_path_ = nh_.subscribe("path_in", 1, &MrsTrajectoryGeneration::callbackPath, this, ros::TransportHints().tcpNoDelay());

  // | --------------------- service servers -------------------- |

  service_server_path_ = nh_.advertiseService("path_in", &MrsTrajectoryGeneration::callbackPathSrv, this);

  service_server_get_path_ = nh_.advertiseService("get_path_in", &MrsTrajectoryGeneration::callbackGetPathSrv, this);

  service_client_trajectory_reference_ = nh_.serviceClient<mrs_msgs::TrajectoryReferenceSrv>("trajectory_reference_out");

  // | ----------------------- parameters ----------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "MrsTrajectoryGeneration");

  _yaml_path_ = param_loader.loadMatrixDynamic2("path", -1, 4);

  param_loader.loadParam("uav_name", _uav_name_);

  param_loader.loadParam("sampling_dt", _sampling_dt_);

  param_loader.loadParam("enforce_fallback_solver", params_.enforce_fallback_solver);

  param_loader.loadParam("max_trajectory_len_factor", _max_trajectory_len_factor_);
  param_loader.loadParam("min_trajectory_len_factor", _min_trajectory_len_factor_);

  param_loader.loadParam("n_attempts", _n_attempts_);
  param_loader.loadParam("fallback_sampling/enabled", _fallback_sampling_enabled_);
  param_loader.loadParam("fallback_sampling/speed_factor", _fallback_sampling_speed_factor_);
  param_loader.loadParam("fallback_sampling/accel_factor", _fallback_sampling_accel_factor_);
  param_loader.loadParam("fallback_sampling/stopping_time", _fallback_sampling_stopping_time_);

  param_loader.loadParam("check_trajectory_deviation/enabled", _trajectory_max_segment_deviation_enabled_);
  param_loader.loadParam("check_trajectory_deviation/max_deviation", _trajectory_max_segment_deviation_);
  param_loader.loadParam("check_trajectory_deviation/max_iterations", _trajectory_max_segment_deviation_max_iterations_);

  param_loader.loadParam("path_straightener/enabled", _path_straightener_enabled_);
  param_loader.loadParam("path_straightener/max_deviation", _path_straightener_max_deviation_);
  param_loader.loadParam("path_straightener/max_hdg_deviation", _path_straightener_max_hdg_deviation_);

  param_loader.loadParam("min_waypoint_distance", _min_waypoint_distance_);

  // | --------------------- tf transformer --------------------- |

  transformer_ = std::make_shared<mrs_lib::Transformer>("TrajectoryGeneration", _uav_name_);

  // | --------------------- service clients -------------------- |

  param_loader.loadParam("time_penalty", params_.time_penalty);
  param_loader.loadParam("soft_constraints_enabled", params_.soft_constraints_enabled);
  param_loader.loadParam("soft_constraints_weight", params_.soft_constraints_weight);
  param_loader.loadParam("time_allocation", params_.time_allocation);
  param_loader.loadParam("equality_constraint_tolerance", params_.equality_constraint_tolerance);
  param_loader.loadParam("inequality_constraint_tolerance", params_.inequality_constraint_tolerance);
  param_loader.loadParam("max_iterations", params_.max_iterations);
  param_loader.loadParam("derivative_to_optimize", params_.derivative_to_optimize);

  param_loader.loadParam("max_time", params_.max_time);
  max_execution_time_ = params_.max_time;

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[MrsTrajectoryGeneration]: could not load all parameters!");
    ros::requestShutdown();
  }

  // | -------------------- batch visualizer -------------------- |

  bw_original_ = mrs_lib::BatchVisualizer(nh_, "markers/original", "");

  bw_original_.clearBuffers();
  bw_original_.clearVisuals();

  bw_final_ = mrs_lib::BatchVisualizer(nh_, "markers/final", "");

  bw_final_.clearBuffers();
  bw_final_.clearVisuals();

  // | --------------- dynamic reconfigure server --------------- |

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(params_);
  Drs_t::CallbackType f = boost::bind(&MrsTrajectoryGeneration::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | --------------------- finish the init -------------------- |

  ROS_INFO_ONCE("[MrsTrajectoryGeneration]: initialized");

  is_initialized_ = true;
}

//}

// | ---------------------- main routines --------------------- |

/*
 * 1. preprocessPath(): preprocessing the incoming path
 *    - throughs away too close waypoints
 *    - straightness path by neglecting waypoints close to segments
 * 2. optimize(): solves the whole problem including
 *    - subdivision for satisfying max deviation
 * 3. findTrajectory(): solves single instance by the ETH tool
 * 4. findTrajectoryFallback(): Baca's sampling for backup solution
 * 5. validateTrajectorySpatial(): checks for the spatial soundness of a trajectory vs. the original path
 */

/* preprocessPath() //{ */

std::vector<Waypoint_t> MrsTrajectoryGeneration::preprocessPath(const std::vector<Waypoint_t>& waypoints_in) {

  std::vector<Waypoint_t> waypoints;

  size_t last_added_idx = 0;  // in "waypoints_in"

  for (size_t i = 0; i < waypoints_in.size(); i++) {

    double x       = waypoints_in.at(i).coords[0];
    double y       = waypoints_in.at(i).coords[1];
    double z       = waypoints_in.at(i).coords[2];
    double heading = waypoints_in.at(i).coords[3];

    bw_original_.addPoint(vec3_t(x, y, z), 1.0, 0.0, 0.0, 1.0);

    if (_path_straightener_enabled_ && waypoints_in.size() >= 3 && i > 0 && i < (waypoints_in.size() - 1)) {

      vec3_t first(waypoints_in.at(last_added_idx).coords[0], waypoints_in.at(last_added_idx).coords[1], waypoints_in.at(last_added_idx).coords[2]);
      vec3_t last(waypoints_in.at(i + 1).coords[0], waypoints_in.at(i + 1).coords[1], waypoints_in.at(i + 1).coords[2]);

      double first_hdg = waypoints_in.at(last_added_idx).coords[3];
      double last_hdg  = waypoints_in.at(i + 1).coords[3];

      size_t next_point = last_added_idx + 1;

      bool segment_is_ok = true;

      for (size_t j = next_point; j < i + 1; j++) {

        vec3_t mid(waypoints_in.at(j).coords[0], waypoints_in.at(j).coords[1], waypoints_in.at(j).coords[2]);
        double mid_hdg = waypoints_in.at(j).coords[3];

        double dist_from_segment = distFromSegment(mid, first, last);

        if (dist_from_segment > _path_straightener_max_deviation_ || fabs(radians::diff(first_hdg, mid_hdg) > _path_straightener_max_hdg_deviation_) ||
            fabs(radians::diff(last_hdg, mid_hdg) > _path_straightener_max_hdg_deviation_)) {
          segment_is_ok = false;
          break;
        }
      }

      if (segment_is_ok) {
        continue;
      }
    }

    if (i > 0 && i < (waypoints_in.size() - 1)) {

      vec3_t first(waypoints_in.at(last_added_idx).coords[0], waypoints_in.at(last_added_idx).coords[1], waypoints_in.at(last_added_idx).coords[2]);
      vec3_t last(waypoints_in.at(i).coords[0], waypoints_in.at(i).coords[1], waypoints_in.at(i).coords[2]);

      if (mrs_lib::geometry::dist(first, last) < _min_waypoint_distance_) {
        ROS_INFO("[MrsTrajectoryGeneration]: waypoint #%d too close (< %.3f m) to the previous one (#%d), throwing it away", int(i), _min_waypoint_distance_,
                 int(last_added_idx));
        continue;
      }
    }

    Waypoint_t wp;
    wp.coords  = Eigen::Vector4d(x, y, z, heading);
    wp.stop_at = waypoints_in.at(i).stop_at;
    waypoints.push_back(wp);

    last_added_idx = i;
  }

  return waypoints;
}

//}

/* optimize() //{ */

std::tuple<bool, std::string, mrs_msgs::TrajectoryReference> MrsTrajectoryGeneration::optimize(const std::vector<Waypoint_t>&          waypoints_in,
                                                                                               const std_msgs::Header&                 waypoints_header,
                                                                                               const mrs_msgs::PositionCommand&        position_cmd,
                                                                                               const mrs_msgs::MpcPredictionFullState& current_prediction,
                                                                                               const bool fallback_sampling, const bool relax_heading) {

  /* mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("optimize()"); */

  ros::Time optimize_time_start = ros::Time::now();

  // | ------------- prepare the initial conditions ------------- |

  mrs_msgs::PositionCommand initial_condition;

  bool path_from_future = false;

  // positive = in the future
  double path_time_offset = 0;

  if (waypoints_header.stamp != ros::Time(0)) {
    path_time_offset = (waypoints_header.stamp - ros::Time::now()).toSec();
  }

  int path_sample_offset = 0;

  // if the desired path starts in the future, more than one MPC step ahead
  if (path_time_offset > 0.2) {

    ROS_INFO("[MrsTrajectoryGeneration]: desired path is from the future by %.2f s", path_time_offset);

    // calculate the offset in samples in the predicted trajectory
    // 0.01 is subtracted for the first sample, which is smaller
    // +1 is added due to the first sample, which was subtarcted
    path_sample_offset = int(ceil((path_time_offset * FUTURIZATION_FIRST_WAYPOINT_FACTOR - 0.01) / 0.2)) + 1;

    if (path_sample_offset > (int(current_prediction.position.size()) - 1)) {

      ROS_ERROR("[MrsTrajectoryGeneration]: can not extrapolate into the waypoints, using position_cmd instead");
      initial_condition = position_cmd;

    } else {

      ROS_INFO("[MrsTrajectoryGeneration]: getting initial condition from the %d-th sample of the MPC prediction", path_sample_offset);

      initial_condition.header.stamp    = current_prediction.header.stamp + ros::Duration(0.01 + 0.2 * path_sample_offset);
      initial_condition.header.frame_id = current_prediction.header.frame_id;

      initial_condition.position     = current_prediction.position[path_sample_offset];
      initial_condition.velocity     = current_prediction.velocity[path_sample_offset];
      initial_condition.acceleration = current_prediction.acceleration[path_sample_offset];
      initial_condition.jerk         = current_prediction.jerk[path_sample_offset];

      initial_condition.heading              = current_prediction.heading[path_sample_offset];
      initial_condition.heading_rate         = current_prediction.heading_rate[path_sample_offset];
      initial_condition.heading_acceleration = current_prediction.heading_acceleration[path_sample_offset];
      initial_condition.heading_jerk         = current_prediction.heading_jerk[path_sample_offset];

      path_from_future = true;
    }

  } else {

    initial_condition = position_cmd;
  }

  // | ---- transform the inital condition to the path frame ---- |

  auto res = transformPositionCmd(initial_condition, waypoints_header.frame_id);

  if (res) {
    initial_condition = res.value();
  } else {
    std::stringstream ss;
    ss << "could not transform initial condition to the path frame";
    ROS_ERROR_STREAM("[MrsTrajectoryGeneration]: " << ss.str());
    return std::tuple(false, ss.str(), mrs_msgs::TrajectoryReference());
  }

  auto control_manager_diag = mrs_lib::get_mutexed(mutex_control_manager_diag_, control_manager_diag_);

  if (waypoints_header.stamp == ros::Time(0)) {
    if (!control_manager_diag.tracker_status.have_goal) {
      initial_condition.header.stamp = ros::Time(0);
    }
  }

  // | ---------------- reset the visual markers ---------------- |

  bw_original_.clearBuffers();
  bw_original_.clearVisuals();
  bw_final_.clearBuffers();
  bw_final_.clearVisuals();

  bw_original_.setParentFrame(transformer_->resolveFrameName(frame_id_));
  bw_final_.setParentFrame(transformer_->resolveFrameName(frame_id_));

  bw_original_.setPointsScale(0.4);
  bw_final_.setPointsScale(0.35);

  // empty path is invalid
  if (waypoints_in.size() == 0) {
    std::stringstream ss;
    ss << "the path is empty (before postprocessing)";
    ROS_ERROR_STREAM("[MrsTrajectoryGeneration]: " << ss.str());
    return std::tuple(false, ss.str(), mrs_msgs::TrajectoryReference());
  }

  std::vector<Waypoint_t> waypoints_in_with_init = waypoints_in;

  // prepend the initial condition
  Waypoint_t initial_waypoint;
  initial_waypoint.coords =
      Eigen::Vector4d(initial_condition.position.x, initial_condition.position.y, initial_condition.position.z, initial_condition.heading);
  initial_waypoint.stop_at = false;
  waypoints_in_with_init.insert(waypoints_in_with_init.begin(), initial_waypoint);

  std::vector<Waypoint_t> waypoints = preprocessPath(waypoints_in_with_init);

  if (waypoints.size() <= 1) {
    std::stringstream ss;
    ss << "the path is empty (after postprocessing)";
    ROS_ERROR_STREAM("[MrsTrajectoryGeneration]: " << ss.str());
    return std::tuple(false, ss.str(), mrs_msgs::TrajectoryReference());
  }

  bool              safe = false;
  int               traj_idx;
  std::vector<bool> segment_safeness;
  double            max_deviation;

  eth_mav_msgs::EigenTrajectoryPoint::Vector trajectory;

  double sampling_dt = 0;

  if (path_from_future) {
    ROS_INFO("[MrsTrajectoryGeneration]: changing dt = 0.2, cause the path is from the future");
    sampling_dt = 0.2;
  } else {
    sampling_dt = _sampling_dt_;
  }

  std::optional<eth_mav_msgs::EigenTrajectoryPoint::Vector> result;

  auto params = mrs_lib::get_mutexed(mutex_params_, params_);

  if (params.enforce_fallback_solver) {
    ROS_WARN("[MrsTrajectoryGeneration]: fallback sampling enforced");
    result = findTrajectoryFallback(waypoints, initial_condition, sampling_dt, relax_heading);
  } else if (fallback_sampling) {
    ROS_WARN("[MrsTrajectoryGeneration]: executing fallback sampling");
    result = findTrajectoryFallback(waypoints, initial_condition, sampling_dt, relax_heading);
  } else if (running_async_planning_) {
    ROS_WARN("[MrsTrajectoryGeneration]: executing fallback sampling, the previous async task is still running");
    result = findTrajectoryFallback(waypoints, initial_condition, sampling_dt, relax_heading);
  } else if (overtime()) {
    ROS_WARN("[MrsTrajectoryGeneration]: executing fallback sampling, we are running over time");
    result = findTrajectoryFallback(waypoints, initial_condition, sampling_dt, relax_heading);
  } else {
    result = findTrajectoryAsync(waypoints, initial_condition, sampling_dt, relax_heading);
  }

  if (result) {
    trajectory = result.value();
  } else {
    std::stringstream ss;
    ss << "failed to find trajectory";
    ROS_ERROR_STREAM("[MrsTrajectoryGeneration]: " << ss.str());
    return std::tuple(false, ss.str(), mrs_msgs::TrajectoryReference());
  }

  for (int k = 0; k < _trajectory_max_segment_deviation_max_iterations_; k++) {

    ROS_DEBUG("[MrsTrajectoryGeneration]: revalidation cycle #%d", k);

    std::tie(safe, traj_idx, segment_safeness, max_deviation) = validateTrajectorySpatial(trajectory, waypoints);

    if (_trajectory_max_segment_deviation_enabled_ && !safe) {

      ROS_DEBUG("[MrsTrajectoryGeneration]: trajectory is not safe, max deviation %.3f m", max_deviation);

      std::vector<Waypoint_t>::iterator waypoint = waypoints.begin();
      std::vector<bool>::iterator       safeness = segment_safeness.begin();

      for (; waypoint < waypoints.end() - 1; waypoint++) {

        if (!(*safeness)) {

          if (waypoint > waypoints.begin() || max_deviation_first_segment_ || int(waypoints.size()) <= 2) {
            Waypoint_t midpoint2 = interpolatePoint(*waypoint, *(waypoint + 1), 0.5);
            waypoint             = waypoints.insert(waypoint + 1, midpoint2);
          }
        }

        safeness++;
      }

      if (params.enforce_fallback_solver) {
        ROS_WARN("[MrsTrajectoryGeneration]: fallback sampling enforced");
        result = findTrajectoryFallback(waypoints, initial_condition, sampling_dt, relax_heading);
      } else if (fallback_sampling) {
        ROS_WARN("[MrsTrajectoryGeneration]: executing fallback sampling");
        result = findTrajectoryFallback(waypoints, initial_condition, sampling_dt, relax_heading);
      } else if (running_async_planning_) {
        ROS_WARN("[MrsTrajectoryGeneration]: executing fallback sampling, the previous async task is still running");
        result = findTrajectoryFallback(waypoints, initial_condition, sampling_dt, relax_heading);
      } else if (overtime()) {
        ROS_WARN("[MrsTrajectoryGeneration]: executing fallback sampling, we are running over time");
        result = findTrajectoryFallback(waypoints, initial_condition, sampling_dt, relax_heading);
      } else {
        result = findTrajectoryAsync(waypoints, initial_condition, sampling_dt, relax_heading);
      }

      if (result) {
        trajectory = result.value();
      } else {
        std::stringstream ss;
        ss << "failed to find trajectory";
        ROS_WARN_STREAM("[MrsTrajectoryGeneration]: " << ss.str());
        return std::tuple(false, ss.str(), mrs_msgs::TrajectoryReference());
      }

    } else {
      ROS_DEBUG("[MrsTrajectoryGeneration]: trajectory is safe (%.2f)", max_deviation);
      safe = true;
      break;
    }
  }

  ROS_INFO("[MrsTrajectoryGeneration]: final max trajectory-path deviation: %.2f m, total trajectory time: %.2fs ", max_deviation,
           trajectory.size() * sampling_dt);

  // prepare rviz markers
  for (int i = 0; i < int(waypoints.size()); i++) {
    bw_final_.addPoint(vec3_t(waypoints.at(i).coords[0], waypoints.at(i).coords[1], waypoints.at(i).coords[2]), 0.0, 1.0, 0.0, 1.0);
  }

  mrs_msgs::TrajectoryReference mrs_trajectory;

  // convert the optimized trajectory to mrs_msgs::TrajectoryReference
  mrs_trajectory = getTrajectoryReference(trajectory, initial_condition.header.stamp, sampling_dt);

  // insert part of the MPC prediction in the front of the generated trajectory to compensate for the future
  if (path_from_future) {

    // calculate the starting idx that we will use from the current_prediction
    double path_time_offset_2   = (ros::Time::now() - current_prediction.header.stamp).toSec();  // = how long did it take to optimize
    int    path_sample_offset_2 = int(floor((path_time_offset_2 - 0.01) / 0.2)) + 1;

    // if there is anything to insert
    if (path_sample_offset > path_sample_offset_2) {

      ROS_INFO("[MrsTrajectoryGeneration]: inserting pre-trajectory from the prediction, idxs %d to %d", path_sample_offset_2, path_sample_offset);

      for (int i = path_sample_offset - 1; i >= 0; i--) {

        ROS_DEBUG("[MrsTrajectoryGeneration]: inserting idx %d", i);

        mrs_msgs::Reference reference;

        reference.heading  = current_prediction.heading[i];
        reference.position = current_prediction.position[i];

        mrs_trajectory.points.insert(mrs_trajectory.points.begin(), reference);
      }
    }

    mrs_trajectory.header.stamp = current_prediction.header.stamp;
  }

  bw_original_.publish();
  bw_final_.publish();

  std::stringstream ss;
  ss << "trajectory generated";

  ROS_DEBUG("[MrsTrajectoryGeneration]: trajectory generated, took %.3f s", (ros::Time::now() - optimize_time_start).toSec());

  return std::tuple(true, ss.str(), mrs_trajectory);
}

//}

/* findTrajectory() //{ */

std::optional<eth_mav_msgs::EigenTrajectoryPoint::Vector> MrsTrajectoryGeneration::findTrajectory(const std::vector<Waypoint_t>&   waypoints,
                                                                                                  const mrs_msgs::PositionCommand& initial_state,
                                                                                                  const double& sampling_dt, const bool& relax_heading) {

  /* mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("findTrajectory()"); */

  mrs_lib::AtomicScopeFlag unset_running(running_async_planning_);

  ROS_DEBUG("[MrsTrajectoryGeneration]: findTrajectory() started");

  ros::Time find_trajectory_time_start = ros::Time::now();

  auto params      = mrs_lib::get_mutexed(mutex_params_, params_);
  auto constraints = mrs_lib::get_mutexed(mutex_constraints_, constraints_);

  auto control_manager_diag = mrs_lib::get_mutexed(mutex_control_manager_diag_, control_manager_diag_);

  if ((initial_state.header.stamp - ros::Time::now()).toSec() < 0.2 && control_manager_diag.tracker_status.have_goal) {
    max_deviation_first_segment_ = false;
  } else {
    max_deviation_first_segment_ = true;
  }

  // optimizer

  eth_trajectory_generation::NonlinearOptimizationParameters parameters;

  parameters.f_rel                  = 0.05;
  parameters.x_rel                  = 0.1;
  parameters.time_penalty           = params.time_penalty;
  parameters.use_soft_constraints   = params.soft_constraints_enabled;
  parameters.soft_constraint_weight = params.soft_constraints_weight;
  parameters.time_alloc_method      = static_cast<eth_trajectory_generation::NonlinearOptimizationParameters::TimeAllocMethod>(params.time_allocation);
  if (params.time_allocation == 2) {
    parameters.algorithm = nlopt::LD_LBFGS;
  }
  parameters.initial_stepsize_rel            = 0.1;
  parameters.inequality_constraint_tolerance = params.inequality_constraint_tolerance;
  parameters.equality_constraint_tolerance   = params.equality_constraint_tolerance;
  parameters.max_iterations                  = params.max_iterations;
  parameters.max_time                        = NLOPT_EXEC_TIME_FACTOR * timeLeft();

  eth_trajectory_generation::Vertex::Vector vertices;
  const int                                 dimension = 4;

  int derivative_to_optimize = eth_trajectory_generation::derivative_order::ACCELERATION;

  switch (params.derivative_to_optimize) {
    case 0: {
      derivative_to_optimize = eth_trajectory_generation::derivative_order::ACCELERATION;
      break;
    }
    case 1: {
      derivative_to_optimize = eth_trajectory_generation::derivative_order::JERK;
      break;
    }
    case 2: {
      derivative_to_optimize = eth_trajectory_generation::derivative_order::SNAP;
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

    eth_trajectory_generation::Vertex vertex(dimension);

    if (i == 0) {

      vertex.makeStartOrEnd(Eigen::Vector4d(x, y, z, heading), derivative_to_optimize);

      vertex.addConstraint(eth_trajectory_generation::derivative_order::POSITION, Eigen::Vector4d(x, y, z, heading));

      vertex.addConstraint(eth_trajectory_generation::derivative_order::VELOCITY,
                           Eigen::Vector4d(initial_state.velocity.x, initial_state.velocity.y, initial_state.velocity.z, initial_state.heading_rate));

      vertex.addConstraint(
          eth_trajectory_generation::derivative_order::ACCELERATION,
          Eigen::Vector4d(initial_state.acceleration.x, initial_state.acceleration.y, initial_state.acceleration.z, initial_state.heading_acceleration));

      vertex.addConstraint(eth_trajectory_generation::derivative_order::JERK,
                           Eigen::Vector4d(initial_state.jerk.x, initial_state.jerk.y, initial_state.jerk.z, initial_state.heading_jerk));

    } else if (i == (waypoints.size() - 1)) {  // the last point

      vertex.makeStartOrEnd(Eigen::Vector4d(x, y, z, heading), derivative_to_optimize);

      vertex.addConstraint(eth_trajectory_generation::derivative_order::POSITION, Eigen::Vector4d(x, y, z, heading));

    } else {  // mid points

      vertex.addConstraint(eth_trajectory_generation::derivative_order::POSITION, Eigen::Vector4d(x, y, z, heading));

      if (waypoints.at(i).stop_at) {
        vertex.addConstraint(eth_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector4d(0, 0, 0, 0));
        vertex.addConstraint(eth_trajectory_generation::derivative_order::ACCELERATION, Eigen::Vector4d(0, 0, 0, 0));
        vertex.addConstraint(eth_trajectory_generation::derivative_order::JERK, Eigen::Vector4d(0, 0, 0, 0));
      }
    }

    vertices.push_back(vertex);
  }

  // | ---------------- compute the segment times --------------- |

  double v_max_horizontal, a_max_horizontal, j_max_horizontal;
  double v_max_vertical, a_max_vertical, j_max_vertical;

  // use the small of the ascending/descending values
  double vertical_speed_lim        = std::min(constraints.vertical_ascending_speed, constraints.vertical_descending_speed);
  double vertical_acceleration_lim = std::min(constraints.vertical_ascending_acceleration, constraints.vertical_descending_acceleration);

  if (override_constraints_) {

    v_max_horizontal = override_max_velocity_horizontal_ < constraints.horizontal_speed ? override_max_velocity_horizontal_ : constraints.horizontal_speed;
    a_max_horizontal = override_max_acceleration_horizontal_ < constraints.horizontal_acceleration ? override_max_acceleration_horizontal_
                                                                                                   : constraints.horizontal_acceleration;

    v_max_vertical = override_max_velocity_vertical_ < vertical_speed_lim ? override_max_velocity_vertical_ : vertical_speed_lim;
    a_max_vertical = override_max_acceleration_vertical_ < vertical_acceleration_lim ? override_max_acceleration_vertical_ : vertical_acceleration_lim;

    ROS_DEBUG("[MrsTrajectoryGeneration]: overriding constraints by a user");
  } else {

    v_max_horizontal = constraints.horizontal_speed;
    a_max_horizontal = constraints.horizontal_acceleration;

    v_max_vertical = vertical_speed_lim;
    a_max_vertical = vertical_acceleration_lim;
  }

  j_max_horizontal = constraints.horizontal_jerk;
  j_max_vertical   = std::min(constraints.vertical_ascending_jerk, constraints.vertical_descending_jerk);

  double v_max_heading, a_max_heading, j_max_heading;

  if (relax_heading) {
    v_max_heading = std::numeric_limits<float>::max();
    a_max_heading = std::numeric_limits<float>::max();
    j_max_heading = std::numeric_limits<float>::max();
  } else {
    v_max_heading = constraints.heading_speed;
    a_max_heading = constraints.heading_acceleration;
    j_max_heading = constraints.heading_jerk;
  }

  ROS_INFO("[MrsTrajectoryGeneration]: using constraints:");
  ROS_INFO("[MrsTrajectoryGeneration]: horizontal: vel = %.2f, acc = %.2f, jerk = %.2f", v_max_horizontal, a_max_horizontal, j_max_horizontal);
  ROS_INFO("[MrsTrajectoryGeneration]: vertical: vel = %.2f, acc = %.2f, jerk = %.2f", v_max_vertical, a_max_vertical, j_max_vertical);
  ROS_INFO("[MrsTrajectoryGeneration]: heading: vel = %.2f, acc = %.2f, jerk = %.2f", v_max_heading, a_max_vertical, j_max_vertical);

  std::vector<double> segment_times, segment_times_baca;
  segment_times      = estimateSegmentTimes(vertices, v_max_horizontal, v_max_vertical, a_max_horizontal, a_max_vertical, j_max_horizontal, j_max_vertical,
                                       v_max_heading, a_max_heading);
  segment_times_baca = estimateSegmentTimesBaca(vertices, v_max_horizontal, v_max_vertical, a_max_horizontal, a_max_vertical, j_max_horizontal, j_max_vertical,
                                                v_max_heading, a_max_heading);

  double initial_total_time      = 0;
  double initial_total_time_baca = 0;
  for (int i = 0; i < int(segment_times_baca.size()); i++) {
    initial_total_time += segment_times[i];
    initial_total_time_baca += segment_times_baca[i];
  }

  ROS_DEBUG("[MrsTrajectoryGeneration]: initial total time (Euclidean): %.2f", initial_total_time);
  ROS_DEBUG("[MrsTrajectoryGeneration]: initial total time (Baca): %.2f", initial_total_time_baca);

  // | --------- create an optimizer object and solve it -------- |

  const int                                                     N = 10;
  eth_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  opt.addMaximumMagnitudeConstraint(0, eth_trajectory_generation::derivative_order::VELOCITY, v_max_horizontal);
  opt.addMaximumMagnitudeConstraint(0, eth_trajectory_generation::derivative_order::ACCELERATION, a_max_horizontal);
  opt.addMaximumMagnitudeConstraint(0, eth_trajectory_generation::derivative_order::JERK, j_max_horizontal);

  opt.addMaximumMagnitudeConstraint(1, eth_trajectory_generation::derivative_order::VELOCITY, v_max_horizontal);
  opt.addMaximumMagnitudeConstraint(1, eth_trajectory_generation::derivative_order::ACCELERATION, a_max_horizontal);
  opt.addMaximumMagnitudeConstraint(1, eth_trajectory_generation::derivative_order::JERK, j_max_horizontal);

  opt.addMaximumMagnitudeConstraint(2, eth_trajectory_generation::derivative_order::VELOCITY, v_max_vertical);
  opt.addMaximumMagnitudeConstraint(2, eth_trajectory_generation::derivative_order::ACCELERATION, a_max_vertical);
  opt.addMaximumMagnitudeConstraint(2, eth_trajectory_generation::derivative_order::JERK, j_max_vertical);

  opt.addMaximumMagnitudeConstraint(3, eth_trajectory_generation::derivative_order::VELOCITY, v_max_heading);
  opt.addMaximumMagnitudeConstraint(3, eth_trajectory_generation::derivative_order::ACCELERATION, a_max_heading);
  opt.addMaximumMagnitudeConstraint(3, eth_trajectory_generation::derivative_order::JERK, j_max_heading);

  opt.optimize();

  if (overtime()) {
    return {};
  }

  std::string result_str;

  switch (opt.getOptimizationInfo().stopping_reason) {
    case nlopt::FAILURE: {
      result_str = "generic failure";
      break;
    }
    case nlopt::INVALID_ARGS: {
      result_str = "invalid args";
      break;
    }
    case nlopt::OUT_OF_MEMORY: {
      result_str = "out of memory";
      break;
    }
    case nlopt::ROUNDOFF_LIMITED: {
      result_str = "roundoff limited";
      break;
    }
    case nlopt::FORCED_STOP: {
      result_str = "forced stop";
      break;
    }
    case nlopt::STOPVAL_REACHED: {
      result_str = "stopval reached";
      break;
    }
    case nlopt::FTOL_REACHED: {
      result_str = "ftol reached";
      break;
    }
    case nlopt::XTOL_REACHED: {
      result_str = "xtol reached";
      break;
    }
    case nlopt::MAXEVAL_REACHED: {
      result_str = "maxeval reached";
      break;
    }
    case nlopt::MAXTIME_REACHED: {
      result_str = "maxtime reached";
      break;
    }
    default: {
      result_str = "UNKNOWN FAILURE CODE";
      break;
    }
  }

  if (opt.getOptimizationInfo().stopping_reason >= 1 && opt.getOptimizationInfo().stopping_reason != 6) {
    ROS_DEBUG("[MrsTrajectoryGeneration]: optimization finished successfully with code %d, '%s'", opt.getOptimizationInfo().stopping_reason,
              result_str.c_str());

  } else if (opt.getOptimizationInfo().stopping_reason == -1) {
    ROS_DEBUG("[MrsTrajectoryGeneration]: optimization finished with a generic error code %d, '%s'", opt.getOptimizationInfo().stopping_reason,
              result_str.c_str());

  } else {
    ROS_WARN("[MrsTrajectoryGeneration]: optimization failed with code %d, '%s', took %.3f s", opt.getOptimizationInfo().stopping_reason, result_str.c_str(),
             (ros::Time::now() - find_trajectory_time_start).toSec());
    return {};
  }

  // | ------------- obtain the polynomial segments ------------- |

  eth_trajectory_generation::Segment::Vector segments;
  opt.getPolynomialOptimizationRef().getSegments(&segments);

  if (overtime()) {
    return {};
  }

  // | --------------- create the trajectory class -------------- |

  eth_trajectory_generation::Trajectory trajectory;
  opt.getTrajectory(&trajectory);

  eth_mav_msgs::EigenTrajectoryPoint::Vector states;

  ROS_DEBUG("[MrsTrajectoryGeneration]: starting eth sampling with dt = %.2f s ", sampling_dt);

  bool success = eth_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_dt, &states);

  if (overtime()) {
    return {};
  }

  // validate the temporal sampling of the trajectory

  if ((states.size() * sampling_dt) > (_max_trajectory_len_factor_ * initial_total_time_baca)) {
    ROS_ERROR("[MrsTrajectoryGeneration]: the final trajectory sampling is too long = %.2f, initial 'baca' estimate = %.2f, allowed factor %.2f, aborting",
              (states.size() * sampling_dt), initial_total_time_baca, _max_trajectory_len_factor_);

    std::stringstream ss;
    ss << "trajectory sampling failed";
    ROS_ERROR_STREAM("[MrsTrajectoryGeneration]: " << ss.str());
    return {};

  } else if ((states.size() * sampling_dt) < (_min_trajectory_len_factor_ * initial_total_time_baca)) {
    ROS_ERROR("[MrsTrajectoryGeneration]: the final trajectory sampling is too short = %.2f, initial 'baca' estimate = %.2f, allowed factor %.2f, aborting",
              (states.size() * sampling_dt), initial_total_time_baca, _min_trajectory_len_factor_);

    std::stringstream ss;
    ss << "trajectory sampling failed";
    ROS_ERROR_STREAM("[MrsTrajectoryGeneration]: " << ss.str());
    return {};

  } else {
    ROS_DEBUG("[MrsTrajectoryGeneration]: estimated/final trajectory length ratio (final/estimated) %.2f",
              (states.size() * sampling_dt) / initial_total_time_baca);
  }

  if (success) {
    ROS_DEBUG("[MrsTrajectoryGeneration]: eth sampling finished, took %.3f s", (ros::Time::now() - find_trajectory_time_start).toSec());
    return std::optional(states);

  } else {
    ROS_ERROR("[MrsTrajectoryGeneration]: eth could not sample the trajectory, took %.3f s", (ros::Time::now() - find_trajectory_time_start).toSec());
    return {};
  }
}

//}

/* findTrajectoryFallback() //{ */

std::optional<eth_mav_msgs::EigenTrajectoryPoint::Vector> MrsTrajectoryGeneration::findTrajectoryFallback(const std::vector<Waypoint_t>&   waypoints,
                                                                                                          const mrs_msgs::PositionCommand& initial_state,
                                                                                                          const double&                    sampling_dt,
                                                                                                          const bool&                      relax_heading) {

  ros::Time time_start = ros::Time::now();

  ROS_WARN("[MrsTrajectoryGeneration]: fallback sampling started");

  auto params      = mrs_lib::get_mutexed(mutex_params_, params_);
  auto constraints = mrs_lib::get_mutexed(mutex_constraints_, constraints_);

  eth_trajectory_generation::Vertex::Vector vertices;
  const int                                 dimension = 4;

  // | --------------- add constraints to vertices -------------- |

  double last_heading = initial_state.heading;

  for (size_t i = 0; i < waypoints.size(); i++) {

    double x       = waypoints.at(i).coords[0];
    double y       = waypoints.at(i).coords[1];
    double z       = waypoints.at(i).coords[2];
    double heading = sradians::unwrap(waypoints.at(i).coords[3], last_heading);
    last_heading   = heading;

    eth_trajectory_generation::Vertex vertex(dimension);

    vertex.addConstraint(eth_trajectory_generation::derivative_order::POSITION, Eigen::Vector4d(x, y, z, heading));

    vertices.push_back(vertex);
  }

  // | ---------------- compute the segment times --------------- |

  double v_max_horizontal, a_max_horizontal, j_max_horizontal;
  double v_max_vertical, a_max_vertical, j_max_vertical;

  // use the small of the ascending/descending values
  double vertical_speed_lim        = std::min(constraints.vertical_ascending_speed, constraints.vertical_descending_speed);
  double vertical_acceleration_lim = std::min(constraints.vertical_ascending_acceleration, constraints.vertical_descending_acceleration);

  if (override_constraints_) {

    v_max_horizontal = override_max_velocity_horizontal_ < constraints.horizontal_speed ? override_max_velocity_horizontal_ : constraints.horizontal_speed;
    a_max_horizontal = override_max_acceleration_horizontal_ < constraints.horizontal_acceleration ? override_max_acceleration_horizontal_
                                                                                                   : constraints.horizontal_acceleration;

    v_max_vertical = override_max_velocity_vertical_ < vertical_speed_lim ? override_max_velocity_vertical_ : vertical_speed_lim;
    a_max_vertical = override_max_acceleration_vertical_ < vertical_acceleration_lim ? override_max_acceleration_vertical_ : vertical_acceleration_lim;

    ROS_DEBUG("[MrsTrajectoryGeneration]: overriding constraints by a user");
  } else {

    v_max_horizontal = constraints.horizontal_speed;
    a_max_horizontal = constraints.horizontal_acceleration;

    v_max_vertical = vertical_speed_lim;
    a_max_vertical = vertical_acceleration_lim;
  }

  j_max_horizontal = constraints.horizontal_jerk;
  j_max_vertical   = std::min(constraints.vertical_ascending_jerk, constraints.vertical_descending_jerk);

  double v_max_heading, a_max_heading, j_max_heading;

  if (relax_heading) {
    v_max_heading = std::numeric_limits<float>::max();
    a_max_heading = std::numeric_limits<float>::max();
    j_max_heading = std::numeric_limits<float>::max();
  } else {
    v_max_heading = constraints.heading_speed;
    a_max_heading = constraints.heading_acceleration;
    j_max_heading = constraints.heading_jerk;
  }

  v_max_horizontal *= _fallback_sampling_speed_factor_;
  v_max_vertical *= _fallback_sampling_speed_factor_;

  a_max_horizontal *= _fallback_sampling_accel_factor_;
  a_max_vertical *= _fallback_sampling_accel_factor_;

  ROS_INFO("[MrsTrajectoryGeneration]: using constraints:");
  ROS_INFO("[MrsTrajectoryGeneration]: horizontal: vel = %.2f, acc = %.2f, jerk = %.2f", v_max_horizontal, a_max_horizontal, j_max_horizontal);
  ROS_INFO("[MrsTrajectoryGeneration]: vertical: vel = %.2f, acc = %.2f, jerk = %.2f", v_max_vertical, a_max_vertical, j_max_vertical);
  ROS_INFO("[MrsTrajectoryGeneration]: heading: vel = %.2f, acc = %.2f, jerk = %.2f", v_max_heading, a_max_vertical, j_max_vertical);

  std::vector<double> segment_times, segment_times_baca;
  segment_times      = estimateSegmentTimes(vertices, v_max_horizontal, v_max_vertical, a_max_horizontal, a_max_vertical, j_max_horizontal, j_max_vertical,
                                       v_max_heading, a_max_heading);
  segment_times_baca = estimateSegmentTimesBaca(vertices, v_max_horizontal, v_max_vertical, a_max_horizontal, a_max_vertical, j_max_horizontal, j_max_vertical,
                                                v_max_heading, a_max_heading);

  double initial_total_time      = 0;
  double initial_total_time_baca = 0;
  for (int i = 0; i < int(segment_times_baca.size()); i++) {
    initial_total_time += segment_times[i];
    initial_total_time_baca += segment_times_baca[i];
  }

  ROS_WARN("[MrsTrajectoryGeneration]: fallback: initial total time (Euclidean): %.2f", initial_total_time);
  ROS_WARN("[MrsTrajectoryGeneration]: fallback: initial total time (Baca): %.2f", initial_total_time_baca);

  // | --------- create an optimizer object and solve it -------- |

  eth_mav_msgs::EigenTrajectoryPoint::Vector states;

  // interpolate each segment
  for (size_t i = 0; i < waypoints.size() - 1; i++) {

    Eigen::VectorXd start, end;

    const double segment_time = segment_times_baca[i];

    int    n_samples;
    double interp_step;

    /* ROS_DEBUG("[MrsTrajectoryGeneration]: interpolating segment %d [%.2f %.2f %.2f %.2f] -> [%.2f %.2f %.2f %.2f], baca time %.2f s", int(i), */
    /*           waypoints[i].coords[0], waypoints[i].coords[1], waypoints[i].coords[2], waypoints[i].coords[3], waypoints[i + 1].coords[0], */
    /*           waypoints[i + 1].coords[1], waypoints[i + 1].coords[2], waypoints[i + 1].coords[3], segment_time); */

    if (segment_time > 1e-1) {

      n_samples = floor(segment_time / sampling_dt);

      // important
      if (n_samples > 0) {
        interp_step = 1.0 / double(n_samples);
      } else {
        interp_step = 0.5;
      }

    } else {
      n_samples   = 0;
      interp_step = 0;
      /* ROS_WARN_THROTTLE(1.0, "[MrsTrajectoryGeneration]: fallback: special case, segment too short"); */
    }

    for (int j = 0; j <= n_samples; j++) {

      Waypoint_t point = interpolatePoint(waypoints[i], waypoints[i + 1], j * interp_step);

      /* ROS_DEBUG_STREAM("[MrsTrajectoryGeneration]: sample " << j << " " << point.coords.transpose()); */

      eth_mav_msgs::EigenTrajectoryPoint eth_point;
      eth_point.position_W[0] = point.coords[0];
      eth_point.position_W[1] = point.coords[1];
      eth_point.position_W[2] = point.coords[2];
      eth_point.setFromYaw(point.coords[3]);

      states.push_back(eth_point);

      auto control_manager_diag = mrs_lib::get_mutexed(mutex_control_manager_diag_, control_manager_diag_);

      // the first sample of the first waypoint
      // we should stop for a little bit to give the transition
      // from the initial cooordinates more time
      if (((control_manager_diag.tracker_status.have_goal && i == 0) || (!control_manager_diag.tracker_status.have_goal && i == 1)) && j == 0) {

        double time_to_stop = fabs(initial_state.velocity.x) / a_max_horizontal + fabs(initial_state.velocity.y) / a_max_horizontal +
                              fabs(initial_state.velocity.z) / a_max_vertical;

        time_to_stop += fabs(initial_state.acceleration.x) / j_max_horizontal + fabs(initial_state.acceleration.y) / j_max_horizontal +
                        fabs(initial_state.acceleration.z) / j_max_vertical;

        int samples_to_stop = int(round(1.0 * (time_to_stop / sampling_dt)));

        if (!control_manager_diag.tracker_status.have_goal) {
          samples_to_stop += int(round(1.0 / sampling_dt));
        }

        ROS_DEBUG("[MrsTrajectoryGeneration]: pre-inserting %d samples of the first point", samples_to_stop);

        for (int k = 0; k < samples_to_stop; k++) {
          states.push_back(eth_point);
        }
      }

      if (j == 0 && i > 0 && waypoints[i].stop_at) {

        int insert_samples = int(round(_fallback_sampling_stopping_time_ / sampling_dt));

        for (int k = 0; k < insert_samples; k++) {
          states.push_back(eth_point);
        }
      }
    }
  }

  bool success = true;

  ROS_WARN("[MrsTrajectoryGeneration]: fallback: sampling finished, took %.3f s", (ros::Time::now() - time_start).toSec());

  // | --------------- create the trajectory class -------------- |

  if (success) {
    return std::optional(states);
  } else {
    ROS_ERROR("[MrsTrajectoryGeneration]: fallback: sampling failed");
    return {};
  }
}

//}

/* validateTrajectorySpatial() //{ */

std::tuple<bool, int, std::vector<bool>, double> MrsTrajectoryGeneration::validateTrajectorySpatial(
    const eth_mav_msgs::EigenTrajectoryPoint::Vector& trajectory, const std::vector<Waypoint_t>& waypoints) {

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
    const vec3_t sample = vec3_t(trajectory[i].position_W[0], trajectory[i].position_W[1], trajectory[i].position_W[2]);

    // next sample
    const vec3_t next_sample = vec3_t(trajectory[i + 1].position_W[0], trajectory[i + 1].position_W[1], trajectory[i + 1].position_W[2]);

    // segment start
    const vec3_t segment_start = vec3_t(waypoints.at(waypoint_idx).coords[0], waypoints.at(waypoint_idx).coords[1], waypoints.at(waypoint_idx).coords[2]);

    // segment end
    const vec3_t segment_end =
        vec3_t(waypoints.at(waypoint_idx + 1).coords[0], waypoints.at(waypoint_idx + 1).coords[1], waypoints.at(waypoint_idx + 1).coords[2]);

    const double distance_from_segment = distFromSegment(sample, segment_start, segment_end);

    const double segment_end_dist = distFromSegment(segment_end, sample, next_sample);

    if (waypoint_idx > 0 || max_deviation_first_segment_ || int(waypoints.size()) <= 2) {

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

/* getTrajectorySegmenCenterIdxs() //{ */

std::vector<int> MrsTrajectoryGeneration::getTrajectorySegmenCenterIdxs(const eth_mav_msgs::EigenTrajectoryPoint::Vector& trajectory,
                                                                        const std::vector<Waypoint_t>&                    waypoints) {

  // prepare the output

  std::vector<int> segment_centers;

  // index in the path
  int last_segment_start = 0;

  // index in the trajectory
  int last_segment_start_sample = 0;

  for (size_t i = 0; i < trajectory.size() - 1; i++) {

    // the trajectory sample
    const vec3_t sample = vec3_t(trajectory[i].position_W[0], trajectory[i].position_W[1], trajectory[i].position_W[2]);

    // next sample
    const vec3_t next_sample = vec3_t(trajectory[i + 1].position_W[0], trajectory[i + 1].position_W[1], trajectory[i + 1].position_W[2]);

    // segment start
    const vec3_t segment_start =
        vec3_t(waypoints.at(last_segment_start).coords[0], waypoints.at(last_segment_start).coords[1], waypoints.at(last_segment_start).coords[2]);

    // segment end
    const vec3_t segment_end =
        vec3_t(waypoints.at(last_segment_start + 1).coords[0], waypoints.at(last_segment_start + 1).coords[1], waypoints.at(last_segment_start + 1).coords[2]);

    const double distance_from_segment = distFromSegment(sample, segment_start, segment_end);

    const double segment_end_dist = distFromSegment(segment_end, sample, next_sample);

    if (segment_end_dist < 0.05 && last_segment_start < (int(waypoints.size()) - 2)) {
      last_segment_start++;

      segment_centers.push_back(int(floor(double(i - last_segment_start_sample) / 2.0)));
    }
  }

  return segment_centers;
}

//}

// | --------------------- minor routines --------------------- |

/* findTrajectoryAsync() //{ */

std::optional<eth_mav_msgs::EigenTrajectoryPoint::Vector> MrsTrajectoryGeneration::findTrajectoryAsync(const std::vector<Waypoint_t>&   waypoints,
                                                                                                       const mrs_msgs::PositionCommand& initial_state,
                                                                                                       const double& sampling_dt, const bool& relax_heading) {

  ROS_DEBUG("[MrsTrajectoryGeneration]: starting the async planning task");

  future_trajectory_result_ =
      std::async(std::launch::async, &MrsTrajectoryGeneration::findTrajectory, this, waypoints, initial_state, sampling_dt, relax_heading);

  while (ros::ok() && future_trajectory_result_.wait_for(std::chrono::milliseconds(1)) != std::future_status::ready) {

    if (overtime()) {
      ROS_WARN("[MrsTrajectoryGeneration]: async task planning timeout, breaking");
      return {};
    }
  }

  ROS_DEBUG("[MrsTrajectoryGeneration]: async planning task finished successfully");

  return future_trajectory_result_.get();
}

//}

/* distFromSegment() //{ */

double MrsTrajectoryGeneration::distFromSegment(const vec3_t& point, const vec3_t& seg1, const vec3_t& seg2) {

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

/* getTrajectoryReference() //{ */

mrs_msgs::TrajectoryReference MrsTrajectoryGeneration::getTrajectoryReference(const eth_mav_msgs::EigenTrajectoryPoint::Vector& trajectory,
                                                                              const ros::Time& stamp, const double& sampling_dt) {

  mrs_msgs::TrajectoryReference msg;

  msg.header.frame_id = frame_id_;
  msg.header.stamp    = stamp;
  msg.fly_now         = fly_now_;
  msg.loop            = loop_;
  msg.use_heading     = use_heading_;
  msg.dt              = sampling_dt;

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

Waypoint_t MrsTrajectoryGeneration::interpolatePoint(const Waypoint_t& a, const Waypoint_t& b, const double& coeff) {

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

/* checkNaN() //{ */

bool MrsTrajectoryGeneration::checkNaN(const Waypoint_t& a) {

  if (!std::isfinite(a.coords[0])) {
    ROS_ERROR("NaN detected in variable \"a.coords[0]\"!!!");
    return false;
  }

  if (!std::isfinite(a.coords[1])) {
    ROS_ERROR("NaN detected in variable \"a.coords[1]\"!!!");
    return false;
  }

  if (!std::isfinite(a.coords[2])) {
    ROS_ERROR("NaN detected in variable \"a.coords[2]\"!!!");
    return false;
  }

  if (!std::isfinite(a.coords[3])) {
    ROS_ERROR("NaN detected in variable \"a.coords[3]\"!!!");
    return false;
  }

  return true;
}

//}

/* trajectorySrv() //{ */

bool MrsTrajectoryGeneration::trajectorySrv(const mrs_msgs::TrajectoryReference& msg) {

  mrs_msgs::TrajectoryReferenceSrv srv;
  srv.request.trajectory = msg;

  bool res = service_client_trajectory_reference_.call(srv);

  if (res) {

    if (!srv.response.success) {
      ROS_WARN("[MrsTrajectoryGeneration]: service call for trajectory_reference returned: '%s'", srv.response.message.c_str());
    }

    return srv.response.success;

  } else {

    ROS_ERROR("[MrsTrajectoryGeneration]: service call for trajectory_reference failed!");

    return false;
  }
}

//}

/* transformPositionCmd() //{ */

std::optional<mrs_msgs::PositionCommand> MrsTrajectoryGeneration::transformPositionCmd(const mrs_msgs::PositionCommand& position_cmd,
                                                                                       const std::string&               target_frame) {

  // if we transform to the current control frame, which is in fact the same frame as the position_cmd is in
  if (target_frame == "") {
    return position_cmd;
  }

  // find the transformation
  auto tf = transformer_->getTransform(position_cmd.header.frame_id, target_frame, position_cmd.header.stamp);

  if (!tf) {
    ROS_ERROR("[MrsTrajectoryGeneration]: could not find transform from '%s' to '%s' in time %f", position_cmd.header.frame_id.c_str(), target_frame.c_str(),
              position_cmd.header.stamp.toSec());
    return {};
  }

  mrs_msgs::PositionCommand cmd_out;

  cmd_out.header.stamp    = tf.value().stamp();
  cmd_out.header.frame_id = tf.value().to();

  /* position + heading //{ */

  {
    geometry_msgs::PoseStamped pos;
    pos.header = position_cmd.header;

    pos.pose.position    = position_cmd.position;
    pos.pose.orientation = mrs_lib::AttitudeConverter(0, 0, position_cmd.heading);

    if (auto ret = transformer_->transform(tf.value(), pos)) {
      cmd_out.position = ret.value().pose.position;
      try {
        cmd_out.heading = mrs_lib::AttitudeConverter(ret.value().pose.orientation).getHeading();
      }
      catch (...) {
        ROS_ERROR("[MrsTrajectoryGeneration]: failed to transform heading in position_cmd");
        cmd_out.heading = 0;
      }
    } else {
      return {};
    }
  }

  //}

  /* velocity //{ */

  {
    geometry_msgs::Vector3Stamped vec;
    vec.header = position_cmd.header;

    vec.vector = position_cmd.velocity;

    if (auto ret = transformer_->transform(tf.value(), vec)) {
      cmd_out.velocity = ret.value().vector;
    } else {
      return {};
    }
  }

  //}

  /* acceleration //{ */

  {
    geometry_msgs::Vector3Stamped vec;
    vec.header = position_cmd.header;

    vec.vector = position_cmd.acceleration;

    if (auto ret = transformer_->transform(tf.value(), vec)) {
      cmd_out.acceleration = ret.value().vector;
    } else {
      return {};
    }
  }

  //}

  /* jerk //{ */

  {
    geometry_msgs::Vector3Stamped vec;
    vec.header = position_cmd.header;

    vec.vector = position_cmd.jerk;

    if (auto ret = transformer_->transform(tf.value(), vec)) {
      cmd_out.jerk = ret.value().vector;
    } else {
      return {};
    }
  }

  //}

  /* heading derivatives //{ */

  // this does not need to be transformed
  cmd_out.heading_rate         = position_cmd.heading_rate;
  cmd_out.heading_acceleration = position_cmd.heading_acceleration;
  cmd_out.heading_jerk         = position_cmd.heading_jerk;

  //}

  return cmd_out;
}

//}

/* overtime() //{ */

bool MrsTrajectoryGeneration::overtime(void) {

  auto start_time_total   = mrs_lib::get_mutexed(mutex_start_time_total_, start_time_total_);
  auto max_execution_time = mrs_lib::get_mutexed(mutex_max_execution_time_, max_execution_time_);

  double overtime = (ros::Time::now() - start_time_total).toSec();

  if (overtime > (OVERTIME_SAFETY_FACTOR * max_execution_time - OVERTIME_SAFETY_OFFSET)) {
    return true;
  }

  return false;
}

//}

/* timeLeft() //{ */

double MrsTrajectoryGeneration::timeLeft(void) {

  auto start_time_total   = mrs_lib::get_mutexed(mutex_start_time_total_, start_time_total_);
  auto max_execution_time = mrs_lib::get_mutexed(mutex_max_execution_time_, max_execution_time_);

  double current_execution_time = (ros::Time::now() - start_time_total).toSec();

  if (current_execution_time >= max_execution_time) {
    return 0;
  } else {
    return max_execution_time - current_execution_time;
  }
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackPath() //{ */

void MrsTrajectoryGeneration::callbackPath(const mrs_msgs::PathConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  /* preconditions //{ */

  if (!got_constraints_) {
    std::stringstream ss;
    ss << "missing constraints";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[MrsTrajectoryGeneration]: " << ss.str());
    return;
  }

  if (!got_position_cmd_) {
    std::stringstream ss;
    ss << "missing position cmd";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[MrsTrajectoryGeneration]: " << ss.str());
    return;
  }

  if (!got_prediction_full_state_) {
    std::stringstream ss;
    ss << "missing full-state prediction";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[MrsTrajectoryGeneration]: " << ss.str());
    return;
  }

  if (!got_control_manager_diag_) {
    std::stringstream ss;
    ss << "missing control manager diagnostics";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[MrsTrajectoryGeneration]: " << ss.str());
    return;
  }

  //}

  {
    std::scoped_lock lock(mutex_start_time_total_);

    start_time_total_ = ros::Time::now();
  }

  double path_time_offset = 0;

  if (msg->header.stamp != ros::Time(0)) {
    path_time_offset = (msg->header.stamp - ros::Time::now()).toSec();
  }

  if (path_time_offset > 1e-3) {

    std::scoped_lock lock(mutex_max_execution_time_);

    max_execution_time_ = FUTURIZATION_EXEC_TIME_FACTOR * path_time_offset;

    ROS_INFO("[MrsTrajectoryGeneration]: setting the max execution time to %.3f s = %.1f * %.3f", max_execution_time_, FUTURIZATION_EXEC_TIME_FACTOR,
             path_time_offset);
  } else {

    std::scoped_lock lock(mutex_max_execution_time_, mutex_params_);

    max_execution_time_ = params_.max_time;
  }

  ROS_INFO("[MrsTrajectoryGeneration]: got path from message");

  try {
    publisher_original_path_.publish(msg);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", publisher_original_path_.getTopic().c_str());
  }

  if (msg->points.empty()) {
    std::stringstream ss;
    ss << "received an empty message";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[MrsTrajectoryGeneration]: " << ss.str());
    return;
  }

  fly_now_                              = msg->fly_now;
  use_heading_                          = msg->use_heading;
  frame_id_                             = msg->header.frame_id;
  override_constraints_                 = msg->override_constraints;
  loop_                                 = msg->loop;
  override_max_velocity_horizontal_     = msg->override_max_velocity_horizontal;
  override_max_velocity_vertical_       = msg->override_max_velocity_vertical;
  override_max_acceleration_horizontal_ = msg->override_max_acceleration_horizontal;
  override_max_acceleration_vertical_   = msg->override_max_acceleration_vertical;
  stop_at_waypoints_                    = msg->stop_at_waypoints;

  std::vector<Waypoint_t> waypoints;

  for (size_t i = 0; i < msg->points.size(); i++) {

    double x       = msg->points[i].position.x;
    double y       = msg->points[i].position.y;
    double z       = msg->points[i].position.z;
    double heading = msg->points[i].heading;

    Waypoint_t wp;
    wp.coords  = Eigen::Vector4d(x, y, z, heading);
    wp.stop_at = stop_at_waypoints_;

    if (!checkNaN(wp)) {
      ROS_ERROR("[MrsTrajectoryGeneration]: NaN detected in waypoint #%d", int(i));
      return;
    }

    waypoints.push_back(wp);
  }

  if (loop_) {
    waypoints.push_back(waypoints[0]);
  }

  auto position_cmd       = mrs_lib::get_mutexed(mutex_position_cmd_, position_cmd_);
  auto current_prediction = mrs_lib::get_mutexed(mutex_prediction_full_state_, prediction_full_state_);

  bool                          success = false;
  std::string                   message;
  mrs_msgs::TrajectoryReference trajectory;

  for (int i = 0; i < _n_attempts_; i++) {

    auto position_cmd       = mrs_lib::get_mutexed(mutex_position_cmd_, position_cmd_);
    auto current_prediction = mrs_lib::get_mutexed(mutex_prediction_full_state_, prediction_full_state_);

    // the last iteration and the fallback sampling is enabled
    bool fallback_sampling = (_n_attempts_ > 1) && (i == (_n_attempts_ - 1)) && _fallback_sampling_enabled_;

    std::tie(success, message, trajectory) = optimize(waypoints, msg->header, position_cmd, prediction_full_state_, fallback_sampling, msg->relax_heading);

    if (success) {
      break;
    } else {
      if (i < _n_attempts_) {
        ROS_WARN("[MrsTrajectoryGeneration]: failed to calculate a feasible trajectory, trying again with different initial conditions!");
      } else {
        ROS_WARN("[MrsTrajectoryGeneration]: failed to calculate a feasible trajectory");
      }
    }
  }

  double total_time = (ros::Time::now() - start_time_total_).toSec();

  auto max_execution_time = mrs_lib::get_mutexed(mutex_max_execution_time_, max_execution_time_);

  if (total_time > max_execution_time) {
    ROS_ERROR("[MrsTrajectoryGeneration]: trajectory ready, took %.3f s in total (exceeding maxtime %.3f s by %.3f s)", total_time, max_execution_time,
              total_time - max_execution_time);
  } else {
    ROS_INFO("[MrsTrajectoryGeneration]: trajectory ready, took %.3f s in total (out of %.3f)", total_time, max_execution_time);
  }

  if (success) {

    bool published = trajectorySrv(trajectory);

    if (published) {

      ROS_INFO("[MrsTrajectoryGeneration]: trajectory successfully published");

    } else {

      ROS_ERROR("[MrsTrajectoryGeneration]: could not publish the trajectory");
    }

  } else {

    ROS_ERROR("[MrsTrajectoryGeneration]: failed to calculate a feasible trajectory, no publishing a result");
  }
}

//}

/* callbackPathSrv() //{ */

bool MrsTrajectoryGeneration::callbackPathSrv(mrs_msgs::PathSrv::Request& req, mrs_msgs::PathSrv::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  /* mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("callback()"); */

  /* precondition //{ */

  if (!got_constraints_) {
    std::stringstream ss;
    ss << "missing constraints";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[MrsTrajectoryGeneration]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  if (!got_position_cmd_) {
    std::stringstream ss;
    ss << "missing position cmd";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[MrsTrajectoryGeneration]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  if (!got_prediction_full_state_) {
    std::stringstream ss;
    ss << "missing full-state prediction";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[MrsTrajectoryGeneration]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  if (!got_control_manager_diag_) {
    std::stringstream ss;
    ss << "missing control manager diagnostics";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[MrsTrajectoryGeneration]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  //}

  {
    std::scoped_lock lock(mutex_start_time_total_);

    start_time_total_ = ros::Time::now();
  }

  double path_time_offset = 0;

  if (req.path.header.stamp != ros::Time(0)) {
    path_time_offset = (req.path.header.stamp - ros::Time::now()).toSec();
  }

  if (path_time_offset > 1e-3) {

    std::scoped_lock lock(mutex_max_execution_time_);

    max_execution_time_ = FUTURIZATION_EXEC_TIME_FACTOR * path_time_offset;

    ROS_INFO("[MrsTrajectoryGeneration]: setting the max execution time to %.3f s = %.1f * %.3f", max_execution_time_, FUTURIZATION_EXEC_TIME_FACTOR,
             path_time_offset);
  } else {

    std::scoped_lock lock(mutex_max_execution_time_, mutex_params_);

    max_execution_time_ = params_.max_time;
  }

  ROS_INFO("[MrsTrajectoryGeneration]: got path from service");

  try {
    publisher_original_path_.publish(req.path);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", publisher_original_path_.getTopic().c_str());
  }

  if (req.path.points.empty()) {
    std::stringstream ss;
    ss << "received an empty message";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[MrsTrajectoryGeneration]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  fly_now_                              = req.path.fly_now;
  use_heading_                          = req.path.use_heading;
  frame_id_                             = req.path.header.frame_id;
  override_constraints_                 = req.path.override_constraints;
  loop_                                 = req.path.loop;
  override_max_velocity_horizontal_     = req.path.override_max_velocity_horizontal;
  override_max_velocity_vertical_       = req.path.override_max_velocity_vertical;
  override_max_acceleration_horizontal_ = req.path.override_max_acceleration_horizontal;
  override_max_acceleration_vertical_   = req.path.override_max_acceleration_vertical;
  stop_at_waypoints_                    = req.path.stop_at_waypoints;

  std::vector<Waypoint_t> waypoints;

  for (size_t i = 0; i < req.path.points.size(); i++) {

    double x       = req.path.points[i].position.x;
    double y       = req.path.points[i].position.y;
    double z       = req.path.points[i].position.z;
    double heading = req.path.points[i].heading;

    Waypoint_t wp;
    wp.coords  = Eigen::Vector4d(x, y, z, heading);
    wp.stop_at = stop_at_waypoints_;

    if (!checkNaN(wp)) {
      ROS_ERROR("[MrsTrajectoryGeneration]: NaN detected in waypoint #%d", int(i));
      res.success = false;
      res.message = "invalid path";
      return true;
    }

    waypoints.push_back(wp);
  }

  if (loop_) {
    waypoints.push_back(waypoints[0]);
  }

  bool                          success = false;
  std::string                   message;
  mrs_msgs::TrajectoryReference trajectory;

  for (int i = 0; i < _n_attempts_; i++) {

    auto position_cmd       = mrs_lib::get_mutexed(mutex_position_cmd_, position_cmd_);
    auto current_prediction = mrs_lib::get_mutexed(mutex_prediction_full_state_, prediction_full_state_);

    // the last iteration and the fallback sampling is enabled
    bool fallback_sampling = (_n_attempts_ > 1) && (i == (_n_attempts_ - 1)) && _fallback_sampling_enabled_;

    std::tie(success, message, trajectory) =
        optimize(waypoints, req.path.header, position_cmd, prediction_full_state_, fallback_sampling, req.path.relax_heading);

    if (success) {
      break;
    } else {
      if (i < _n_attempts_) {
        ROS_WARN("[MrsTrajectoryGeneration]: failed to calculate a feasible trajectory, trying again with different initial conditions!");
      } else {
        ROS_WARN("[MrsTrajectoryGeneration]: failed to calculate a feasible trajectory");
      }
    }
  }

  double total_time = (ros::Time::now() - start_time_total_).toSec();

  auto max_execution_time = mrs_lib::get_mutexed(mutex_max_execution_time_, max_execution_time_);

  if (total_time > max_execution_time) {
    ROS_ERROR("[MrsTrajectoryGeneration]: trajectory ready, took %.3f s in total (exceeding maxtime %.3f s by %.3f s)", total_time, max_execution_time,
              total_time - max_execution_time);
  } else {
    ROS_INFO("[MrsTrajectoryGeneration]: trajectory ready, took %.3f s in total (out of %.3f)", total_time, max_execution_time);
  }

  if (success) {

    bool published = trajectorySrv(trajectory);

    if (published) {

      res.success = success;
      res.message = message;

    } else {

      std::stringstream ss;
      ss << "could not publish the trajectory";

      res.success = false;
      res.message = ss.str();

      ROS_ERROR_STREAM("[MrsTrajectoryGeneration]: " << ss.str());
    }

  } else {

    ROS_ERROR("[MrsTrajectoryGeneration]: failed to calculate a feasible trajectory, not publishing a result");

    res.success = success;
    res.message = message;
  }

  return true;
}

//}

/* callbackGetPathSrv() //{ */

bool MrsTrajectoryGeneration::callbackGetPathSrv(mrs_msgs::GetPathSrv::Request& req, mrs_msgs::GetPathSrv::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  /* precondition //{ */

  if (!got_constraints_) {
    std::stringstream ss;
    ss << "missing constraints";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[MrsTrajectoryGeneration]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  if (!got_position_cmd_) {
    std::stringstream ss;
    ss << "missing position cmd";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[MrsTrajectoryGeneration]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  if (!got_prediction_full_state_) {
    std::stringstream ss;
    ss << "missing full-state prediction";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[MrsTrajectoryGeneration]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  if (!got_control_manager_diag_) {
    std::stringstream ss;
    ss << "missing control manager diagnostics";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[MrsTrajectoryGeneration]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  //}

  {
    std::scoped_lock lock(mutex_start_time_total_);

    start_time_total_ = ros::Time::now();
  }

  double path_time_offset = 0;

  if (req.path.header.stamp != ros::Time(0)) {
    path_time_offset = (req.path.header.stamp - ros::Time::now()).toSec();
  }

  if (path_time_offset > 1e-3) {

    std::scoped_lock lock(mutex_max_execution_time_);

    max_execution_time_ = FUTURIZATION_EXEC_TIME_FACTOR * path_time_offset;

    ROS_INFO("[MrsTrajectoryGeneration]: setting the max execution time to %.3f s = %.1f * %.3f", max_execution_time_, FUTURIZATION_EXEC_TIME_FACTOR,
             path_time_offset);
  } else {

    std::scoped_lock lock(mutex_max_execution_time_, mutex_params_);

    max_execution_time_ = params_.max_time;
  }

  ROS_INFO("[MrsTrajectoryGeneration]: got path from service");

  try {
    publisher_original_path_.publish(req.path);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", publisher_original_path_.getTopic().c_str());
  }

  if (req.path.points.empty()) {
    std::stringstream ss;
    ss << "received an empty message";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[MrsTrajectoryGeneration]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  fly_now_                              = req.path.fly_now;
  use_heading_                          = req.path.use_heading;
  frame_id_                             = req.path.header.frame_id;
  override_constraints_                 = req.path.override_constraints;
  loop_                                 = req.path.loop;
  override_max_velocity_horizontal_     = req.path.override_max_velocity_horizontal;
  override_max_velocity_vertical_       = req.path.override_max_velocity_vertical;
  override_max_acceleration_horizontal_ = req.path.override_max_acceleration_horizontal;
  override_max_acceleration_vertical_   = req.path.override_max_acceleration_vertical;
  stop_at_waypoints_                    = req.path.stop_at_waypoints;

  std::vector<Waypoint_t> waypoints;

  for (size_t i = 0; i < req.path.points.size(); i++) {

    double x       = req.path.points[i].position.x;
    double y       = req.path.points[i].position.y;
    double z       = req.path.points[i].position.z;
    double heading = req.path.points[i].heading;

    Waypoint_t wp;
    wp.coords  = Eigen::Vector4d(x, y, z, heading);
    wp.stop_at = stop_at_waypoints_;

    if (!checkNaN(wp)) {
      ROS_ERROR("[MrsTrajectoryGeneration]: NaN detected in waypoint #%d", int(i));
      res.success = false;
      res.message = "invalid path";
      return true;
    }

    waypoints.push_back(wp);
  }

  if (loop_) {
    waypoints.push_back(waypoints[0]);
  }

  bool                          success = false;
  std::string                   message;
  mrs_msgs::TrajectoryReference trajectory;

  for (int i = 0; i < _n_attempts_; i++) {

    auto position_cmd       = mrs_lib::get_mutexed(mutex_position_cmd_, position_cmd_);
    auto current_prediction = mrs_lib::get_mutexed(mutex_prediction_full_state_, prediction_full_state_);

    // the last iteration and the fallback sampling is enabled
    bool fallback_sampling = (_n_attempts_ > 1) && (i == (_n_attempts_ - 1)) && _fallback_sampling_enabled_;

    std::tie(success, message, trajectory) =
        optimize(waypoints, req.path.header, position_cmd, prediction_full_state_, fallback_sampling, req.path.relax_heading);

    if (success) {
      break;
    } else {
      if (i < _n_attempts_) {
        ROS_WARN("[MrsTrajectoryGeneration]: failed to calculate a feasible trajectory, trying again with different initial conditions!");
      } else {
        ROS_WARN("[MrsTrajectoryGeneration]: failed to calculate a feasible trajectory");
      }
    }
  }

  double total_time = (ros::Time::now() - start_time_total_).toSec();

  auto max_execution_time = mrs_lib::get_mutexed(mutex_max_execution_time_, max_execution_time_);

  if (total_time > max_execution_time) {
    ROS_ERROR("[MrsTrajectoryGeneration]: trajectory ready, took %.3f s in total (exceeding maxtime %.3f s by %.3f s)", total_time, max_execution_time,
              total_time - max_execution_time);
  } else {
    ROS_INFO("[MrsTrajectoryGeneration]: trajectory ready, took %.3f s in total (out of %.3f)", total_time, max_execution_time);
  }

  if (success) {

    res.trajectory = trajectory;
    res.success    = success;
    res.message    = message;

  } else {

    ROS_ERROR("[MrsTrajectoryGeneration]: failed to calculate a feasible trajectory");

    res.success = success;
    res.message = message;
  }

  return true;
}

//}

/* callbackConstraints() //{ */

void MrsTrajectoryGeneration::callbackConstraints(const mrs_msgs::DynamicsConstraintsConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsTrajectoryGeneration]: got constraints");

  mrs_lib::set_mutexed(mutex_constraints_, *msg, constraints_);

  got_constraints_ = true;
}

//}

/* callbackPositionCmd() //{ */

void MrsTrajectoryGeneration::callbackPositionCmd(const mrs_msgs::PositionCommandConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsTrajectoryGeneration]: got position cmd");

  got_position_cmd_ = true;

  transformer_->setCurrentControlFrame(msg->header.frame_id);

  mrs_lib::set_mutexed(mutex_position_cmd_, *msg, position_cmd_);
}

//}

/* callbackPredictionFullState() //{ */

void MrsTrajectoryGeneration::callbackPredictionFullState(const mrs_msgs::MpcPredictionFullStateConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsTrajectoryGeneration]: got full-state prediction");

  got_prediction_full_state_ = true;

  mrs_lib::set_mutexed(mutex_prediction_full_state_, *msg, prediction_full_state_);
}

//}

/* callbackControlManagerDiag() //{ */

void MrsTrajectoryGeneration::callbackControlManagerDiag(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsTrajectoryGeneration]: got control manager diagnostics");

  got_control_manager_diag_ = true;

  mrs_lib::set_mutexed(mutex_control_manager_diag_, *msg, control_manager_diag_);
}

//}

/* //{ callbackDrs() */

void MrsTrajectoryGeneration::callbackDrs(mrs_uav_trajectory_generation::drsConfig& params, [[maybe_unused]] uint32_t level) {

  mrs_lib::set_mutexed(mutex_params_, params, params_);

  {
    std::scoped_lock lock(mutex_max_execution_time_);

    max_execution_time_ = params.max_time;
  }

  ROS_INFO("[MrsTrajectoryGeneration]: DRS updated");
}

//}

}  // namespace mrs_uav_trajectory_generation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_trajectory_generation::MrsTrajectoryGeneration, nodelet::Nodelet);
