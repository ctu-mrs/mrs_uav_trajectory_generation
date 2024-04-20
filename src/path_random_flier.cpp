/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <stdio.h>
#include <stdlib.h>

#include <mrs_msgs/PathSrv.h>
#include <mrs_msgs/Reference.h>
#include <mrs_msgs/ValidateReference.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/msg_extractor.h>
#include <mrs_lib/transformer.h>

#include <std_srvs/Trigger.h>

#include <random>

//}

namespace mrs_uav_trajectory_generation
{

/* class PathRandomFlier //{ */

class PathRandomFlier : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool is_initialized_ = false;

  bool   callbackActivate([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  void   timerMain(const ros::TimerEvent& event);
  double randd(const double from, const double to);
  int    randi(const int from, const int to);
  bool   setPathSrv(const mrs_msgs::Path path_in);

  bool checkReference(const std::string frame, const double x, const double y, const double z, const double hdg);

  mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>            sh_tracker_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_;

  std::optional<mrs_msgs::TrackerCommand> transformTrackerCmd(const mrs_msgs::TrackerCommand& tracker_cmd, const std::string& target_frame);

  std::shared_ptr<mrs_lib::Transformer> transformer_;

  ros::Publisher publisher_goto_;

  ros::ServiceServer service_server_activate_;

  ros::ServiceClient service_client_path_;
  ros::ServiceClient service_client_check_reference_;

  ros::Timer timer_main_;

  std::string _frame_id_;
  std::string _uav_name_;

  double _main_timer_rate_;

  bool _relax_heading_;
  bool _use_heading_;

  int _n_points_min_;
  int _n_points_max_;

  double _point_distance_min_;
  double _point_distance_max_;

  double _z_value_;
  double _z_deviation_;

  double _future_stamp_prob_;
  double _future_stamp_min_;
  double _future_stamp_max_;

  double _replanning_time_min_;
  double _replanning_time_max_;

  double _heading_change_;
  double _bearing_change_;
  double _initial_bearing_change_;

  bool   _override_constraints_;
  double _override_speed_;
  double _override_acceleration_;

  bool active_ = true;

  int path_id_ = 0;

  bool      next_wait_for_finish_ = false;
  ros::Time next_replan_time_;

  ros::Time last_successfull_command_;

  double bearing_ = 0;
};

//}

/* onInit() //{ */

void PathRandomFlier::onInit(void) {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "PathRandomFlier");

  param_loader.loadParam("main_timer_rate", _main_timer_rate_);
  param_loader.loadParam("active", active_);

  param_loader.loadParam("frame_id", _frame_id_);
  param_loader.loadParam("uav_name", _uav_name_);

  param_loader.loadParam("relax_heading", _relax_heading_);
  param_loader.loadParam("use_heading", _use_heading_);

  param_loader.loadParam("heading_change", _heading_change_);
  param_loader.loadParam("bearing_change", _bearing_change_);
  param_loader.loadParam("initial_bearing_change", _initial_bearing_change_);
  param_loader.loadParam("n_points/min", _n_points_min_);
  param_loader.loadParam("n_points/max", _n_points_max_);
  param_loader.loadParam("point_distance/min", _point_distance_min_);
  param_loader.loadParam("point_distance/max", _point_distance_max_);
  param_loader.loadParam("z/value", _z_value_);
  param_loader.loadParam("z/deviation", _z_deviation_);

  param_loader.loadParam("future_stamp/prob", _future_stamp_prob_);
  param_loader.loadParam("future_stamp/min", _future_stamp_min_);
  param_loader.loadParam("future_stamp/max", _future_stamp_max_);

  param_loader.loadParam("replanning_time/min", _replanning_time_min_);
  param_loader.loadParam("replanning_time/max", _replanning_time_max_);

  param_loader.loadParam("override_constraints/enabled", _override_constraints_);
  param_loader.loadParam("override_constraints/speed", _override_speed_);
  param_loader.loadParam("override_constraints/acceleration", _override_acceleration_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[PathRandomFlier]: Could not load all parameters!");
    ros::shutdown();
  }

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "PathRandomFlier";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;

  sh_tracker_cmd_          = mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>(shopts, "tracker_command_in");
  sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diag_in");

  service_server_activate_        = nh_.advertiseService("activate_in", &PathRandomFlier::callbackActivate, this);
  service_client_path_            = nh_.serviceClient<mrs_msgs::PathSrv>("path_out");
  service_client_check_reference_ = nh_.serviceClient<mrs_msgs::ValidateReference>("check_reference_out");

  // initialize the random number generator
  srand(static_cast<unsigned int>(ros::Time::now().nsec));
  /* srand(time(NULL)); */

  last_successfull_command_ = ros::Time(0);

  timer_main_ = nh_.createTimer(ros::Rate(_main_timer_rate_), &PathRandomFlier::timerMain, this);

  transformer_ = std::make_shared<mrs_lib::Transformer>(nh_, "PathRandomFlier");
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);

  // | ----------------------- finish init ---------------------- |

  is_initialized_ = true;

  ROS_INFO("[PathRandomFlier]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackActivate() //{ */

bool PathRandomFlier::callbackActivate([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  active_ = true;

  res.success = true;
  res.message = "activated";

  return true;
}

//}

// | ------------------------- timers ------------------------- |

/* timerMain() //{ */

void PathRandomFlier::timerMain([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  if (!active_) {

    ROS_INFO_ONCE("[PathRandomFlier]: waiting for initialization");
    return;
  }

  if (!sh_tracker_cmd_.hasMsg()) {

    ROS_INFO_THROTTLE(1.0, "waiting for TrackerCommand");
    return;
  }

  if (!sh_tracker_cmd_.getMsg()->use_full_state_prediction) {

    ROS_INFO_THROTTLE(1.0, "waiting for prediction");
    return;
  }

  if (!sh_control_manager_diag_.hasMsg()) {

    ROS_INFO_THROTTLE(1.0, "waiting for ControlManager diagnostics");
    return;
  }

  bool has_goal = sh_control_manager_diag_.getMsg()->tracker_status.have_goal;

  auto tracker_cmd_transformed = transformTrackerCmd(*sh_tracker_cmd_.getMsg(), _uav_name_ + "/" + _frame_id_);

  if (!tracker_cmd_transformed) {
    std::stringstream ss;
    ss << "could not transform tracker_cmd to the path frame";
    ROS_ERROR_STREAM("[MrsTrajectoryGeneration]: " << ss.str());
    return;
  }

  auto [cmd_x, cmd_y, cmd_z] = mrs_lib::getPosition(tracker_cmd_transformed.value());

  // if the uav reached the previousy set destination
  if ((ros::Time::now() - last_successfull_command_).toSec() > 1.0 &&
      (!has_goal || (!next_wait_for_finish_ && (next_replan_time_ - ros::Time::now()).toSec() < 0))) {

    // create new point to fly to
    mrs_msgs::Path path;
    path.fly_now       = true;
    path.use_heading   = _use_heading_;
    path.relax_heading = _relax_heading_;

    double pos_x, pos_y, pos_z;

    if (!next_wait_for_finish_) {

      double time_offset = randd(_future_stamp_min_, _future_stamp_max_);

      int prediction_idx = int(round((time_offset - 0.01) / 0.2));

      mrs_msgs::ReferenceStamped new_point;

      auto prediction = sh_tracker_cmd_.getMsg()->full_state_prediction;

      new_point.header               = prediction.header;
      new_point.reference.position.x = prediction.position.at(prediction_idx).x;
      new_point.reference.position.y = prediction.position.at(prediction_idx).y;
      new_point.reference.position.z = prediction.position.at(prediction_idx).z;
      new_point.reference.heading    = prediction.heading.at(prediction_idx);

      auto res = transformer_->transformSingle(new_point, _frame_id_);

      if (res) {
        new_point = res.value();
      } else {
        std::stringstream ss;
        ss << "could not transform initial condition to the desired frame";
        ROS_ERROR_STREAM("[MrsTrajectoryGeneration]: " << ss.str());
        return;
      }

      if (has_goal) {
        path.header.stamp = ros::Time::now() + ros::Duration(time_offset);
      } else {
        path.header.stamp = ros::Time(0);
      }

      pos_x    = new_point.reference.position.x;
      pos_y    = new_point.reference.position.y;
      pos_z    = new_point.reference.position.z;
      bearing_ = new_point.reference.heading;

      path.points.push_back(new_point.reference);

    } else {

      pos_x = cmd_x;
      pos_y = cmd_y;
      pos_z = cmd_z;

      path.header.stamp = ros::Time(0);
    }

    path.header.frame_id = _uav_name_ + "/" + _frame_id_;

    bearing_ += randd(-_initial_bearing_change_, _initial_bearing_change_);

    double heading = randd(-M_PI, M_PI);

    ROS_INFO("[PathRandomFlier]: pos_x: %.2f, pos_y: %.2f, pos_z: %.2f", pos_x, pos_y, pos_z);

    int n_points = randi(_n_points_min_, _n_points_max_);

    for (int it = 0; it < n_points; it++) {

      ROS_INFO("[PathRandomFlier]: check pos_x: %.2f, pos_y: %.2f, pos_z: %.2f", pos_x, pos_y, pos_z);
      if (!checkReference(_uav_name_ + "/" + _frame_id_, pos_x, pos_y, pos_z, bearing_)) {
        break;
      }

      bearing_ += randd(-_bearing_change_, _bearing_change_);

      heading += randd(-_heading_change_, _heading_change_);

      double distance = randd(_point_distance_min_, _point_distance_max_);

      pos_x += cos(bearing_) * distance;
      pos_y += sin(bearing_) * distance;
      pos_z = _z_value_ + randd(-_z_deviation_, _z_deviation_);

      mrs_msgs::Reference new_point;
      new_point.position.x = pos_x;
      new_point.position.y = pos_y;
      new_point.position.z = pos_z;
      new_point.heading    = bearing_;

      ROS_INFO("[PathRandomFlier]: pos_x: %.2f, pos_y: %.2f, pos_z: %.2f", pos_x, pos_y, pos_z);

      path.points.push_back(new_point);
    }

    next_wait_for_finish_ = randd(0, 10) <= 10 * _future_stamp_prob_ ? false : true;

    if (!next_wait_for_finish_) {
      double replan_time = randd(_replanning_time_min_, _replanning_time_max_);
      next_replan_time_  = ros::Time::now() + ros::Duration(replan_time);
      ROS_INFO("[PathRandomFlier]: replanning in %.2f s", replan_time);
    }

    if (_override_constraints_) {

      path.override_constraints = true;

      path.override_max_velocity_horizontal = _override_speed_;
      path.override_max_velocity_vertical   = _override_speed_;

      path.override_max_acceleration_horizontal = _override_acceleration_;
      path.override_max_acceleration_vertical   = _override_acceleration_;

      ROS_INFO_THROTTLE(1.0, "[PathRandomFlier]: overriding constraints to speed: %.2f m/s, acc: %.2f m/s2", path.override_max_velocity_horizontal,
                        path.override_max_acceleration_horizontal);
    }

    if (setPathSrv(path)) {

      ROS_INFO("[PathRandomFlier]: path set");

      last_successfull_command_ = ros::Time::now();
    }
  }
}  // namespace mrs_uav_testing_old

//}

// | ------------------------ routines ------------------------ |

/* randd() //{ */

double PathRandomFlier::randd(const double from, const double to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return (to - from) * zero_to_one + from;
}

//}

/* randi() //{ */

int PathRandomFlier::randi(const int from, const int to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return int(double(to - from) * zero_to_one + from);
}

//}

/* setPathSrv() //{ */

bool PathRandomFlier::setPathSrv(const mrs_msgs::Path path_in) {

  mrs_msgs::PathSrv srv;
  srv.request.path = path_in;

  srv.request.path.input_id = path_id_++;

  bool success = service_client_path_.call(srv);

  if (success) {

    if (!srv.response.success) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "[PathRandomFlier]: service call for setting path failed: " << srv.response.message);
      return false;
    } else {
      return true;
    }

  } else {
    ROS_ERROR_THROTTLE(1.0, "[PathRandomFlier]: service call for setting path failed");
    return false;
  }
}

//}

/* checkReference() //{ */

bool PathRandomFlier::checkReference(const std::string frame, const double x, const double y, const double z, const double hdg) {

  mrs_msgs::ValidateReference srv;
  srv.request.reference.header.frame_id      = frame;
  srv.request.reference.reference.position.x = x;
  srv.request.reference.reference.position.y = y;
  srv.request.reference.reference.position.z = z;
  srv.request.reference.reference.heading    = hdg;

  bool success = service_client_check_reference_.call(srv);

  if (success) {

    return srv.response.success;

  } else {
    ROS_ERROR_THROTTLE(1.0, "[PathRandomFlier]: service call for setting path failed");
    return false;
  }
}

//}

/* transformTrackerCmd() //{ */

std::optional<mrs_msgs::TrackerCommand> PathRandomFlier::transformTrackerCmd(const mrs_msgs::TrackerCommand& tracker_cmd, const std::string& target_frame) {

  // if we transform to the current control frame, which is in fact the same frame as the tracker_cmd is in
  if (target_frame == "") {
    return tracker_cmd;
  }

  // find the transformation
  auto tf = transformer_->getTransform(tracker_cmd.header.frame_id, target_frame, tracker_cmd.header.stamp);

  if (!tf) {
    ROS_ERROR("[MrsTrajectoryGeneration]: could not find transform from '%s' to '%s' in time %f", tracker_cmd.header.frame_id.c_str(), target_frame.c_str(),
              tracker_cmd.header.stamp.toSec());
    return {};
  }

  mrs_msgs::TrackerCommand cmd_out;

  cmd_out.header.stamp    = tf.value().header.stamp;
  cmd_out.header.frame_id = transformer_->frame_to(tf.value());

  /* position + heading //{ */

  {
    geometry_msgs::PoseStamped pos;
    pos.header = tracker_cmd.header;

    pos.pose.position    = tracker_cmd.position;
    pos.pose.orientation = mrs_lib::AttitudeConverter(0, 0, tracker_cmd.heading);

    if (auto ret = transformer_->transform(pos, tf.value())) {
      cmd_out.position = ret.value().pose.position;
      try {
        cmd_out.heading = mrs_lib::AttitudeConverter(ret.value().pose.orientation).getHeading();
      }
      catch (...) {
        ROS_ERROR("[MrsTrajectoryGeneration]: failed to transform heading in tracker_cmd");
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
    vec.header = tracker_cmd.header;

    vec.vector = tracker_cmd.velocity;

    if (auto ret = transformer_->transform(vec, tf.value())) {
      cmd_out.velocity = ret.value().vector;
    } else {
      return {};
    }
  }

  //}

  /* acceleration //{ */

  {
    geometry_msgs::Vector3Stamped vec;
    vec.header = tracker_cmd.header;

    vec.vector = tracker_cmd.acceleration;

    if (auto ret = transformer_->transform(vec, tf.value())) {
      cmd_out.acceleration = ret.value().vector;
    } else {
      return {};
    }
  }

  //}

  /* jerk //{ */

  {
    geometry_msgs::Vector3Stamped vec;
    vec.header = tracker_cmd.header;

    vec.vector = tracker_cmd.jerk;

    if (auto ret = transformer_->transform(vec, tf.value())) {
      cmd_out.jerk = ret.value().vector;
    } else {
      return {};
    }
  }

  //}

  /* heading derivatives //{ */

  // this does not need to be transformed
  cmd_out.heading_rate         = tracker_cmd.heading_rate;
  cmd_out.heading_acceleration = tracker_cmd.heading_acceleration;
  cmd_out.heading_jerk         = tracker_cmd.heading_jerk;

  //}

  return cmd_out;
}

//}

}  // namespace mrs_uav_trajectory_generation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_trajectory_generation::PathRandomFlier, nodelet::Nodelet)
