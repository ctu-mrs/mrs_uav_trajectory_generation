/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <stdlib.h>

#include <mrs_msgs/srv/path_srv.hpp>
#include <mrs_msgs/msg/path.hpp>
#include <mrs_msgs/msg/reference.hpp>
#include <mrs_msgs/srv/validate_reference.hpp>
#include <mrs_msgs/msg/control_manager_diagnostics.hpp>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/msg_extractor.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/node.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_server_handler.h>
#include <mrs_lib/service_client_handler.h>

#include <std_srvs/srv/trigger.hpp>

//}

/* typedefs //{ */

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

//}

namespace mrs_uav_trajectory_generation
{

/* class PathRandomFlier //{ */

class PathRandomFlier : public mrs_lib::Node {

public:
  PathRandomFlier(rclcpp::NodeOptions options);

private:
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;

  bool is_initialized_ = false;

  bool   callbackActivate(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void   timerMain();
  double randd(const double from, const double to);
  int    randi(const int from, const int to);
  bool   setPathSrv(const mrs_msgs::msg::Path path_in);

  bool checkReference(const std::string frame, const double x, const double y, const double z, const double hdg);

  mrs_lib::SubscriberHandler<mrs_msgs::msg::TrackerCommand>            sh_tracker_cmd_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics> sh_control_manager_diag_;

  std::optional<mrs_msgs::msg::TrackerCommand> transformTrackerCmd(const mrs_msgs::msg::TrackerCommand &tracker_cmd, const std::string &target_frame);

  std::shared_ptr<mrs_lib::Transformer> transformer_;

  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger> ss_activate_;

  mrs_lib::ServiceClientHandler<mrs_msgs::srv::PathSrv>           sc_path_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::ValidateReference> sc_check_reference_;

  std::shared_ptr<TimerType> timer_main_;

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

  bool next_wait_for_finish_ = false;

  rclcpp::Time next_replan_time_;
  rclcpp::Time last_successfull_command_;

  double bearing_ = 0;
};

//}

/* PathRandomFlier() //{ */

PathRandomFlier::PathRandomFlier(rclcpp::NodeOptions options) : mrs_lib::Node("path_random_flier", options), clock_(this_node().get_clock()) {

  cbkgrp_subs_ = this_node().create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_ss_   = this_node().create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_sc_   = this_node().create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  mrs_lib::ParamLoader param_loader(this_node_ptr());

  std::string custom_config_path;

  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("private_config");

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
    RCLCPP_ERROR(this_node().get_logger(), "Could not load all parameters!");
    rclcpp::shutdown();
    exit(1);
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node                                = this_node_ptr();
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.subscription_options.callback_group = cbkgrp_subs_;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;

  sh_tracker_cmd_          = mrs_lib::SubscriberHandler<mrs_msgs::msg::TrackerCommand>(shopts, "~/tracker_command_in");
  sh_control_manager_diag_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics>(shopts, "~/control_manager_diag_in");

  // | --------------------- service clients -------------------- |

  rclcpp::Node::SharedPtr node = this_node_ptr();
  sc_path_                     = mrs_lib::ServiceClientHandler<mrs_msgs::srv::PathSrv>(node, "~/path_out", cbkgrp_sc_);
  sc_check_reference_          = mrs_lib::ServiceClientHandler<mrs_msgs::srv::ValidateReference>(node, "~/check_reference_out", cbkgrp_sc_);

  // | --------------------- service servers -------------------- |

  ss_activate_ = mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node, "~/activate_in", std::bind(&PathRandomFlier::callbackActivate, this, std::placeholders::_1, std::placeholders::_2), cbkgrp_ss_);

  // | ------------------------- timers ------------------------- |

  // | ------------------------- timers ------------------------- |

  mrs_lib::TimerHandlerOptions timer_opts_start;

  timer_opts_start.node      = this_node_ptr();
  timer_opts_start.autostart = true;

  {
    std::function<void()> callback_fcn = std::bind(&PathRandomFlier::timerMain, this);

    timer_main_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(_main_timer_rate_, clock_), callback_fcn);
  }

  // | ----------------------- randomizer ----------------------- |

  // initialize the random number generator
  srand(static_cast<unsigned int>(clock_->now().nanoseconds()));
  /* srand(time(NULL)); */

  last_successfull_command_ = rclcpp::Time(0, 0, clock_->get_clock_type());
  next_replan_time_         = rclcpp::Time(0, 0, clock_->get_clock_type());

  // | --------------------- tf transformer --------------------- |

  transformer_ = std::make_shared<mrs_lib::Transformer>(this_node_ptr());
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);

  // | ----------------------- finish init ---------------------- |

  is_initialized_ = true;

  RCLCPP_INFO(this_node().get_logger(), "initialied");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackActivate() //{ */

bool PathRandomFlier::callbackActivate([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                       const std::shared_ptr<std_srvs::srv::Trigger::Response>                 response) {

  if (!is_initialized_) {
    return false;
  }

  active_ = true;

  response->success = true;
  response->message = "activated";

  return true;
}

//}

// | ------------------------- timers ------------------------- |

/* timerMain() //{ */

void PathRandomFlier::timerMain(void) {

  if (!is_initialized_) {
    return;
  }

  if (!active_) {

    RCLCPP_INFO_ONCE(this_node().get_logger(), "waiting for innitialization");
    return;
  }

  if (!sh_tracker_cmd_.hasMsg()) {

    RCLCPP_INFO_THROTTLE(this_node().get_logger(), *clock_, 1000, "waiting for tracker command");
    return;
  }

  if (!sh_tracker_cmd_.getMsg()->use_full_state_prediction) {

    RCLCPP_INFO_THROTTLE(this_node().get_logger(), *clock_, 1000, "waiting for MPC prediction");
    return;
  }

  if (!sh_control_manager_diag_.hasMsg()) {

    RCLCPP_INFO_THROTTLE(this_node().get_logger(), *clock_, 1000, "waiting for control manager diagnostics");
    return;
  }

  bool has_goal = sh_control_manager_diag_.getMsg()->tracker_status.have_goal;

  auto tracker_cmd_transformed = transformTrackerCmd(*sh_tracker_cmd_.getMsg(), _uav_name_ + "/" + _frame_id_);

  if (!tracker_cmd_transformed) {
    std::stringstream ss;
    ss << "could not transform tracker_cmd to the path frame";
    RCLCPP_ERROR(this_node().get_logger(), "%s", ss.str().c_str());
    return;
  }

  auto [cmd_x, cmd_y, cmd_z] = mrs_lib::getPosition(tracker_cmd_transformed.value());

  // if the uav reached the previousy set destination
  if ((clock_->now() - last_successfull_command_).seconds() > 1.0 &&
      (!has_goal || (!next_wait_for_finish_ && (next_replan_time_ - clock_->now()).seconds() < 0))) {

    // create new point to fly to
    mrs_msgs::msg::Path path;
    path.fly_now       = true;
    path.use_heading   = _use_heading_;
    path.relax_heading = _relax_heading_;

    double pos_x, pos_y, pos_z;

    if (!next_wait_for_finish_) {

      double time_offset = randd(_future_stamp_min_, _future_stamp_max_);

      int prediction_idx = int(round((time_offset - 0.01) / 0.2));

      mrs_msgs::msg::ReferenceStamped new_point;

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
        RCLCPP_ERROR(this_node().get_logger(), "%s", ss.str().c_str());
        return;
      }

      if (has_goal) {
        path.header.stamp = clock_->now() + std::chrono::duration<double>(time_offset);
      } else {
        path.header.stamp = rclcpp::Time(0);
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

      path.header.stamp = rclcpp::Time(0);
    }

    path.header.frame_id = _uav_name_ + "/" + _frame_id_;

    bearing_ += randd(-_initial_bearing_change_, _initial_bearing_change_);

    double heading = randd(-M_PI, M_PI);

    RCLCPP_INFO(this_node().get_logger(), "pos_x: %.2f, pos_y: %.2f, pos_z: %.2f", pos_x, pos_y, pos_z);

    int n_points = randi(_n_points_min_, _n_points_max_);

    for (int it = 0; it < n_points; it++) {

      RCLCPP_INFO(this_node().get_logger(), "check pos_x: %.2f, pos_y: %.2f, pos_z: %.2f", pos_x, pos_y, pos_z);
      if (!checkReference(_uav_name_ + "/" + _frame_id_, pos_x, pos_y, pos_z, bearing_)) {
        break;
      }

      bearing_ += randd(-_bearing_change_, _bearing_change_);

      heading = heading + randd(-_heading_change_, _heading_change_);

      double distance = randd(_point_distance_min_, _point_distance_max_);

      pos_x += cos(bearing_) * distance;
      pos_y += sin(bearing_) * distance;
      pos_z = _z_value_ + randd(-_z_deviation_, _z_deviation_);

      mrs_msgs::msg::Reference new_point;
      new_point.position.x = pos_x;
      new_point.position.y = pos_y;
      new_point.position.z = pos_z;
      new_point.heading    = bearing_;

      RCLCPP_INFO(this_node().get_logger(), "pos_x: %.2f, pos_y: %.2f, pos_z: %.2f", pos_x, pos_y, pos_z);

      path.points.push_back(new_point);
    }

    next_wait_for_finish_ = randd(0, 10) <= 10 * _future_stamp_prob_ ? false : true;

    if (!next_wait_for_finish_) {
      double replan_time = randd(_replanning_time_min_, _replanning_time_max_);
      next_replan_time_  = clock_->now() + std::chrono::duration<double>(replan_time);
      RCLCPP_INFO(this_node().get_logger(), "replanning in %.2f s", replan_time);
    }

    if (_override_constraints_) {

      path.override_constraints = true;

      path.override_max_velocity_horizontal = _override_speed_;
      path.override_max_velocity_vertical   = _override_speed_;

      path.override_max_acceleration_horizontal = _override_acceleration_;
      path.override_max_acceleration_vertical   = _override_acceleration_;

      RCLCPP_INFO_THROTTLE(this_node().get_logger(), *clock_, 1000, "overriding constraints to speed: %.2f m/s, acc: %.2f m/s2",
                           path.override_max_velocity_horizontal, path.override_max_acceleration_horizontal);
    }

    if (setPathSrv(path)) {

      RCLCPP_INFO(this_node().get_logger(), "path set");

      last_successfull_command_ = clock_->now();
    }
  }
} // namespace mrs_uav_testing_old

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

bool PathRandomFlier::setPathSrv(const mrs_msgs::msg::Path path_in) {

  std::shared_ptr<mrs_msgs::srv::PathSrv::Request> request = std::make_shared<mrs_msgs::srv::PathSrv::Request>();

  request->path = path_in;

  request->path.input_id = path_id_++;

  auto response = sc_path_.callSync(request);

  if (response) {

    if (!response.value()->success) {
      RCLCPP_ERROR_THROTTLE(this_node().get_logger(), *clock_, 1000, "service call for setting path failed: %s", response.value()->message.c_str());
      return false;
    } else {
      return true;
    }

  } else {
    RCLCPP_ERROR_THROTTLE(this_node().get_logger(), *clock_, 1000, "service call for path failed");
    return false;
  }
}

//}

/* checkReference() //{ */

bool PathRandomFlier::checkReference(const std::string frame, const double x, const double y, const double z, const double hdg) {

  std::shared_ptr<mrs_msgs::srv::ValidateReference::Request> request = std::make_shared<mrs_msgs::srv::ValidateReference::Request>();
  request->reference.header.frame_id                                 = frame;
  request->reference.reference.position.x                            = x;
  request->reference.reference.position.y                            = y;
  request->reference.reference.position.z                            = z;
  request->reference.reference.heading                               = hdg;

  auto response = sc_check_reference_.callSync(request);

  if (response) {

    if (!response.value()->success) {
      return false;
    } else {
      return true;
    }

  } else {
    RCLCPP_ERROR_THROTTLE(this_node().get_logger(), *clock_, 1000, "service call for reference validation");
    return false;
  }
}

//}

/* transformTrackerCmd() //{ */

std::optional<mrs_msgs::msg::TrackerCommand> PathRandomFlier::transformTrackerCmd(const mrs_msgs::msg::TrackerCommand &tracker_cmd,
                                                                                  const std::string                   &target_frame) {

  // if we transform to the current control frame, which is in fact the same frame as the tracker_cmd is in
  if (target_frame == "") {
    return tracker_cmd;
  }

  // find the transformation
  auto tf = transformer_->getTransform(tracker_cmd.header.frame_id, target_frame, tracker_cmd.header.stamp);

  if (!tf) {
    RCLCPP_ERROR(this_node().get_logger(), "could not find transform from '%s' to '%s' in time %f", tracker_cmd.header.frame_id.c_str(), target_frame.c_str(),
                 rclcpp::Time(tracker_cmd.header.stamp).seconds());
    return {};
  }

  mrs_msgs::msg::TrackerCommand cmd_out;

  cmd_out.header.stamp    = tf.value().header.stamp;
  cmd_out.header.frame_id = transformer_->frame_to(tf.value());

  /* position + heading //{ */

  {
    geometry_msgs::msg::PoseStamped pos;
    pos.header = tracker_cmd.header;

    pos.pose.position    = tracker_cmd.position;
    pos.pose.orientation = mrs_lib::AttitudeConverter(0, 0, tracker_cmd.heading);

    if (auto ret = transformer_->transform(pos, tf.value())) {
      cmd_out.position = ret.value().pose.position;
      try {
        cmd_out.heading = mrs_lib::AttitudeConverter(ret.value().pose.orientation).getHeading();
      }
      catch (...) {
        RCLCPP_ERROR(this_node().get_logger(), "failed to transform heading in tracker_cmd");
        cmd_out.heading = 0;
      }
    } else {
      return {};
    }
  }

  //}

  /* velocity //{ */

  {
    geometry_msgs::msg::Vector3Stamped vec;
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
    geometry_msgs::msg::Vector3Stamped vec;
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
    geometry_msgs::msg::Vector3Stamped vec;
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

} // namespace mrs_uav_trajectory_generation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_uav_trajectory_generation::PathRandomFlier)
