/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_msgs/TrajectoryReferenceSrv.h>
#include <mrs_msgs/TrajectoryReference.h>

#include <std_srvs/Trigger.h>

#include <mrs_msgs/DynamicsConstraints.h>
#include <mrs_msgs/Path.h>

#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

#include <mav_msgs/eigen_mav_msgs.h>

#include <mutex>

//}

namespace trajectory_generation
{

/* class TrajectoryGeneration //{ */
class TrajectoryGeneration : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool is_initialized_ = false;

  bool               callbackTest(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer service_server_path_;

  void                          callbackConstraints(const mrs_msgs::DynamicsConstraintsConstPtr& msg);
  ros::Subscriber               subscriber_constraints_;
  bool                          got_constraints_ = false;
  mrs_msgs::DynamicsConstraints constraints_;
  std::mutex                    mutex_constraints_;

  void            callbackPath(const mrs_msgs::PathConstPtr& msg);
  ros::Subscriber subscriber_path_;

  ros::ServiceClient service_client_trajectory_reference_;
};
//}

/* onInit() //{ */

void TrajectoryGeneration::onInit() {

  /* obtain node handle */
  ros::NodeHandle nh_("~");

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ----------------------- subscribers ---------------------- |

  subscriber_constraints_ = nh_.subscribe("constraints_in", 1, &TrajectoryGeneration::callbackConstraints, this, ros::TransportHints().tcpNoDelay());
  subscriber_path_        = nh_.subscribe("path_in", 1, &TrajectoryGeneration::callbackPath, this, ros::TransportHints().tcpNoDelay());

  // | --------------------- service servers -------------------- |

  service_server_path_ = nh_.advertiseService("test_in", &TrajectoryGeneration::callbackTest, this);

  service_client_trajectory_reference_ = nh_.serviceClient<mrs_msgs::TrajectoryReferenceSrv>("trajectory_reference_out");

  // | --------------------- service clients -------------------- |

  ROS_INFO_ONCE("[TrajectoryGeneration]: initialized");

  is_initialized_ = true;
}
//}

// | ------------------------ callbacks ----------------------- |

/* callbackTest() //{ */

bool TrajectoryGeneration::callbackTest(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  if (!got_constraints_) {
    std::stringstream ss;
    ss << "missing constraints";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[TrajectoryGeneration]: " << ss.str());
    res.success = false;
    res.message = ss.str();
  }

  mrs_msgs::DynamicsConstraints constraints;

  mutex_constraints_.lock();
  constraints = constraints_;
  mutex_constraints_.unlock();

  mav_trajectory_generation::NonlinearOptimizationParameters parameters;

  parameters.max_iterations                  = 1000;
  parameters.f_rel                           = 0.05;
  parameters.x_rel                           = 0.1;
  parameters.time_penalty                    = 500;
  parameters.initial_stepsize_rel            = 0.1;
  parameters.inequality_constraint_tolerance = 1e10;
  parameters.equality_constraint_tolerance   = 1e10;
  /* parameters.equality_constraint_tolerance   = 1.0; */

  /* parameters.time_alloc_method = parameters.kRichterTimeAndConstraints; */

  /* parameters.time_alloc_method = parameters.kRichterTime; */

  /* parameters.time_alloc_method = parameters.kMellingerOuterLoop; */
  /* parameters.algorithm         = nlopt::LD_LBFGS; */

  /* parameters.equality_constraint_tolerance   = 1.0; */

  mav_trajectory_generation::Vertex::Vector vertices;
  const int                                 dimension              = 3;
  const int                                 derivative_to_optimize = mav_trajectory_generation::derivative_order::ACCELERATION;

  // | --------------- add constraints to vertices -------------- |

  {
    mav_trajectory_generation::Vertex vertex(dimension);
    vertex.makeStartOrEnd(Eigen::Vector3d(0, 0, 1), derivative_to_optimize);
    vertex.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(0, 0, 0));
    vertices.push_back(vertex);
  }

  {
    mav_trajectory_generation::Vertex vertex(dimension);
    vertex.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(2, 2, 1));
    vertices.push_back(vertex);
  }

  {
    mav_trajectory_generation::Vertex vertex(dimension);
    vertex.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(4, 0, 1));
    vertices.push_back(vertex);
  }

  {
    mav_trajectory_generation::Vertex vertex(dimension);
    vertex.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(6, 2, 1));
    vertices.push_back(vertex);
  }

  {
    mav_trajectory_generation::Vertex vertex(dimension);
    vertex.makeStartOrEnd(Eigen::Vector3d(8, 0, 1), derivative_to_optimize);
    vertices.push_back(vertex);
  }

  // | ---------------- compute the segment times --------------- |

  const double v_max = constraints.horizontal_speed;
  const double a_max = constraints.horizontal_acceleration;

  ROS_INFO("[TrajectoryGeneration]: constraints: v_max %.2f, a_max %.2f", v_max, a_max);

  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, v_max, a_max);

  for (size_t i = 0; i < segment_times.size(); i++) {
    ROS_INFO_STREAM("[TrajectoryGeneration]: segment: " << segment_times[i]);
  }

  // | --------- create an optimizer object and solve it -------- |

  const int                                                     N = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);
  opt.optimize();

  // | ------------- obtain the polynomial segments ------------- |

  mav_trajectory_generation::Segment::Vector segments;
  opt.getPolynomialOptimizationRef().getSegments(&segments);

  // | --------------- create the trajectory class -------------- |

  mav_trajectory_generation::Trajectory trajectory;
  opt.getTrajectory(&trajectory);

  // | ------------------ sample the trajectory ----------------- |

  mav_msgs::EigenTrajectoryPoint         state;
  mav_msgs::EigenTrajectoryPoint::Vector states;

  // Whole trajectory:
  double sampling_interval = 0.1;
  bool   success           = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);

  if (success) {

    mrs_msgs::TrajectoryReferenceSrv srv;

    srv.request.trajectory.dt          = sampling_interval;
    srv.request.trajectory.fly_now     = true;
    srv.request.trajectory.use_heading = true;

    for (size_t it = 0; it < states.size(); it++) {

      mrs_msgs::Reference point;
      point.heading    = 0;
      point.position.x = states[it].position_W[0];
      point.position.y = states[it].position_W[1];
      point.position.z = states[it].position_W[2];

      srv.request.trajectory.points.push_back(point);
    }

    service_client_trajectory_reference_.call(srv);
  }

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

  ROS_INFO("[TrajectoryGeneration]: got trajectory");

  mrs_msgs::DynamicsConstraints constraints;

  mutex_constraints_.lock();
  constraints = constraints_;
  mutex_constraints_.unlock();

  mrs_msgs::Path path = *msg;

  if (path.points.size() == 0) {
    ROS_ERROR_THROTTLE(1.0, "[TrajectoryGeneration]: trajectory is empty");
    return;
  }

  mav_trajectory_generation::NonlinearOptimizationParameters parameters;
  parameters.max_iterations                  = 1000;
  parameters.f_rel                           = 0.05;
  parameters.x_rel                           = 0.1;
  parameters.time_penalty                    = 500.0;
  parameters.initial_stepsize_rel            = 0.1;
  parameters.inequality_constraint_tolerance = 0.1;
  /* parameters.equality_constraint_tolerance   = 1.0; */

  mav_trajectory_generation::Vertex::Vector vertices;
  const int                                 dimension              = 4;
  const int                                 derivative_to_optimize = mav_trajectory_generation::derivative_order::JERK;
  mav_trajectory_generation::Vertex         vertex(dimension);

  // | --------------- add constraints to vertices -------------- |

  double lx, ly, lz;

  for (size_t i = 0; i < path.points.size(); i++) {

    double x       = path.points[i].position.x;
    double y       = path.points[i].position.y;
    double z       = path.points[i].position.z;
    double heading = path.points[i].heading;

    if (i == 0) {

      ROS_INFO("[TrajectoryGeneration]: first point x %.2f, y %.2f, z %.2f, h %.2f", x, y, z, heading);

      vertex.makeStartOrEnd(Eigen::Vector4d(x, y, z, heading), mav_trajectory_generation::derivative_order::POSITION);
      vertices.push_back(vertex);

    } else if (i == (path.points.size() - 1)) {

      ROS_INFO("[TrajectoryGeneration]: last point x %.2f, y %.2f, z %.2f, h %.2f", x, y, z, heading);

      if (sqrt(pow(lx - x, 2) + pow(ly - y, 2) + pow(lz - z, 2)) <= 1.0) {
        ROS_INFO("[TrajectoryGeneration]: skipping");
        continue;
      }

      vertex.makeStartOrEnd(Eigen::Vector4d(x, y, z, heading), mav_trajectory_generation::derivative_order::POSITION);
      vertices.push_back(vertex);

    } else {

      ROS_INFO("[TrajectoryGeneration]: mid point x %.2f, y %.2f, z %.2f, h %.2f", x, y, z, heading);

      if (sqrt(pow(lx - x, 2) + pow(ly - y, 2) + pow(lz - z, 2)) <= 1.0) {
        ROS_INFO("[TrajectoryGeneration]: skipping");
        continue;
      }

      vertex.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector4d(x, y, z, heading));
      vertices.push_back(vertex);
    }

    lx = x;
    ly = y;
    lz = z;
  }


  // | ---------------- compute the segment times --------------- |

  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, constraints.horizontal_speed, constraints.horizontal_acceleration);

  // | --------- create an optimizer object and solve it -------- |

  const int                                                     N = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, constraints.horizontal_speed);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, constraints.horizontal_acceleration);
  /* opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::JERK, constraints.horizontal_jerk); */
  opt.optimize();

  // | ------------- obtain the polynomial segments ------------- |

  mav_trajectory_generation::Segment::Vector segments;
  opt.getPolynomialOptimizationRef().getSegments(&segments);

  // | --------------- create the trajectory class -------------- |

  mav_trajectory_generation::Trajectory trajectory;
  opt.getTrajectory(&trajectory);

  // | ------------------ sample the trajectory ----------------- |

  mav_msgs::EigenTrajectoryPoint         state;
  mav_msgs::EigenTrajectoryPoint::Vector states;

  // Whole trajectory:
  double sampling_interval = 0.1;
  bool   success           = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);

  if (success) {

    mrs_msgs::TrajectoryReferenceSrv srv;

    srv.request.trajectory.header      = path.header;
    srv.request.trajectory.dt          = sampling_interval;
    srv.request.trajectory.fly_now     = true;
    srv.request.trajectory.use_heading = true;

    for (size_t it = 0; it < states.size(); it++) {

      mrs_msgs::Reference point;
      point.heading    = 0;
      point.position.x = states[it].position_W[0];
      point.position.y = states[it].position_W[1];
      point.position.z = states[it].position_W[2];
      point.heading    = states[it].position_W[3];

      srv.request.trajectory.points.push_back(point);
    }

    service_client_trajectory_reference_.call(srv);
  }
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

}  // namespace trajectory_generation

/* every nodelet must export its class as nodelet plugin */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(trajectory_generation::TrajectoryGeneration, nodelet::Nodelet);
