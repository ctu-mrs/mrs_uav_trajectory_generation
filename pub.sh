rostopic pub /uav1/trajectory_generation/path mrs_msgs/PathWithVelocity "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
use_heading: true
fly_now: true
stop_at_waypoints: false
loop: false
override_constraints: false
override_max_velocity_horizontal: 5.0
override_max_acceleration_horizontal: 1.0
override_max_velocity_vertical: 3.0
override_max_acceleration_vertical: 1.0
relax_heading: false
points:
- position: {x: 0.0, y: 0.0, z: 5.0}
  enforce_velocity: false
  velocity: {x: 0.0, y: 0.0, z: 5.0}
  heading: 0.0
- position: {x: -25.0, y: 0.0, z: 5.0}
  enforce_velocity: true
  velocity: {x: -5.0, y: 0.0, z: 5.0}
  heading: 0.0
- position: {x: -25.0, y: 25.0, z: 5.0}
  enforce_velocity: false
  velocity: {x: 0.0, y: 0.0, z: 5.0}
  heading: 0.0
- position: {x: 0.0, y: 25.0, z: 5.0}
  enforce_velocity: false
  velocity: {x: 0.0, y: 0.0, z: 5.0}
  heading: 0.0
  "
