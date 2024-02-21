#!/bin/bash

rostopic echo -n1 /robot1/pioneer2dx/ground_truth/pose
rosservice call /gazebo/apply_body_wrench "body_name: 'robot1::base_link'
reference_frame: 'robot1::base_link'
reference_point: {x: 0.2, y: 0.0, z: -0.05}
wrench:
  force: {x: $1, y: 0.0, z: 0.0}
  torque: {x: 0.0, y: 0.0, z: 0.0}
start_time: {secs: 0, nsecs: 0}
duration: {secs: 10, nsecs: 0}"
rostopic echo -n1 /robot1/pioneer2dx/ground_truth/pose
