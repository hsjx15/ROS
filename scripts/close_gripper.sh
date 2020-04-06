#!/bin/bash

rostopic pub /gripper_controller/command trajectory_msgs/JointTrajectory "
header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names: ['left_gripper_joint', 'right_gripper_joint']
points:
  -
    positions: [0.0, 0.0]
    velocities: []
    accelerations: []
    effort: []
    time_from_start: {secs: 5, nsecs: 0}" -1
