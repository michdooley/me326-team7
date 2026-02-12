#!/bin/bash
# Quick test script for grasp planner service

ros2 service call /plan_grasp tidybot_msgs/srv/PlanGrasp "{
  object_position: {header: {frame_id: 'camera_link'}, point: {x: 0.4, y: 0.0, z: 0.0}},
  object_class: 'block',
  arm_name: 'right'
}"
