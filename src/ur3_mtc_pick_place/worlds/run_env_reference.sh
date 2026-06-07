#!/bin/bash

export ROS_DOMAIN_ID=15
source /opt/ros/humble/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$(pwd)/models

echo "Starting UR3 Gazebo world..."

ros2 launch ur_simulation_gz ur_sim_control.launch.py \
  ur_type:=ur3 \
  world_file:=$(pwd)/worlds/empty_with_sensors.sdf &

echo "Waiting for Gazebo to start..."
sleep 8

echo "Spawning convenience shelf..."

ros2 run ros_gz_sim create \
  -file $(pwd)/models/convenience_shelf.sdf \
  -name convenience_shelf \
  -x 0.55 -y 0.0 -z -0.05 \
  -Y 1.5708

sleep 1

echo "Spawning target coke can..."

ros2 run ros_gz_sim create \
  -file $(pwd)/models/coke_can/model.sdf \
  -name target_bottle \
  -x 0.47 -y 0.0 -z 0.39

sleep 1

echo "Spawning ball distractor..."

ros2 run ros_gz_sim create \
  -file $(pwd)/models/robocup_3Dsim_ball/model.sdf \
  -name ball \
  -x 0.50 -y 0.30 -z 0.40

sleep 1

echo "Spawning toaster distractor..."

ros2 run ros_gz_sim create \
  -file $(pwd)/models/toaster/model.sdf \
  -name toaster \
  -x 0.50 -y -0.38 -z 0.39 \
  -Y 1.5708

echo "Environment setup complete."
echo "UR3 convenience shelf environment is running."

wait