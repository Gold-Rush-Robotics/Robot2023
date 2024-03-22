#!/usr/bin/env bash

echo "hello world"

source /opt/ros/humble/setup.bash
source /robotics/workspace/install/setup.bash

ros2 launch grr_bringup bloodstone.launch.py