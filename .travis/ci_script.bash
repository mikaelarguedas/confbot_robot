#!/bin/bash
set -e

cd $ROS2_WS

# Installing crystal packages on nightly may lead to problem, commenting out for now
# install dependencies
rosdep update && apt-get -qq update && rosdep install -y \
  --from-paths src \
  --ignore-src \
  --rosdistro $ROS_DISTRO

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# build
colcon build \
    --symlink-install \
    --cmake-args -DSECURITY=ON --no-warn-unused-cli
# test
colcon test \
    --executor sequential \
    --event-handlers console_direct+
colcon test-result
