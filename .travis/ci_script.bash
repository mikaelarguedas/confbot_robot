#!/bin/bash
set -e

cd $ROS2_WS

# install dependencies
rosdep update && apt-get -qq update && rosdep install -y \
  --from-paths src \
  --ignore-src

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# build
colcon build \
    --symlink-install \
    --cmake-args -DSECURITY=ON --no-warn-unused-cli
# test
colcon test \
    --executor sequential \
    --packages-skip gazebo_dev gazebo_msgs gazebo_plugins gazebo_ros gazebo_ros_pkgs \
      cv_bridge image_geometry opencv_tests vision_opencv \
      camera_calibration_parsers camera_info_manager image_common image_transport polled_camera \
    --event-handlers console_direct+
colcon test-result
