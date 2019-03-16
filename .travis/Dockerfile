FROM osrf/ros2:nightly

# install packages from the ROS repositories
RUN apt-get -qq update && apt-get -qq install -y --no-install-recommends \
    python3-catkin-pkg-modules \
    python3-colcon-common-extensions \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# install python packages
RUN pip3 install -U \
    argcomplete \
    flake8 \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures

# clone source
ENV ROS2_WS /root/ros2_ws
RUN mkdir -p $ROS2_WS/src/roscon2018
COPY ./ci_script.bash $ROS2_WS
COPY ./workspace.repos $ROS2_WS
WORKDIR $ROS2_WS
RUN vcs import src < workspace.repos

CMD ["bash"]
