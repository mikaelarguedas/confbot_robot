# roscon2018
workspace comprising demo packages for our roscon2018 talk

# versions
The code found in the `master` branch is targeted towards the latest source build of ROS2. In case you're building against the latest binary release (`crystal`), you're finding a compatible tag to it.

# travis
linux: [![Build Status](https://travis-ci.org/Karsten1987/roscon2018.svg?branch=master)](https://travis-ci.org/Karsten1987/roscon2018)

# How to run the demo

## Prerequisites

- Have ROS2 installed, here assumed ROS Crystal located as /opr/ros/crystal/setup.bash
- ROS 2 development tools (`sudo apt install python3-colcon-common-extensions python3-rosdep`)

## Getting and building the code

```bash
export CONFBOT_WS=$HOME/confbot_ws
mkdir -p $CONFBOT_WS/src && cd $CONFBOT_WS/src && git clone https://github.com/Karsten1987/roscon2018 && cd $CONFBOT_WS
```

### Adapting the code for your machine:
```bash
cd $CONFBOT_WS
sed -i s@/Users/karsten/workspace/osrf/roscon2018_ws/@$CONFBOT_WS/@g `grep -lr "/karsten/"  --exclude README.md`
```

### Installing the dependencies
```bash
sudo rosdep init
rosdep update
cd $CONFBOT_WS
rosdep install -y --from-paths src --ignore-src --rosdistro crystal
```

### Source crystal environment:
```bash
source /opt/ros/crystal/setup.bash
```

### Build the workspace:
```bash
cd $CONFBOT_WS
colcon build --merge-install --cmake-args --no-warn-unused-cli -DSECURITY=ON
```

### Run the demo:
In each terminal from now on, always assume use a sourced environment:
```
source /opt/ros/crystal/setup.bash
source $CONFBOT_WS/install/setup.bash
```

#### Use CLI to introspect and interact with the system

##### Launch the demo
Terminal 1:
```
ros2 launch confbot_bringup confbot_bringup.launch.py
```

##### Introspect the system
Terminal 2:
```
$ ros2 node list

/launch_ros
/robot_state_publisher
/confbot_driver
/twist_publisher
/confbot_laser
/safe_zone_publisher
```

```
$ ros2 topic list -t

/cmd_vel [geometry_msgs/Twist]
/confbot_laser/transition_event [lifecycle_msgs/TransitionEvent]
/danger_zone [visualization_msgs/Marker]
/joint_states [sensor_msgs/JointState]
/parameter_events [rcl_interfaces/ParameterEvent]
/robot_description [std_msgs/String]
/rosout [rcl_interfaces/Log]
/safe_zone [visualization_msgs/Marker]
/tf [tf2_msgs/TFMessage]
/tf_static [tf2_msgs/TFMessage]
```

```
$ ros2 topic echo /cmd_vel

linear:
  x: 0.10000000149011612
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.10000000149011612

linear:
  x: 0.10000000149011612
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.10000000149011612

^C
```


##### Visualize the data
```
ros2 run rviz2 rviz2 -d `ros2 pkg prefix confbot_bringup --share`/config/confbot.rviz
```

##### Set parameters
The node `/twist_publisher` has a speed parameter.

```
$ ros2 param set /twist_publisher speed 0.2
Set parameter successful
```

In Terminal 1:
```
[INFO] [twist_publisher]: set parameter 'speed' to "0.200000"
```

In RViz: robot goes twice as fast


Introspecting lifecycle nodes:
```
$ ros2 lifecycle nodes
/confbot_laser
```

```
$ ros2 lifecycle list /confbot_laser
- configure [1]
	Start: unconfigured
	Goal: configuring
- shutdown [5]
	Start: unconfigured
	Goal: shuttingdown
```

```
$ ros2 lifecycle set /confbot_laser 1
Transitioning successful
```

```
$ ros2 lifecycle list /confbot_laser
- cleanup [2]
	Start: inactive
	Goal: cleaningup
- activate [3]
	Start: inactive
	Goal: activating
- shutdown [6]
	Start: inactive
	Goal: shuttingdown
```

```
$ ros2 lifecycle set /confbot_laser 3
```
In RViz: the laser node is now publishing fake laser data!

##### Compromise the system
Tempering with the system:
```
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0}" -r 100
```
Robot stops moving



#### Use ROS 2 security

We will now use ROS2 Security to harden the system.

Prerequisite: Terminate all the running nodes by Ctrl+C them.


##### Create security artifacts:
```
export ROS_SECURITY_ROOT_DIRECTORY=$CONFBOT_WS/confbot_keystore
cd
colcon build --cmake-args -DSECURITY=ON -DPOLICY_FILE=$CONFBOT_WS/src/roscon2018/confbot_security/safe_zone_publisher_policies.xml --packages-select confbot_security --cmake-force-configure
```

##### Launch the demo in secure mode

All terminals are assumed to have the environment setup and security environment variables set
```
source /opt/ros/crystal/setup.bash
source $CONFBOT_WS/install/setup.bash
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
export ROS_SECURITY_ROOT_DIRECTORY=$CONFBOT_WS/confbot_keystore
```

Terminal 1: launch the system
```
ros2 launch confbot_bringup confbot_bringup_activates.launch.py
```

Terminal 2: visualize the system
```
ros2 run rviz2 rviz2 -d `ros2 pkg prefix confbot_bringup --share`/config/confbot.rviz
```

##### Attempt tempering with the system

```
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0}" -r 100
```


```
ros2 run confbot_driver twist_publisher __node:=my_hacky_node
```


##### Restrict individual node permission

