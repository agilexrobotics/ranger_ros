# ROS Packages for Ranger Robot

This repository contains ROS support packages for the Ranger robot bases to provide a ROS interface to the robot.

## Supported hardware

* Ranger Mini V1.0
<img src="./docs/ranger_mini_v1.png" width="350" />

* Ranger Mini V2.0
<img src="./docs/ranger_mini_v2.png" width="350" />

* Ranger
<img src="./docs/ranger.png" width="300" />

## Build the package

* Install dependencies

```bash
$ sudo apt install libasio-dev libboost-all-dev
```

* Clone and build the packages in a catkin workspace

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/westonrobot/ugv_sdk.git
$ git clone https://github.com/westonrobot/ranger_ros.git
$ cd ..
$ catkin_make
```

## ROS interface

### Parameters

* can_device (string): **can0**
* robot_model (string): **ranger**/ranger_mini_v1/ranger_mini_v2
* update_rate (int): **50**
* base_frame (string): **base_link**
* odom_frame (string): **odom**
* publish_odom_tf (bool): **true**
* odom_topic_name (string): **odom**

### Published topics

* /system_state (ranger_msgs::SystemState)
* /motion_state (ranger_msgs::MotionState)
* /actuator_state (ranger_msgs::ActuatorStateArray)
* /odom (nav_msgs::Odometry)
* /battery_state (sensor_msgs::BatteryState)

### Subscribed topics

* /cmd_vel (geometry_msgs::Twist)

### Services
