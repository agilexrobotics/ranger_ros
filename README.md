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

1. Install dependencies

```bash
$ sudo apt install libasio-dev libboost-all-dev
$ sudo apt install -y ros-$ROS_DISTRO-teleop-twist-keyboard
```

2. Clone and build the packages in a catkin workspace

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/agilexrobotics/ugv_sdk.git
$ git clone https://github.com/agilexrobotics/ranger_ros.git
$ cd ..
$ catkin_make
```
3. Setup CAN-To-USB adapter

* Enable gs_usb kernel module(If you have already added this module, you do not need to add it)
    ```
    $ sudo modprobe gs_usb
    ```
    
* first time use scout-ros package
   ```
   $ rosrun ranger_bringup setup_can2usb.bash
   ```
   
* if not the first time use scout-ros package(Run this command every time you turn off the power) 
   ```
   $ rosrun ranger_bringup bringup_can2usb.bash
   ```
   
* Testing command
    ```
    # receiving data from can0
    $ candump can0
    ```

4. Launch ROS nodes

* Start the base node for ranger

    ```shell
    $ roslaunch ranger_bringup ranger.launch #for ranger
    ```

* Start the base node for ranger_mini_v1

    ```shell
    $ roslaunch ranger_bringup ranger_mini_v1.launch #for ranger_mini 1.0
    ```

* Start the base node for ranger_mini_v2

    ```bash
    $ roslaunch ranger_bringup ranger_mini_v2.launch #for ranger_mini 2.0
    ```
* Use keyboard to control ranger
    ```bash
    $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
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
* /battery_state (ranger_msgs::BatteryState)

### Subscribed topics

* /cmd_vel (geometry_msgs::Twist)

### Services
