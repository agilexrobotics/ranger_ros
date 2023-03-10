# ranger_ros

## ranger

* ranger mini  is the name of the following car

  ![ranger mini](./images/ranger_mini.png)

## Communicate Flow

![ranger_mini](./images/ranger_mini_car.png)

* The `ugv_sdk` communicate with the car by `can protocol`
* The `ranger_ros` package call the function `GetRangerState` to get the newest machine information, call the `SetMotionCommand` to set linear velocity 、angle、etc, call the `SetMotionMode` to change the motion mode
* Ranger mini have 4 type of motion mode. see ranger mini manual from    [agilex develop manuals](https://github.com/westonrobot/ugv_sdk/tree/master/docs) 
* pub `ranger_ros` can set motion mode
* subscribe `ranger_status` can get the robot status

## Params

see `ranger_base/launch/ranger_mini_base.launch`

* is_ranger_mini :   ranger mini or ranger pro
* port_name:   can port name , usually is can0
* simulated_robot:   sim robot for test or not
* odom_frame:   the odometry frame name in tf
* base_frame:   the base link frame name in tf
* odom_topic_name:   the odometry topic name
* **pub_odom_tf:  publish tf transformation of odometry frame or not.  if true publish** 

## Build

dependencies:

* ROS1 melodic or newer

assume your ros workspace is ~/agilex_ws

```shell
# install ugv_sdk
cd ~/agilex_ws/src
git clone https://github.com/westonrobot/async_port.git
git clone https://github.com/westonrobot/ugv_sdk.git
cd ugv_sdk
git checkout -b v2.x origin/v2.x
cd ../
catkin_make install --pkg ugv_sdk

# source the packages
source devel/setup.bash

# install ranger_ros
cd ~/agilex_ws/src/
git clone https://github.com/agilexrobotics/ranger_ros.git

# install the ascent library at first
cd ranger_ros/ranger_base/ascent
mkdir -p build && cd build && cmake -DBUILD_TESTING=OFF .. && sudo make install

cd ~/agilex_ws/
catkin_make install # or just catkin_make

# if you catch error:  ranger_msgs/RangerSetting.h: No such file or directory
# then install `ranger_msgs` first
catkin_make install --pkg ranger_msgs

# for install
catkin_make install
```

## Run

```shell
# run ranger_ros
cd ~/agilex_ws
source devel/setup.bash
roslaunch ranger_bringup ranger_minimal.launch

```

use keyboard to control
> the default motion mode is Head-Back  Ackermann
```shell
# if you want to remote control the car by keyboard
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard
roslaunch ranger_bringup ranger_teleop_keyboard.launch
```







----

## Ros Topic Examples

* Input:   the car linear velocity and the heading angle
* Output:  total linear velocity, x direction velocity, y direction velocity, angular velocity, central steer angle and rotate radius, .etc

### Publish topic to control the car

see `ranger_ros/ranger_examples/src/input.cpp` for details

```c++
////----------------control by ros topic---------------------------------
ros::Publisher motion_mode =
    node.advertise<ranger_msgs::RangerSetting>("/ranger_setting", 1);
ranger_msgs::RangerSetting setting;
setting.motion_mode = ranger_msgs::RangerSetting::MOTION_MODE_ACKERMAN;
motion_mode.publish(setting);

////------------------move by ros topic --------------------------------
ros::Publisher move_cmd =
    node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
geometry_msgs::Twist cmd;
cmd.linear.x = 0.1;                   // the motor will run at 0.1m/s
cmd.angular.z = 30.0 / 180.0 * M_PI;  // the heading angle of the car

// publish robot state at 50Hz while listening to twist commands
ros::Rate rate(50);
while (ros::ok()) {
    ros::spinOnce();

    // /cmd_vel topic must send at 50Hz, even stop need send 0m/s
    move_cmd.publish(cmd);

    rate.sleep();
}
```

or publish by command line

```shell
# 0 Ackrmann, 1 Slide, 2 Round, 3 Sloping
# 0 前后阿克曼，1 斜移,   2 自旋,  3  侧移
rostopic pub -1 /ranger_setting ranger_msgs/RangerSetting -- '[0, 0, setting_frame]' '1'

rostopic pub /cmd_vel geometry_msgs/Twist --rate 50 '[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.52358]'  # 0.52358 = 30 degree
```



### Subcribe the car output

see `ranger_ros/ranger_examples/src/output.cpp` for details

```c++
ros::Subscriber status_sub = node.subscribe<ranger_msgs::RangerStatus>(
    "/ranger_status", 10, StatusCallback);
```

```c++
void StatusCallback(ranger_msgs::RangerStatus::ConstPtr msg) {
    std::cout << "linear velocity: " << msg->linear_velocity << std::endl;
    std::cout << "angular velocity: " << msg->angular_velocity << std::endl;
    std::cout << "x direction linear velocity: " << msg->x_linear_vel << std::endl;
    std::cout << "y direction linear linear velocity: " << msg->y_linear_vel << std::endl;
    std::cout << "rotate radius: " << msg->motion_radius << std::endl;
    std::cout << "car heading angle: " << msg->steering_angle << std::endl;
    // ...etc
}
```



or show the data by rostopic 

```shell
rostopic echo /ranger_status
```







----

## Sdk API Examples

### 0. enable can control

```shell
robot->Connect("can0");
robot->EnableCommandedMode();
```



### 1. set motion mode

see `ranger_ros/ranger_examples/src/change_the_mode.cpp` for more details

```c++
robot->Connect("can0");
robot->EnableCommandedMode();

// 0 Arckmann 1 Slide 2 round, 3 Sloping
// 0 前后阿克曼 1 横移  2 自旋  3 侧移 
robot->SetMotionMode(0);
//  robot->SetMotionMode(1);
//  robot->SetMotionMode(2);
//  robot->SetMotionMode(3);
```



### 2. set the movement linear velocity and angle of the car

see `ranger_ros/ranger_examples/src/control_the_car.cpp` for more details

```c++
robot->SetMotionCommand(0.1, 30.0/180.0 * M_PI); // steer angle = 30°

// or write them in a function
void Ackermann() {
  robot->SetMotionMode(0);
  // or
  robot->SetMotionMode(RangerSetting::MOTION_MODE_ACKERMAN);
  l_v = 0.1;                  // m/s
  angle_z = 30 / 180 * M_PI;  // rad
}
void Slide() {
  robot->SetMotionMode(1);
  // or
  robot->SetMotionMode(RangerSetting::MOTION_MODE_SLIDE);
  l_v = 0.1;                  // m/s
  angle_z = 30 / 180 * M_PI;  // rad
}
void Round() {
  robot->SetMotionMode(2);
  // or
  robot->SetMotionMode(RangerSetting::MOTION_MODE_ROUND);
  l_v = 0.1;
  // angle_z is not used
}
void Sloping() {
  robot->SetMotionMode(3);
  // or
  robot->SetMotionMode(RangerSetting::MOTION_MODE_SLOPING);
  l_v = 0.1;
  // angle_z is not used
}
```

## use odometry

* subscribe `/odom` topic

