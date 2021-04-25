# ranger_ros

## ranger

* ranger mini  is the name of the following car

  ![ranger mini](./images/ranger_mini.png)

## Communicate Flow

![ranger_mini](./images/ranger_mini_car.png)

* The `ugv_sdk` communicate with the car by `can protocol`
* The `ranger_ros` package call the function `GetRangerState` to get the newest machine information, call the `SetMotionCommand` to set linear velocity 、angle、etc, call the `SetMotionMode` to change the motion mode
* Ranger mini have 4 type of motion mode. see ranger mini manual from    [agilex develop manuals](https://github.com/westonrobot/ugv_sdk/tree/master/docs) 

## Build

dependencies:

* ROS1 melodic or newer

assume your ros workspace is ~/agilex_ws

```shell
cd ~/agilex_ws/src
git clone https://github.com/westonrobot/ugv_sdk.git
git clone https://github.com/westonrobot/ranger_ros.git

cd ..
catkin_make 

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

## Examples

### 0. enable can control

```shell
  robot->Connect("can0");
  robot->EnableCommandedMode();
```



### 1. set motion mode

see `ranger_ros/ranger_examples/src/change_the_mode.cpp` for more details

```c++

```



### 2. set the movement linear velocity and angle of the car

see `ranger_ros/ranger_examples/src/control_the_car.cpp` for more details

```c++

```



