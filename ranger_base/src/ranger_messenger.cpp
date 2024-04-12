/**
* @file ranger_messenger.cpp
* @date 2021-04-20
* @brief
*
# @copyright Copyright (c) 2021 AgileX Robotics
* @copyright Copyright (c) 2023 Weston Robot Pte. Ltd.
*/

#include "ranger_base/ranger_messenger.hpp"

#include <cmath>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include "ranger_msgs/ActuatorState.h"
#include "ranger_msgs/DriverState.h"
#include "ranger_msgs/MotorState.h"
#include "ranger_msgs/MotionState.h"
#include "ranger_msgs/SystemState.h"
#include "ranger_msgs/TriggerParkMode.h"

#include "ranger_base/ranger_params.hpp"
#include "ranger_base/kinematics_model.hpp"

using namespace ros;
using namespace ranger_msgs;

namespace westonrobot {
namespace {
double DegreeToRadian(double x) { return x * M_PI / 180.0; }
}  // namespace

///////////////////////////////////////////////////////////////////////////////////

RangerROSMessenger::RangerROSMessenger(ros::NodeHandle* nh) : nh_(nh) {
  LoadParameters();

  // connect to robot and setup ROS subscription
  if (robot_type_ == RangerSubType::kRangerMiniV1) {
    robot_ = std::make_shared<RangerRobot>(true);
  } else {
    robot_ = std::make_shared<RangerRobot>(false);
  }

  if (port_name_.find("can") != std::string::npos) {
    if (!robot_->Connect(port_name_)) {
      ROS_ERROR("Failed to connect to the CAN port");
      ros::shutdown();
    }
    robot_->EnableCommandedMode();
  } else {
    ROS_ERROR("Invalid port name: %s", port_name_.c_str());
    ros::shutdown();
  }

  SetupSubscription();
}

void RangerROSMessenger::Run() {
  ros::Rate rate(update_rate_);
  while (ros::ok()) {
    PublishStateToROS();
    ros::spinOnce();
    rate.sleep();
  }
}

void RangerROSMessenger::LoadParameters() {
  // load parameter from launch files
  nh_->param<std::string>("port_name", port_name_, std::string("can0"));
  nh_->param<std::string>("robot_model", robot_model_, std::string("ranger"));
  nh_->param<std::string>("odom_frame", odom_frame_, std::string("odom"));
  nh_->param<std::string>("base_frame", base_frame_, std::string("base_link"));
  nh_->param<int>("update_rate", update_rate_, 50);
  nh_->param<std::string>("odom_topic_name", odom_topic_name_,
                          std::string("odom"));
  nh_->param<bool>("publish_odom_tf", publish_odom_tf_, false);

  ROS_INFO(
      "Successfully loaded the following parameters: \n port_name: %s\n "
      "robot_model: %s\n odom_frame: %s\n base_frame: %s\n "
      "update_rate: %d\n odom_topic_name: %s\n "
      "publish_odom_tf: %d\n",
      port_name_.c_str(), robot_model_.c_str(), odom_frame_.c_str(),
      base_frame_.c_str(), update_rate_, odom_topic_name_.c_str(),
      publish_odom_tf_);

  // load robot parameters
  if (robot_model_ == "ranger_mini_v1") {
    robot_type_ = RangerSubType::kRangerMiniV1;

    robot_params_.track = RangerMiniV1Params::track;
    robot_params_.wheelbase = RangerMiniV1Params::wheelbase;
    robot_params_.max_linear_speed = RangerMiniV1Params::max_linear_speed;
    robot_params_.max_angular_speed = RangerMiniV1Params::max_angular_speed;
    robot_params_.max_speed_cmd = RangerMiniV1Params::max_speed_cmd;
    robot_params_.max_steer_angle_ackermann =
        RangerMiniV1Params::max_steer_angle_ackermann;
    robot_params_.max_steer_angle_parallel =
        RangerMiniV1Params::max_steer_angle_parallel;
    robot_params_.max_round_angle = RangerMiniV1Params::max_round_angle;
    robot_params_.min_turn_radius = RangerMiniV1Params::min_turn_radius;
  } else {
    if (robot_model_ == "ranger_mini_v2") {
      robot_type_ = RangerSubType::kRangerMiniV2;

      robot_params_.track = RangerMiniV2Params::track;
      robot_params_.wheelbase = RangerMiniV2Params::wheelbase;
      robot_params_.max_linear_speed = RangerMiniV2Params::max_linear_speed;
      robot_params_.max_angular_speed = RangerMiniV2Params::max_angular_speed;
      robot_params_.max_speed_cmd = RangerMiniV2Params::max_speed_cmd;
      robot_params_.max_steer_angle_ackermann =
          RangerMiniV2Params::max_steer_angle_ackermann;
      robot_params_.max_steer_angle_parallel =
          RangerMiniV2Params::max_steer_angle_parallel;
      robot_params_.max_round_angle = RangerMiniV2Params::max_round_angle;
      robot_params_.min_turn_radius = RangerMiniV2Params::min_turn_radius;
    } else {
      robot_type_ = RangerSubType::kRanger;

      robot_params_.track = RangerParams::track;
      robot_params_.wheelbase = RangerParams::wheelbase;
      robot_params_.max_linear_speed = RangerParams::max_linear_speed;
      robot_params_.max_angular_speed = RangerParams::max_angular_speed;
      robot_params_.max_speed_cmd = RangerParams::max_speed_cmd;
      robot_params_.max_steer_angle_ackermann =
          RangerParams::max_steer_angle_ackermann;
      robot_params_.max_steer_angle_parallel =
          RangerParams::max_steer_angle_parallel;
      robot_params_.max_round_angle = RangerParams::max_round_angle;
      robot_params_.min_turn_radius = RangerParams::min_turn_radius;
    }
  }
  parking_mode_ = false;
}

void RangerROSMessenger::SetupSubscription() {
  // publisher
  system_state_pub_ =
      nh_->advertise<ranger_msgs::SystemState>("/system_state", 10);
  motion_state_pub_ =
      nh_->advertise<ranger_msgs::MotionState>("/motion_state", 10);
  actuator_state_pub_ =
      nh_->advertise<ranger_msgs::ActuatorStateArray>("/actuator_state", 10);
  odom_pub_ = nh_->advertise<nav_msgs::Odometry>(odom_topic_name_, 10);
  battery_state_pub_ =
      nh_->advertise<ranger_msgs::BatteryState>("/battery_state", 10);

  // subscriber
  motion_cmd_sub_ = nh_->subscribe<geometry_msgs::Twist>(
      "/cmd_vel", 5, &RangerROSMessenger::TwistCmdCallback, this);
  light_cmd_subscriber_ = nh_->subscribe<ranger_msgs::RangerLightCmd>(
      "/ranger_light_control", 5, &RangerROSMessenger::LightCmdCallback, this);


  // service server
  trigger_parking_server = nh_->advertiseService(
      "parking_service", &RangerROSMessenger::TriggerParkingService, this);
}

void RangerROSMessenger::PublishStateToROS() {
  current_time_ = ros::Time::now();

  static bool init_run = true;
  if (init_run) {
    last_time_ = current_time_;
    init_run = false;
    return;
  }

  auto state = robot_->GetRobotState();
  auto actuator_state = robot_->GetActuatorState();

  // update odometry
  {
    double dt = (current_time_ - last_time_).toSec();
    UpdateOdometry(state.motion_state.linear_velocity,
                   state.motion_state.angular_velocity,
                   state.motion_state.steering_angle, dt);
    last_time_ = current_time_;
  }

  // publish system state
  {
    ranger_msgs::SystemState system_msg;
    system_msg.header.stamp = current_time_;
    system_msg.vehicle_state = state.system_state.vehicle_state;
    system_msg.control_mode = state.system_state.control_mode;
    system_msg.error_code = state.system_state.error_code;
    system_msg.battery_voltage = state.system_state.battery_voltage;
    system_msg.motion_mode = state.motion_mode_state.motion_mode;

    system_state_pub_.publish(system_msg);
  }

  // publish motion mode
  {
    motion_mode_ = state.motion_mode_state.motion_mode;

    ranger_msgs::MotionState motion_msg;
    motion_msg.header.stamp = current_time_;
    motion_msg.motion_mode = state.motion_mode_state.motion_mode;

    motion_state_pub_.publish(motion_msg);
  }

  // publish actuator state
  {
    ROS_DEBUG_NAMED("feedback", "Angle_5:%f Angle_6:%f Angle_7:%f Angle_8:%f",
                    actuator_state.motor_angles.angle_5,
                    actuator_state.motor_angles.angle_6,
                    actuator_state.motor_angles.angle_7,
                    actuator_state.motor_angles.angle_8);
    ROS_DEBUG_NAMED("feedback", "speed_1:%f speed_2:%f speed_3:%f speed_4:%f",
                    actuator_state.motor_speeds.speed_1,
                    actuator_state.motor_speeds.speed_2,
                    actuator_state.motor_speeds.speed_3,
                    actuator_state.motor_speeds.speed_4);

    ranger_msgs::ActuatorStateArray actuator_msg;
    actuator_msg.header.stamp = current_time_;
    for (int i = 0; i < 8; i++) {
      ranger_msgs::DriverState driver_state_msg;
      driver_state_msg.driver_voltage =
          actuator_state.actuator_ls_state->driver_voltage;
      driver_state_msg.driver_temperature =
          actuator_state.actuator_ls_state->driver_temp;
      driver_state_msg.motor_temperature =
          actuator_state.actuator_ls_state->motor_temp;
      driver_state_msg.driver_state =
          actuator_state.actuator_ls_state->driver_state;

      ranger_msgs::MotorState motor_state_msg;
      motor_state_msg.rpm =
          actuator_state.actuator_hs_state->rpm;
      motor_state_msg.current =
          actuator_state.actuator_hs_state->current;
      motor_state_msg.pulse_count =
          actuator_state.actuator_hs_state->pulse_count;

      ranger_msgs::ActuatorState actuator_state_msg;
      actuator_state_msg.id = i;
      actuator_state_msg.driver = driver_state_msg;
      actuator_state_msg.motor = motor_state_msg;

      actuator_msg.states.push_back(actuator_state_msg);
    }

    actuator_state_pub_.publish(actuator_msg);
  }

  // publish BMS state
  {
    auto common_sensor_state = robot_->GetCommonSensorState();

    ranger_msgs::BatteryState batt_msg;
    batt_msg.header.stamp = current_time_;
    batt_msg.voltage = common_sensor_state.bms_basic_state.voltage;
    batt_msg.temperature = common_sensor_state.bms_basic_state.temperature;
    batt_msg.current = common_sensor_state.bms_basic_state.current;
    batt_msg.percentage = common_sensor_state.bms_basic_state.battery_soc;
    batt_msg.charge = std::numeric_limits<float>::quiet_NaN();
    batt_msg.capacity = std::numeric_limits<float>::quiet_NaN();
    batt_msg.design_capacity = std::numeric_limits<float>::quiet_NaN();
    batt_msg.power_supply_status =
        ranger_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    batt_msg.power_supply_health =
        ranger_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    batt_msg.power_supply_technology =
        ranger_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    batt_msg.present = std::numeric_limits<uint8_t>::quiet_NaN();

    battery_state_pub_.publish(batt_msg);
  }
}

void RangerROSMessenger::UpdateOdometry(double linear, double angular,
                                        double steering_angle, double dt) {
  // update odometry calculations
  if (motion_mode_ == MotionState::MOTION_MODE_DUAL_ACKERMAN) {
    DualAckermanModel::state_type x = {position_x_, position_y_, theta_};
    DualAckermanModel::control_type u;
    u.v = linear;
    u.phi = steering_angle;

    boost::numeric::odeint::integrate_const(
        boost::numeric::odeint::runge_kutta4<DualAckermanModel::state_type>(),
        DualAckermanModel(robot_params_.wheelbase, robot_params_.track, u), x,
        0.0, dt, (dt / 10.0));

    position_x_ = x[0];
    position_y_ = x[1];
    theta_ = x[2];
  } else if (motion_mode_ == MotionState::MOTION_MODE_PARALLEL ||
             motion_mode_ == MotionState::MOTION_MODE_SIDE_SLIP) {
    ParallelModel::state_type x = {position_x_, position_y_, theta_};
    ParallelModel::control_type u;
    u.v = linear;
    if (motion_mode_ == MotionState::MOTION_MODE_SIDE_SLIP) {
      u.phi = M_PI / 2.0;
    } else {
      u.phi = steering_angle;
    }
    boost::numeric::odeint::integrate_const(
        boost::numeric::odeint::runge_kutta4<ParallelModel::state_type>(),
        ParallelModel(u), x, 0.0, dt, (dt / 10.0));

    position_x_ = x[0];
    position_y_ = x[1];
    theta_ = x[2];
  } else if (motion_mode_ == MotionState::MOTION_MODE_SPINNING) {
    SpinningModel::state_type x = {position_x_, position_y_, theta_};
    SpinningModel::control_type u;
    u.w = angular;

    boost::numeric::odeint::integrate_const(
        boost::numeric::odeint::runge_kutta4<SpinningModel::state_type>(),
        SpinningModel(u), x, 0.0, dt, (dt / 10.0));

    position_x_ = x[0];
    position_y_ = x[1];
    theta_ = x[2];
  }

  // update odometry topics
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

  // ROS_INFO("Pose: %f, %f, %f", position_x_, position_y_, theta_ / 3.14 *
  // 180.0);

  // publish odometry and tf messages
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = current_time_;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  odom_msg.pose.pose.position.x = position_x_;
  odom_msg.pose.pose.position.y = position_y_;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;

  if (motion_mode_ == MotionState::MOTION_MODE_DUAL_ACKERMAN) {
    odom_msg.twist.twist.linear.x = linear;
    odom_msg.twist.twist.linear.y = 0.0;
    if (steering_angle == 0) {
      odom_msg.twist.twist.angular.z = 0;
    } else {
      odom_msg.twist.twist.angular.z =
          (steering_angle / std::abs(steering_angle)) * 2 * linear /
          (robot_params_.wheelbase / std::abs(std::tan(steering_angle)) +
           robot_params_.track);
    }
  } else if (motion_mode_ == MotionState::MOTION_MODE_PARALLEL ||
             motion_mode_ == MotionState::MOTION_MODE_SIDE_SLIP) {
    double phi = steering_angle;

    if (motion_mode_ == MotionState::MOTION_MODE_SIDE_SLIP) {
      phi = M_PI / 2.0;
    }
    odom_msg.twist.twist.linear.x = linear * std::cos(phi);
    odom_msg.twist.twist.linear.y = linear * std::sin(phi);

    odom_msg.twist.twist.angular.z = 0;
  } else if (motion_mode_ == MotionState::MOTION_MODE_SPINNING) {
    odom_msg.twist.twist.linear.x = 0;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.angular.z = angular;
  }

  odom_pub_.publish(odom_msg);

  // // publish tf transformation
  if (publish_odom_tf_) {
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = position_x_;
    tf_msg.transform.translation.y = position_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;

    tf_broadcaster_.sendTransform(tf_msg);
  }
}

void RangerROSMessenger::TwistCmdCallback(
    const geometry_msgs::Twist::ConstPtr& msg) {
  double steer_cmd;
  double radius;

  // analyze Twist msg and switch motion_mode
  // check for parking mode, only applicable to RangerMiniV2
  if (parking_mode_ && robot_type_ == RangerSubType::kRangerMiniV2) {
    return;
  } else if (msg->linear.y != 0) {
    if (msg->linear.x == 0.0 && robot_type_ == RangerSubType::kRangerMiniV1) {
      motion_mode_ = MotionState::MOTION_MODE_SIDE_SLIP;
      robot_->SetMotionMode(MotionState::MOTION_MODE_SIDE_SLIP);
    } else {
      motion_mode_ = MotionState::MOTION_MODE_PARALLEL;
      robot_->SetMotionMode(MotionState::MOTION_MODE_PARALLEL);
    }
  } else {
    steer_cmd = CalculateSteeringAngle(*msg, radius);
    // Use minimum turn radius to switch between dual ackerman and spinning mode
    if (radius < robot_params_.min_turn_radius) {
      motion_mode_ = MotionState::MOTION_MODE_SPINNING;
      robot_->SetMotionMode(MotionState::MOTION_MODE_SPINNING);
    } else {
      motion_mode_ = MotionState::MOTION_MODE_DUAL_ACKERMAN;
      robot_->SetMotionMode(MotionState::MOTION_MODE_DUAL_ACKERMAN);
    }
  }

  // send motion command to robot
  switch (motion_mode_) {
    case MotionState::MOTION_MODE_DUAL_ACKERMAN: {
      if (steer_cmd > robot_params_.max_steer_angle_ackermann) {
        steer_cmd = robot_params_.max_steer_angle_ackermann;
      }
      if (steer_cmd < -robot_params_.max_steer_angle_ackermann) {
        steer_cmd = -robot_params_.max_steer_angle_ackermann;
      }
      robot_->SetMotionCommand(msg->linear.x, steer_cmd);
      break;
    }
    case MotionState::MOTION_MODE_PARALLEL: {
      steer_cmd = atan(msg->linear.y / msg->linear.x);
      if (steer_cmd > robot_params_.max_steer_angle_parallel) {
        steer_cmd = robot_params_.max_steer_angle_parallel;
      }
      if (steer_cmd < -robot_params_.max_steer_angle_parallel) {
        steer_cmd = -robot_params_.max_steer_angle_parallel;
      }
      double vel = msg->linear.x >= 0 ? 1.0 : -1.0;
      robot_->SetMotionCommand(vel * sqrt(msg->linear.x * msg->linear.x +
                                          msg->linear.y * msg->linear.y),
                               steer_cmd);
      break;
    }
    case MotionState::MOTION_MODE_SPINNING: {
      double a_v = msg->angular.z;
      if (a_v > robot_params_.max_angular_speed) {
        a_v = robot_params_.max_angular_speed;
      }
      if (a_v < -robot_params_.max_angular_speed) {
        a_v = -robot_params_.max_angular_speed;
      }
      robot_->SetMotionCommand(0.0, 0.0, a_v);
      break;
    }
    case MotionState::MOTION_MODE_SIDE_SLIP: {
      double l_v = msg->linear.y;
      if (l_v > robot_params_.max_linear_speed) {
        l_v = robot_params_.max_linear_speed;
      }
      if (l_v < -robot_params_.max_linear_speed) {
        l_v = -robot_params_.max_linear_speed;
      }
      robot_->SetMotionCommand(0.0, 0.0, l_v);
      break;
    }
  }
}

void RangerROSMessenger::LightCmdCallback(
    const ranger_msgs::RangerLightCmd::ConstPtr &msg)
{
    if (msg->enable_cmd_light_control)
    {
      LightCommandMessage cmd;

      switch (msg->front_mode)
      {
        case ranger_msgs::RangerLightCmd::LIGHT_CONST_OFF:
        {
          cmd.front_light.mode = CONST_OFF;
          break;
        }
        case ranger_msgs::RangerLightCmd::LIGHT_CONST_ON:
        {
          cmd.front_light.mode = CONST_ON;
          break;
        }
      }
      robot_->SetLightCommand(cmd.front_light.mode,0,cmd.front_light.mode,0);
    }
    else
    {
      robot_->DisableLightControl();
    }
}

double RangerROSMessenger::CalculateSteeringAngle(geometry_msgs::Twist msg,
                                                  double& radius) {
  double linear = std::abs(msg.linear.x);
  double angular = std::abs(msg.angular.z);

  // Circular motion
  radius = linear / angular;
  int k = (msg.angular.z * msg.linear.x) >= 0 ? 1.0 : -1.0;

  double l, w, phi_i;
  l = robot_params_.wheelbase;
  w = robot_params_.track;
  phi_i = atan((l / 2) / (radius - w / 2));
  ROS_INFO("command linear: %f, steering_angle: %f", linear, k * phi_i);
  return k * phi_i;
}

bool RangerROSMessenger::TriggerParkingService(
    ranger_msgs::TriggerParkMode::Request& req,
    ranger_msgs::TriggerParkMode::Response& res) {
  // Call to trigger park mode
  if (req.TriggerParkedMode) {
    res.isParked = true;
    robot_->SetMotionCommand(0.0,
                             0.0);  // This functions needs to be invoked before
                                    // the parking mode can be triggered
    robot_->SetMotionMode(MotionState::MOTION_MODE_PARKING);
  } else {  // Call to release park mode
    res.isParked = false;
    robot_->SetMotionMode(MotionState::MOTION_MODE_DUAL_ACKERMAN);
    robot_->SetMotionCommand(
        0.0, 0.0);  // Setting the mode to dual Ackerman doesn't return the
                    // wheels to its original position, hence this function.
  }
  parking_mode_ = res.isParked;
  return true;
}
}  // namespace westonrobot
