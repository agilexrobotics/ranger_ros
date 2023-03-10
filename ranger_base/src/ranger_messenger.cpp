/**
 * @Kit       : Qt-Creator: Desktop
 * @Author    : Wang Zhe
 * @Date      : 2021-04-20  11:50:26
 * @FileName  : ranger_messenger.cpp
 * @Mail      : zhe.wang@agilex.ai
 * Copyright  : AgileX Robotics (2021)
 **/

#include "ranger_base/ranger_messenger.hpp"
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "ranger_base/ranger_params.hpp"
#include "ranger_msgs/RangerStatus.h"
#include "ranger_msgs/RangerBmsStatus.h"
#include "ranger_msgs/RobotStatus.h"
#include <tinyxml2.h>
#include <boost/algorithm/string.hpp>



using namespace ros;
using namespace ranger_msgs;
using namespace std;
using namespace boost::algorithm;

namespace westonrobot {
RangerROSMessenger::RangerROSMessenger(ros::NodeHandle *nh)
    : ranger_(nullptr), nh_(nh) {}

RangerROSMessenger::RangerROSMessenger(RangerBase *ranger, ros::NodeHandle *nh)
    : ranger_(ranger), nh_(nh) {}

void RangerROSMessenger::SetupSubscription() {

  std::string robot_type;
  ros::NodeHandle private_nh("~");
  private_nh.param<string>("robot_type",robot_type,"ranger");
  if(robot_type == "ranger")
  {
    ROS_INFO("use ranger param");
    robot_params.track = RangerParams::track;
    robot_params.wheelbase = RangerParams::wheelbase;
    robot_params.max_speed_cmd = RangerParams::max_speed_cmd;
    robot_params.max_round_angle = RangerParams::max_round_angle;
    robot_params.min_turn_radius = RangerParams::min_turn_radius;
    robot_params.max_angular_speed = RangerParams::max_angular_speed;
    robot_params.max_steer_angle_slide = RangerParams::max_steer_angle_slide;
    robot_params.max_steer_angle_central = RangerParams::max_steer_angle_central;


  }
  else if(robot_type == "ranger-mini")
  {
    ROS_INFO("use ranger-mini param");
    robot_params.track = RangerMiniParams::track;
    robot_params.wheelbase = RangerMiniParams::wheelbase;
    robot_params.max_speed_cmd = RangerMiniParams::max_speed_cmd;
    robot_params.max_round_angle = RangerMiniParams::max_round_angle;
    robot_params.min_turn_radius = RangerMiniParams::min_turn_radius;
    robot_params.max_angular_speed = RangerMiniParams::max_angular_speed;
    robot_params.max_steer_angle_slide = RangerMiniParams::max_steer_angle_slide;
    robot_params.max_steer_angle_central = RangerMiniParams::max_steer_angle_central;
  }
  odom_publisher_ = nh_->advertise<nav_msgs::Odometry>(odom_topic_name_, 50);
  status_publisher_ =
      nh_->advertise<ranger_msgs::RangerStatus>("/ranger_status", 10);
  robot_status_publisher_ = nh_->advertise<ranger_msgs::RobotStatus>("/robot_status",10);

  motion_cmd_subscriber_ = nh_->subscribe<geometry_msgs::Twist>(
      "/cmd_vel", 5, &RangerROSMessenger::TwistCmdCallback, this);
  ranger_setting_sub_ = nh_->subscribe<ranger_msgs::RangerSetting>(
      "/ranger_setting", 1, &RangerROSMessenger::RangerSettingCbk, this);


  curr_transform_ = Eigen::Matrix4d::Identity();
}



void RangerROSMessenger::PublishStateToROS() {
  current_time_ = ros::Time::now();
  double dt = (current_time_ - last_time_).toSec();
  static bool init_run = true;
  if (init_run) {
    last_time_ = current_time_;
    init_run = false;
    return;
  }

  auto state = ranger_->GetRobotState();
  auto motor_state = ranger_->GetMotorState();


  ranger_msgs::RangerStatus status_msg;
  ranger_msgs::RobotStatus robot_status;
  status_msg.header.stamp = current_time_;

  status_msg.vehicle_state = state.system_state.vehicle_state;
  status_msg.control_mode = state.system_state.control_mode;
  status_msg.error_code = state.system_state.error_code;
  status_msg.battery_voltage = state.system_state.battery_voltage;
  status_msg.current_motion_mode = state.current_motion_mode.motion_mode;

  motion_mode_ = status_msg.current_motion_mode;

  // linear_velocity, angular_velocity, central steering_angle
  double l_v = 0.0, a_v = 0.0, phi = 0.0,c_v=0.0;
  // x , y direction linear velocity, motion radius
  double x_v = 0.0, y_v = 0.0, radius = 0.0;

  double phi_i = state.motion_state.steering_angle / 180.0 * M_PI;
  ROS_DEBUG_NAMED("feedback","phi_i:%f steering_angle:%f wheel_cpeed:%f\n",phi_i, -state.motion_state.steering_angle,state.motion_state.linear_velocity);
  switch (motion_mode_) {
    case RangerSetting::MOTION_MODE_ACKERMAN: {
    double i_v;
    if(fabs(phi_i) < steer_angle_tolerance)
    {
      l_v = state.motion_state.linear_velocity ;

      a_v = 0.0;
      y_v = 0.0;
      radius = 0.0;
    }
    else if(phi_i > 0)
    {
      i_v= (motor_state.motor_speed_state.speed_3 + motor_state.motor_speed_state.speed_4)/2.0;

      phi_i = (fabs(motor_state.motor_angle_state.angle_7) + fabs(motor_state.motor_angle_state.angle_8))/2;
      if(phi_i == 0)
      {
        return;
      }
      phi_i = phi_i/180.0*M_PI;
      double c,r_c,v_c,l,w,k;
      if(i_v != 0)
        k = i_v / fabs(i_v);
      else
        k = 0;
      l = robot_params.wheelbase;
      w =  robot_params.track;
      a_v = 2*fabs(i_v)*sin(phi_i)/l;
      c = atan(l*tan(phi_i)/(l-w*tan(phi_i)));
      r_c = l/(2*sin(c));
      v_c = fabs(a_v * r_c) * k;

      l_v = v_c;
      radius = r_c;
      a_v = -a_v;
    }
    else
    {
      ROS_DEBUG_NAMED("aaa","speed_1:%f speed_2:%f",motor_state.motor_speed_state.speed_1,motor_state.motor_speed_state.speed_2);
      i_v= (motor_state.motor_speed_state.speed_1 + motor_state.motor_speed_state.speed_2)/2.0;
      phi_i = (fabs(motor_state.motor_angle_state.angle_5) + fabs(motor_state.motor_angle_state.angle_6))/2;
      phi_i = phi_i/180.0*M_PI;
      double c,r_c,v_c,l,w,k;
      if(i_v != 0)
        k = i_v / fabs(i_v);
      else
        k = 0;
      l = robot_params.wheelbase;
      w =  robot_params.track;
      a_v = 2*fabs(i_v)*sin(phi_i)/l;//角速度
//      c = atan(l*tan(phi_i)/(w*tan(phi_i)+l));
      c = atan(l*tan(phi_i)/(l-w*tan(phi_i)));//中心角
      r_c = l/(2*sin(c));//中心半径
      v_c = fabs(a_v * r_c) * k;


      l_v = v_c;
      radius = r_c;
    }
    x_v = l_v;
    break;
    }
    case RangerSetting::MOTION_MODE_SLIDE: {
      l_v = state.motion_state.linear_velocity;
      phi = phi_i;
      a_v = 0.0;
      x_v = l_v * std::cos(phi);
      y_v = l_v * std::sin(phi);
      radius = 0.0;
      break;
    }
    case RangerSetting::MOTION_MODE_ROUND: {
      double k;
      l_v = 0.0;
      phi = std::fabs(phi_i);
      if(state.motion_state.linear_velocity != 0)
        k = state.motion_state.linear_velocity / state.motion_state.linear_velocity;
      else
        k = 0;
      a_v = state.motion_state.linear_velocity / (robot_params.track/(2 * cos(robot_params.max_round_angle)));
      x_v = 0.0;
      y_v = 0.0;
      radius = a_v / state.motion_state.linear_velocity;
      break;
    }
    case RangerSetting::MOTION_MODE_SLOPING: {
      l_v = -state.motion_state.linear_velocity;
      phi = std::fabs(phi_i);
      a_v = 0.0;
      x_v = 0.0;
      y_v = l_v;
      radius = 0.0;
      break;
    }
  }


  status_msg.linear_velocity = l_v;
  status_msg.angular_velocity = a_v;
  status_msg.lateral_velocity = 0.0;
  status_msg.steering_angle = -phi;
  status_msg.x_linear_vel = x_v;
  status_msg.y_linear_vel = y_v;
  status_msg.motion_radius = radius;

  status_publisher_.publish(status_msg);

  if(isnan(l_v) ||
     isnan(x_v))
  {
    ROS_WARN("motion_mode:%d",motion_mode_);
  }

  PublishOdometryToROS(l_v, a_v, x_v, y_v, dt);


  robot_status.base_state = status_msg.vehicle_state;
  robot_status.control_mode = status_msg.control_mode;
  robot_status.fault_code = status_msg.error_code;
  robot_status.linear_velocity = status_msg.linear_velocity;
  robot_status.angular_velocity = status_msg.angular_velocity;
  robot_status_publisher_.publish(robot_status);

  last_time_ = current_time_;
}

void RangerROSMessenger::PublishSimStateToROS(double linear, double angular) {}

void RangerROSMessenger::GetCurrentMotionCmdForSim(double &linear,
                                                   double &angular) {
  std::lock_guard<std::mutex> lg(twist_mutex_);
  linear = current_twist_.linear.x;
  angular = current_twist_.angular.z;
}

void RangerROSMessenger::TwistCmdCallback( const geometry_msgs::Twist::ConstPtr &msg) {
  double steer_cmd = msg->angular.z;

  if (simulated_robot_) {
    std::lock_guard<std::mutex> lg(twist_mutex_);
    current_twist_ = *msg.get();
    return;
  }

  double radius;
  steer_cmd = AngelVelocity2Angel(*msg,radius);
  if(radius < robot_params.min_turn_radius)
  {
    motion_mode_ = RangerSetting::MOTION_MODE_ROUND;
    ranger_->SetMotionMode(RangerSetting::MOTION_MODE_ROUND);
  }
  else
  {
    motion_mode_ = RangerSetting::MOTION_MODE_ACKERMAN;
    ranger_->SetMotionMode(RangerSetting::MOTION_MODE_ACKERMAN);
  }
  switch (motion_mode_) {
    case RangerSetting::MOTION_MODE_ACKERMAN: {
      double radius;
      steer_cmd = AngelVelocity2Angel(*msg,radius);
      if (steer_cmd > robot_params.max_steer_angle_central) {
        steer_cmd = robot_params.max_steer_angle_central;
      }
      if (steer_cmd < -robot_params.max_steer_angle_central) {
        steer_cmd = -robot_params.max_steer_angle_central;
      }

      double phi_i = steer_cmd;
      ROS_DEBUG_NAMED("send","phi_i:%f",phi_i);
      double phi_degree = -(phi_i / M_PI * 180.0);
      ROS_DEBUG_NAMED("ranger","l_x:%f a_z:%f",msg->linear.x,msg->angular.z);
      ranger_->SetMotionCommand(msg->linear.x , phi_degree);
      break;
    }
    case RangerSetting::MOTION_MODE_SLIDE: {
      if (steer_cmd > robot_params.max_steer_angle_slide) {
        steer_cmd = robot_params.max_steer_angle_slide;
      }
      if (steer_cmd < -robot_params.max_steer_angle_slide) {
        steer_cmd = -robot_params.max_steer_angle_slide;
      }

      //double phi_degree = -(steer_cmd / M_PI * 180.0);
      double phi_degree = steer_cmd / M_PI * 180.0;
      ranger_->SetMotionCommand(msg->linear.x, phi_degree);
      break;
    }
    case RangerSetting::MOTION_MODE_ROUND:
    case RangerSetting::MOTION_MODE_SLOPING: {

      float l_v , w,a_v;
      a_v = msg->angular.z;
      w = robot_params.track;
      l_v = w * a_v /(2*cos(robot_params.max_round_angle));
      ROS_INFO("%f",l_v);
      ranger_->SetMotionCommand(0.0, 0.0, l_v /*/ 1.68*/, 0.0);
      break;
    }
  }
//  ranger_->SetMotionCommand(msg->linear.x, steer_cmd);
}

void RangerROSMessenger::RangerSettingCbk(
    const ranger_msgs::RangerSetting::ConstPtr &msg) {
  ROS_DEBUG_NAMED("variance","ddd");
  auto mode = msg->motion_mode;
  motion_mode_ = mode;
  switch (mode) {
    case RangerSetting::MOTION_MODE_ACKERMAN: {
      ranger_->SetMotionMode(RangerSetting::MOTION_MODE_ACKERMAN);
      break;
    }
    case RangerSetting::MOTION_MODE_SLIDE: {
      ranger_->SetMotionMode(RangerSetting::MOTION_MODE_SLIDE);
      break;
    }
    case RangerSetting::MOTION_MODE_ROUND: {
      ranger_->SetMotionMode(RangerSetting::MOTION_MODE_ROUND);
      break;
    }
    case RangerSetting::MOTION_MODE_SLOPING: {
      ranger_->SetMotionMode(RangerSetting::MOTION_MODE_SLOPING);
      break;
    }
    default:
      ROS_WARN("ranger motion mode not support %d", mode);
      break;
  }
}
double RangerROSMessenger::ConvertInnerAngleToCentral(double angle) {
  double phi = 0;
  double phi_i = angle;
  if (phi_i > steer_angle_tolerance) {
    // left turn
    double r = l / std::tan(phi_i) + w;
    phi = std::atan(l / r);
  } else if (phi_i < -steer_angle_tolerance) {
    // right turn
    double r = l / std::tan(-phi_i) + w;
    phi = std::atan(l / r);
    phi = -phi;
  }
    ROS_DEBUG_NAMED("trans","Central:%f",phi);
  return phi;
}

double RangerROSMessenger::ConvertCentralAngleToInner(double angle) {
  double phi = angle;
  double phi_i = 0;
  if (phi > steer_angle_tolerance) {
    // left turn
    phi_i =
        std::atan(l * std::sin(phi) / (l * std::cos(phi) - w * std::sin(phi)));
  } else if (phi < -steer_angle_tolerance) {
    // right turn
    phi = -phi;
    phi_i =
        std::atan(l * std::sin(phi) / (l * std::cos(phi) - w * std::sin(phi)));
    phi_i = -phi_i;
  }
    ROS_DEBUG_NAMED("trans","Inner:%f",phi_i);
  return phi_i;
}
void RangerROSMessenger::PublishOdometryToROS(double linear, double angle_vel,
                                              double x_linear_vel,
                                              double y_linear_vel, double dt) {
  linear_speed_ = linear;
  angular_vel_ = angle_vel;
  x_linear_vel_ = x_linear_vel;
  y_linear_vel_ = y_linear_vel;
  double theta = angular_vel_ * dt;

  // update
  position_x_ +=
      cos(theta_) * x_linear_vel_ * dt - sin(theta_) * y_linear_vel_ * dt;
  position_y_ +=
      sin(theta_) * x_linear_vel_ * dt + cos(theta_) * y_linear_vel_ * dt;
  theta_ = theta_ + angular_vel_ * dt;


  if(isnan(position_x_) ||
     isnan(position_y_))
  {
    return;
  }

  if(isnan(position_x_) || isnan(position_y_))
  {
    ROS_INFO("%f,%f,%f,%f",linear,angle_vel,x_linear_vel,y_linear_vel);
  }
  if (theta_ > M_PI) {
    theta_ -= 2 * M_PI;
  } else if (theta_ < -M_PI) {
    theta_ += 2 * M_PI;
  }
  //  printf("theta_ %f,  theta cos angle: %f\n", curr_transform_(0, 0));

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

  if (pub_odom_tf_) {
    // publish tf transformation
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
  // publish odometry and tf messages
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = current_time_;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  odom_msg.pose.pose.position.x = position_x_;
  odom_msg.pose.pose.position.y = position_y_;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;

  odom_msg.twist.twist.linear.x = x_linear_vel_;
  odom_msg.twist.twist.linear.y = y_linear_vel_;
  odom_msg.twist.twist.angular.z = angular_vel_;

  //  std::cerr << "linear: " << linear_speed_ << " , angular vel: " <<
  //  angular_vel_
  //            << " , pose: (" << position_x_ << "," << position_y_ << ","
  //            << theta_ << ")" << std::endl;

  odom_publisher_.publish(odom_msg);
}

double RangerROSMessenger::AngelVelocity2Angel(geometry_msgs::Twist msg,double &radius)
{
  double linear = fabs(msg.linear.x);
  double angular = fabs(msg.angular.z);
  if(angular == 0)
  {
    radius = robot_params.min_turn_radius;
    return 0.0;
  }

  radius = linear / angular;
  int k = msg.angular.z / fabs(msg.angular.z);
  if ((2*radius-robot_params.track)<0 )
  {
    return  k*robot_params.max_steer_angle_central;
  }

  double l,w,phi_i,x;
  l = robot_params.wheelbase;
  w = robot_params.track;
  x = sqrt(radius*radius + (l/2)*(l/2));
  phi_i = atan((l/2)/(x-w/2));
  return k*phi_i;
  /******/
//  double phi_c,phi_i;
//  phi_c = asin(RangerParams::wheelbase*angular/(2*linear));
//  phi_i = atan(RangerParams::wheelbase*angular/(2*linear*cos(phi_c)-RangerParams::track*angular));
//  return k*phi_i;
//  ROS_DEBUG_NAMED("new_formula_send","phi_i:%f\n",phi_i);
  /******/

//  return k*std::atan(RangerParams::wheelbase / (2*radius-RangerParams::track));
}

double RangerROSMessenger::Deg2Rad(double x)
{
  return x*M_PI/180.0;
}

}  // namespace westonrobot


