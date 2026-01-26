#pragma once
#include <memory>
#include <mutex>
#include <atomic>
#include <string>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <boost/numeric/odeint.hpp>

// 运动模式枚举
enum class MotionMode
{
    DUAL_ACKERMAN = 0, // 双阿克曼转向模式
    PARALLEL = 1,      // 平行移动模式
    SPINNING = 2,      // 自旋模式
    PARKING = 3,       // 停车模式
    SIDE_SLIP = 4      // 侧滑模式
};

// 机器人参数结构体
struct RobotParameters
{
    double track = 0.34;                   // 轮距 (m)
    double wheelbase = 0.39;               // 轴距 (m)
    double max_linear_speed = 0.0;         // 最大线速度 (m/s)
    double max_angular_speed = 0.0;        // 最大角速度 (rad/s)
    double max_speed_cmd = 0.0;            // 最大速度命令
    double max_steer_angle_central = 0.0;  // 最大中央转向角 (rad)
    double max_steer_angle_parallel = 0.0; // 最大平行转向角 (rad)
    double max_round_angle = 0.0;          // 最大旋转角度
    double min_turn_radius = 0.0;          // 最小转弯半径
    double spin_angular_ratio = 1.0;       // 自旋角速度比例系数
    double linear_speed_ratio = 1.0;       // 线速度比例系数
};

// 运动学模型类
class DualAckermanModel
{
public:
    using state_type = std::vector<double>;

    struct control_type
    {
        double v;
        double phi;
    };

    DualAckermanModel(double L, control_type u) : L_(L), u_(u) {};

    void operator()(const state_type &x, state_type &xd, double)
    {
        xd[0] = u_.v * std::cos(u_.phi) * std::cos(x[2]);
        xd[1] = u_.v * std::cos(u_.phi) * std::sin(x[2]);
        xd[2] = 2 * u_.v * std::sin(u_.phi) / L_;
    }

private:
    double L_;
    control_type u_;
};

class ParallelModel
{
public:
    using state_type = std::vector<double>;

    struct control_type
    {
        double v;
        double phi;
    };

    ParallelModel(control_type u) : u_(u) {};

    void operator()(const state_type &x, state_type &xd, double)
    {
        xd[0] = u_.v * std::cos(x[2] + u_.phi);
        xd[1] = u_.v * std::sin(x[2] + u_.phi);
        xd[2] = 0;
    }

private:
    control_type u_;
};

class SpinningModel
{
public:
    using state_type = std::vector<double>;

    struct control_type
    {
        double w;
    };

    SpinningModel(control_type u) : u_(u) {};

    void operator()(const state_type &x, state_type &xd, double)
    {
        xd[0] = 0;
        xd[1] = 0;
        xd[2] = u_.w;
    }

private:
    control_type u_;
};

/**
 * @class GlobalState
 * @brief 线程安全的全局状态管理器单例
 */
class GlobalState
{
public:
    // 删除拷贝构造函数和赋值运算符
    GlobalState(const GlobalState &) = delete;
    GlobalState &operator=(const GlobalState &) = delete;

    // 获取全局唯一实例（线程安全）
    static GlobalState &getInstance()
    {
        static GlobalState instance;
        return instance;
    }

    // ===== 初始化方法 =====
    void initialize(ros::NodeHandle &nh)
    {
        std::lock_guard<std::mutex> lock(init_mutex_);
        if (!initialized_)
        {
            nh_ = nh;
            loadParameters(nh);
            odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/wheel/odom", 10);
            mode_pub_ = nh_.advertise<std_msgs::Int32>("current_motion_mode", 10);
            distance_pub_ = nh_.advertise<std_msgs::Float32>("total_distance", 10);

            // 初始化时间戳
            last_update_ = ros::Time::now();

            // 设置默认坐标系
            map_frame_ = "map";
            odom_frame_ = "odom";
            base_frame_ = "base_link";

            // 创建定时器进行更新
            timer_ = nh_.createTimer(ros::Duration(0.02),
                                     &GlobalState::update, this);

            // 重置里程计位置
            resetPose();

            initialized_ = true;

            // 记录初始化日志
            ROS_INFO("里程计系统初始化完成");
            ROS_INFO("TF框架: %s -> %s",
                    odom_frame_.c_str(), base_frame_.c_str());
        }
    }

    void loadParameters(ros::NodeHandle &nh)
    {
        // 机器人物理参数
        nh.param<double>("/agx_bringup_node/track", params_.track, 0.34);
        nh.param<double>("/agx_bringup_node/wheelbase", params_.wheelbase, 0.39);

        // 性能参数
        nh.param<double>("/agx_bringup_node/max_linear_speed", params_.max_linear_speed, 1.0);
        nh.param<double>("/agx_bringup_node/max_angular_speed", params_.max_angular_speed, 1.0);
        nh.param<double>("/agx_bringup_node/spin_angular_ratio", params_.spin_angular_ratio, 1.0);
        nh.param<double>("/agx_bringup_node/linear_speed_ratio", params_.linear_speed_ratio, 1.0);
        
        // 坐标系参数
        nh.param<std::string>("/agx_bringup_node/map_frame", map_frame_, "map");
        nh.param<std::string>("/agx_bringup_node/odom_frame", odom_frame_, "odom");
        nh.param<std::string>("/agx_bringup_node/base_frame", base_frame_, "base_link");
    }

    // 重置机器人位姿
    void resetPose(double x = 0.0, double y = 0.0, double theta = 0.0)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        position_x_ = x;
        position_y_ = y;
        theta_ = theta;
    }

    // ===== 状态设置接口 =====
    void setLinearVelocity(double velocity)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        linear_velocity_ = velocity * params_.linear_speed_ratio;
    }

    void setAngularVelocity(double velocity)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        angular_velocity_ = velocity;
    }

    void setSteeringAngle(double angle)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        steering_angle_ = angle;
    }

    void setMotionMode(MotionMode mode)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        motion_mode_ = mode;
    }

    void setParameters(const RobotParameters &params)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        params_ = params;
    }

    void setMapFrame(const std::string &frame)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        map_frame_ = frame;
    }

    void setOdomFrame(const std::string &frame)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        odom_frame_ = frame;
    }

    void setBaseFrame(const std::string &frame)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        base_frame_ = frame;
    }

    // ===== 状态获取接口 =====
    double getLinearVelocity() const
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return linear_velocity_;
    }

    double getAngularVelocity() const
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return angular_velocity_;
    }

    // 获取当前实际角速度
    double getCurrentAngularVelocity() const
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return current_angular_velocity_;
    }

    double getSteeringAngle() const
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return steering_angle_;
    }

    double getSteeringAngleDeg() const
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return steering_angle_ * 180.0 / M_PI;
    }

    MotionMode getMotionMode() const
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return motion_mode_;
    }

    std::string getMotionModeString() const
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        switch (motion_mode_)
        {
        case MotionMode::DUAL_ACKERMAN:
            return "双阿克曼模式";
        case MotionMode::PARALLEL:
            return "平行模式";
        case MotionMode::SPINNING:
            return "自旋模式";
        case MotionMode::SIDE_SLIP:
            return "侧滑模式";
        case MotionMode::PARKING:
            return "停车模式";
        default:
            return "未知模式";
        }
    }

    // 获取机器人相对于odom坐标系的位置
    void getPosition(double &x, double &y, double &theta) const
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        x = position_x_;
        y = position_y_;
        theta = theta_;
    }

    RobotParameters getParameters() const
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return params_;
    }

    ros::NodeHandle getNodeHandle() const
    {
        return nh_;
    }

private:
    // 私有构造函数
    GlobalState() : last_update_(ros::Time(0)) {}

    // 更新里程计
    void update(const ros::TimerEvent &event)
    {
        auto current_time = ros::Time::now();

        // 第一次调用时初始化时间
        if (last_update_.toSec() == 0)
        {
            last_update_ = current_time;
            return;
        }

        double dt = (current_time - last_update_).toSec();
        last_update_ = current_time;

        // 防止时间间隔异常
        if (dt <= 0 || dt > 0.5)
        {
            ROS_WARN("异常时间间隔: %.4fs, 使用默认值", dt);
            dt = 0.02; // 默认20ms
        }

        updateOdometry(dt);
        publishOdometry(current_time);
        publishTotalDistance();
        publishMotionMode();
    }

    // 更新里程计内部方法 - 使用运动学模型
    void updateOdometry(double dt)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        double linear = linear_velocity_;
        double angular = angular_velocity_;
        double steer = steering_angle_;

        double prev_x = position_x_;
        double prev_y = position_y_;
        double prev_theta = theta_;

        // 使用运动学模型更新位置
        switch (motion_mode_)
        {
        case MotionMode::DUAL_ACKERMAN:
        {
            DualAckermanModel::state_type x = {position_x_, position_y_, theta_};
            DualAckermanModel::control_type u;
            u.v = linear;
            u.phi = ConvertInnerAngleToCentral(steer);

            double central_angle = ConvertInnerAngleToCentral(steer);
            current_angular_velocity_ = 2 * linear * std::sin(central_angle) / params_.wheelbase;

            boost::numeric::odeint::integrate_const(
                boost::numeric::odeint::runge_kutta4<DualAckermanModel::state_type>(),
                DualAckermanModel(params_.wheelbase, u), x, 0.0, dt, (dt / 10.0));

            position_x_ = x[0];
            position_y_ = x[1];
            theta_ = x[2];
            break;
        }

        case MotionMode::PARALLEL:
        {
            ParallelModel::state_type x = {position_x_, position_y_, theta_};
            ParallelModel::control_type u;
            u.v = linear;
            u.phi = steer;
            current_angular_velocity_ = 0.0;
            boost::numeric::odeint::integrate_const(
                boost::numeric::odeint::runge_kutta4<ParallelModel::state_type>(),
                ParallelModel(u), x, 0.0, dt, (dt / 10.0));

            position_x_ = x[0];
            position_y_ = x[1];
            theta_ = x[2];
            break;
        }

        case MotionMode::SPINNING:
        {
            SpinningModel::state_type x = {position_x_, position_y_, theta_};
            SpinningModel::control_type u;
            u.w = angular * params_.spin_angular_ratio;
            current_angular_velocity_ = angular * params_.spin_angular_ratio;
            boost::numeric::odeint::integrate_const(
                boost::numeric::odeint::runge_kutta4<SpinningModel::state_type>(),
                SpinningModel(u), x, 0.0, dt, (dt / 10.0));

            position_x_ = x[0];
            position_y_ = x[1];
            theta_ = x[2];
            break;
        }

        case MotionMode::SIDE_SLIP:
        {
            ParallelModel::state_type x = {position_x_, position_y_, theta_};
            ParallelModel::control_type u;
            u.v = linear;
            u.phi = M_PI / 2.0; // 固定90度侧滑
            current_angular_velocity_ = 0.0;
            boost::numeric::odeint::integrate_const(
                boost::numeric::odeint::runge_kutta4<ParallelModel::state_type>(),
                ParallelModel(u), x, 0.0, dt, (dt / 10.0));

            position_x_ = x[0];
            position_y_ = x[1];
            theta_ = x[2];
            break;
        }

        case MotionMode::PARKING:
            // 停车模式，位置不变
            current_angular_velocity_ = 0.0;
            break;
        }

        // 计算移动距离并累加到总里程
        double dx = position_x_ - prev_x;
        double dy = position_y_ - prev_y;
        double step_distance = std::sqrt(dx * dx + dy * dy);
        total_distance_ += step_distance;

        // 规范化角度到 [-π, π]
        theta_ = fmod(theta_ + M_PI, 2 * M_PI);
        if (theta_ < 0)
            theta_ += 2 * M_PI;
        theta_ -= M_PI;
    }

    // 发布TF和里程计信息
    void publishOdometry(const ros::Time &current_time)
    {
        double x, y, theta;
        double linear, angular;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            x = position_x_;
            y = position_y_;
            theta = theta_;
            linear = linear_velocity_;
            angular = current_angular_velocity_;
        }

        // 发布TF变换
        // publishTfTransforms(current_time, x, y, theta);

        // 发布里程计消息
        publishOdomMessage(current_time, x, y, theta, linear, angular);
    }

    // 发布TF变换 (三层坐标系链)
    void publishTfTransforms(const ros::Time &time, double x, double y, double theta)
    {
        static tf::TransformBroadcaster tf_broadcaster;

        // map -> odom (静态恒等变换)
        tf::Transform map_to_odom;
        map_to_odom.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        tf::Quaternion q_map_odom;
        q_map_odom.setRPY(0, 0, 0);
        map_to_odom.setRotation(q_map_odom);

        // odom -> base_link (机器人位姿变换)
        tf::Transform odom_to_base;
        odom_to_base.setOrigin(tf::Vector3(x, y, 0.0));
        tf::Quaternion q_base;
        q_base.setRPY(0, 0, theta);
        odom_to_base.setRotation(q_base);

        // 发布变换
        // tf_broadcaster.sendTransform(tf::StampedTransform(map_to_odom, time, map_frame_, odom_frame_));
        tf_broadcaster.sendTransform(tf::StampedTransform(odom_to_base, time, odom_frame_, base_frame_));

        // 调试信息
        ROS_DEBUG("发布TF: %s -> %s -> %s [x=%.2f, y=%.2f, θ=%.2f]",
                  map_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str(), x, y, theta);
    }

    // 发布里程计消息
    void publishOdomMessage(const ros::Time &time,
                            double x, double y, double theta,
                            double linear, double angular)
    {
        tf::Quaternion odom_quat;
        odom_quat.setRPY(0, 0, theta);

        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = time;
        odom_msg.header.frame_id = odom_frame_; // 坐标系: odom
        odom_msg.child_frame_id = base_frame_;  // 子坐标系: base_link

        // 位置 (相对于odom坐标系)
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = 0.0;
        tf::quaternionTFToMsg(odom_quat, odom_msg.pose.pose.orientation);

        // 速度
        odom_msg.twist.twist.linear.x = linear;
        odom_msg.twist.twist.angular.z = angular;

        // 发布消息
        odom_pub_.publish(odom_msg);
    }

    // 发布总里程
    void publishTotalDistance()
    {
        if (!distance_pub_)
            return;

        double distance;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            distance = total_distance_;
        }

        std_msgs::Float32 msg;
        msg.data = distance;
        distance_pub_.publish(msg);
    }

    void publishMotionMode()
    {
        if (!mode_pub_)
            return;

        MotionMode current_mode;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            current_mode = motion_mode_;
        }

        std_msgs::Int32 msg;
        msg.data = static_cast<int>(current_mode);
        mode_pub_.publish(msg);
    }

    // 内轮转角转中央转角
    double ConvertInnerAngleToCentral(double inner_angle)
    {
        double phi_i = std::abs(inner_angle);
        double numerator = params_.wheelbase * std::sin(phi_i);
        double denominator = params_.wheelbase * std::cos(phi_i) + params_.track * std::sin(phi_i);
        double phi_central = std::atan(numerator / denominator);
        return (inner_angle >= 0 ? phi_central : -phi_central);
    }

    // 中央转角转内轮转角
    double ConvertCentralAngleToInner(double central_angle)
    {
        double phi = std::abs(central_angle);
        double numerator = params_.wheelbase * std::sin(phi);
        double denominator = params_.wheelbase * std::cos(phi) - params_.track * std::sin(phi);
        double phi_inner = std::atan(numerator / denominator);
        return (central_angle >= 0 ? phi_inner : -phi_inner);
    }

    // ROS1相关
    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    ros::Publisher mode_pub_;
    ros::Publisher distance_pub_; // 总里程发布器
    ros::Timer timer_;
    ros::Time last_update_;

    // 状态数据 - 这些是相对于odom坐标系的值
    mutable std::mutex data_mutex_;
    double position_x_ = 0.0; // 相对于odom原点的X位置
    double position_y_ = 0.0; // 相对于odom原点的Y位置
    double theta_ = 0.0;      // 相对于odom原点的朝向 (radians)
    double linear_velocity_ = 0.0;
    double angular_velocity_ = 0.0;
    double steering_angle_ = 0.0;
    double total_distance_ = 0.0; // 总里程计数器（单位：米）
    MotionMode motion_mode_ = MotionMode::DUAL_ACKERMAN;
    RobotParameters params_;

    // 坐标系名称
    std::string map_frame_ = "map";
    std::string odom_frame_ = "odom";
    std::string base_frame_ = "base_link";

    // 初始化控制
    mutable std::mutex init_mutex_;
    bool initialized_ = false;

    // 添加当前实际角速度状态变量
    double current_angular_velocity_ = 0.0; // 当前实际角速度 (rad/s)
};