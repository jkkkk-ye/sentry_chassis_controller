#ifndef SENTRY_CHASSIS_CONTROLLER_H
#define SENTRY_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include "sentry_chassis_controller/SentryChassisConfig.h"
// ===== 新增：必要的依赖头文件 =====
#include <boost/thread/mutex.hpp>  // 互斥锁（实时性保护）
#include <angles/angles.h>         // 角度归一化（全向轮角度处理）

namespace sentry_chassis_controller {

class SentryChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
    SentryChassisController();
    ~SentryChassisController();
    
    bool init(hardware_interface::EffortJointInterface* hw,
              ros::NodeHandle& root_nh,
              ros::NodeHandle& controller_nh) override;
    
    void starting(const ros::Time& time) override;
    void update(const ros::Time& time, const ros::Duration& period) override;
    void stopping(const ros::Time& time) override;
    
private:
    enum ControlMode {
        BASE_LINK_MODE = 0,
        GLOBAL_MODE = 1
    };
    
    control_toolbox::Pid pid_wheel_fl_, pid_wheel_fr_, pid_wheel_bl_, pid_wheel_br_;
    control_toolbox::Pid pid_steer_fl_, pid_steer_fr_, pid_steer_bl_, pid_steer_br_;
    
    hardware_interface::JointHandle wheel_fl_, wheel_fr_, wheel_bl_, wheel_br_;
    hardware_interface::JointHandle steer_fl_, steer_fr_, steer_bl_, steer_br_;
    
    double wheel_radius_;
    double wheel_track_;
    double wheel_base_;
    
    ControlMode control_mode_;
    std::string global_frame_;
    std::string base_frame_;
    
    realtime_tools::RealtimeBuffer<geometry_msgs::Twist> cmd_vel_buffer_;
    ros::Subscriber cmd_vel_sub_;
    
    boost::shared_ptr<dynamic_reconfigure::Server<SentryChassisConfig>> dyn_reconf_server_;
    dynamic_reconfigure::Server<SentryChassisConfig>::CallbackType dyn_reconf_callback_;
    
    ros::Publisher odom_pub_;
    boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> realtime_odom_publisher_;
    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener tf_listener_;
    
    struct Odometry {
        double x;
        double y;
        double theta;
        double vx;
        double vy;
        double vtheta;
    };
    Odometry odometry_;
    ros::Time last_odom_update_time_;
    
    // ===== 新增：核心优化/功能对应的成员变量 =====
    // 1. 实时性保护：互斥锁（包裹控制逻辑）
    boost::mutex control_mutex_;
    // 2. 自锁功能相关（参数+状态）
    bool is_locked_;               // 自锁状态标记
    ros::Time last_move_time_;     // 上次运动时间（判断静止超时）
    double lock_delay_;            // 自锁延迟时间（秒，如1秒）
    double lock_speed_thresh_;     // 自锁静止阈值（速度<该值判定为静止）

    // 3. 前馈控制增益（提升响应速度，解决上坡掉速）
    double wheel_feedforward_gain_;  // 车轮速度前馈增益
    double steer_feedforward_gain_;  // 舵机角度前馈增益

    // 4. 打滑抑制相关（上坡专用）
    double slip_thresh_;           // 打滑检测阈值（目标-实际轮速>该值判定为打滑）
    double slip_reduce_gain_;      // 打滑降速增益（如0.7，降30%）

    // 5. 力矩限幅参数（电机保护+爬坡增力）
    double max_wheel_torque_;      // 车轮最大输出力矩（N·m）
    double max_steer_torque_;      // 舵机最大输出力矩（N·m）
    
    //加速度限制新增
    // 指令平滑滤波器
    geometry_msgs::Twist filtered_cmd_vel_; // 经过平滑后的当前指令
    geometry_msgs::Twist last_raw_cmd_vel_; // 上一周期的原始指令，用于计算变化
    ros::Time last_cmd_vel_time_;           // 上次收到指令的时间
    double max_accel_linear_;               // 最大线加速度 (m/s²)
    double max_accel_angular_;              // 最大角加速度 (rad/s²)
    
    
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void reconfigureCallback(SentryChassisConfig &config, uint32_t level);
    void computeWheelCommands(const geometry_msgs::Twist& cmd_vel,
                              double wheel_angles[4],
                              double wheel_speeds[4]);
    void computeOdometry(const ros::Duration& period);
    void publishOdometry(const ros::Time& time);
    void forwardKinematics(double wheel_angular_velocities[4], 
                          double wheel_angles[4],
                          double& vx, double& vy, double& omega);
    bool transformVelocityToBaseLink(const geometry_msgs::Twist& global_vel,
                                     geometry_msgs::Twist& base_vel);
/**
     * @brief 使用加速度限制对速度指令进行平滑处理
     * @param raw_cmd 新接收到的原始指令
     * @param period 控制周期
     */
    void smoothVelocityCommand(const geometry_msgs::Twist& raw_cmd, const ros::Duration& period);                                 
};

}
#endif
