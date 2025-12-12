#include "sentry_chassis_controller/sentry_chassis_controller.h"
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

namespace sentry_chassis_controller {

SentryChassisController::SentryChassisController() 
    : tf_listener_(ros::Duration(10.0)),  // 关键修复：初始化TF监听器
      last_odom_update_time_(0) {}

SentryChassisController::~SentryChassisController() {}

bool SentryChassisController::init(hardware_interface::EffortJointInterface* hw,
                                   ros::NodeHandle& root_nh,
                                   ros::NodeHandle& controller_nh) {
    try {
        wheel_fl_ = hw->getHandle("left_front_wheel_joint");
        wheel_fr_ = hw->getHandle("right_front_wheel_joint");
        wheel_bl_ = hw->getHandle("left_back_wheel_joint");
        wheel_br_ = hw->getHandle("right_back_wheel_joint");
        
        steer_fl_ = hw->getHandle("left_front_pivot_joint");
        steer_fr_ = hw->getHandle("right_front_pivot_joint");
        steer_bl_ = hw->getHandle("left_back_pivot_joint");
        steer_br_ = hw->getHandle("right_back_pivot_joint");
    } catch (const hardware_interface::HardwareInterfaceException& e) {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
    }
    
    controller_nh.param("wheel_radius", wheel_radius_, 0.0762);
    controller_nh.param("wheel_track", wheel_track_, 0.362);
    controller_nh.param("wheel_base", wheel_base_, 0.362);

    // ===== 新增1：初始化功能优化相关参数（默认值与cfg文件一致）=====
    // 自锁功能参数
    controller_nh.param("lock_delay", lock_delay_, 1.0);          // 自锁延迟1秒
    controller_nh.param("lock_speed_thresh", lock_speed_thresh_, 0.001); // 静止阈值
    is_locked_ = false;                                          // 初始未自锁
    last_move_time_ = ros::Time::now();                          // 初始时间戳

    // 前馈控制增益
    controller_nh.param("wheel_feedforward_gain", wheel_feedforward_gain_, 0.8);
    controller_nh.param("steer_feedforward_gain", steer_feedforward_gain_, 0.8);

    // 上坡打滑抑制
    controller_nh.param("slip_thresh", slip_thresh_, 0.5);        // 打滑检测阈值
    controller_nh.param("slip_reduce_gain", slip_reduce_gain_, 0.7); // 打滑降速增益

    // 力矩限幅（电机保护）
    controller_nh.param("max_wheel_torque", max_wheel_torque_, 25.0); // 车轮最大力矩
    controller_nh.param("max_steer_torque", max_steer_torque_, 10.0); // 舵机最大力矩
    
    int mode;
    controller_nh.param("control_mode", mode, 0);
    control_mode_ = static_cast<ControlMode>(mode);
    
    controller_nh.param("global_frame", global_frame_, std::string("odom"));
    controller_nh.param("base_frame", base_frame_, std::string("base_link"));
    
    ROS_INFO("Control mode: %s", control_mode_ == BASE_LINK_MODE ? "BASE_LINK_MODE" : "GLOBAL_MODE");
    ROS_INFO("Global frame: %s, Base frame: %s", global_frame_.c_str(), base_frame_.c_str());
    
    // 如果是GLOBAL_MODE，添加额外信息
    if (control_mode_ == GLOBAL_MODE) {
        ROS_INFO("GLOBAL_MODE: Will transform velocities from %s to %s", 
                 global_frame_.c_str(), base_frame_.c_str());
    }
    
    if (!pid_wheel_fl_.init(ros::NodeHandle(controller_nh, "wheel_fl_pid"))) {
        ROS_ERROR("Failed to initialize wheel FL PID");
        return false;
    }
    if (!pid_wheel_fr_.init(ros::NodeHandle(controller_nh, "wheel_fr_pid"))) {
        ROS_ERROR("Failed to initialize wheel FR PID");
        return false;
    }
    if (!pid_wheel_bl_.init(ros::NodeHandle(controller_nh, "wheel_bl_pid"))) {
        ROS_ERROR("Failed to initialize wheel BL PID");
        return false;
    }
    if (!pid_wheel_br_.init(ros::NodeHandle(controller_nh, "wheel_br_pid"))) {
        ROS_ERROR("Failed to initialize wheel BR PID");
        return false;
    }
    
    if (!pid_steer_fl_.init(ros::NodeHandle(controller_nh, "steer_fl_pid"))) {
        ROS_ERROR("Failed to initialize steer FL PID");
        return false;
    }
    if (!pid_steer_fr_.init(ros::NodeHandle(controller_nh, "steer_fr_pid"))) {
        ROS_ERROR("Failed to initialize steer FR PID");
        return false;
    }
    if (!pid_steer_bl_.init(ros::NodeHandle(controller_nh, "steer_bl_pid"))) {
        ROS_ERROR("Failed to initialize steer BL PID");
        return false;
    }
    if (!pid_steer_br_.init(ros::NodeHandle(controller_nh, "steer_br_pid"))) {
        ROS_ERROR("Failed to initialize steer BR PID");
        return false;
    }
    
    cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>(
        "cmd_vel", 1, &SentryChassisController::cmdVelCallback, this);
    
    odom_pub_ = root_nh.advertise<nav_msgs::Odometry>("odom", 50);
    realtime_odom_publisher_.reset(
        new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(root_nh, "odom", 100));
    
    odometry_.x = 0.0;
    odometry_.y = 0.0;
    odometry_.theta = 0.0;
    odometry_.vx = 0.0;
    odometry_.vy = 0.0;
    odometry_.vtheta = 0.0;
    
    dyn_reconf_server_.reset(
        new dynamic_reconfigure::Server<SentryChassisConfig>(controller_nh));
    dyn_reconf_callback_ = boost::bind(&SentryChassisController::reconfigureCallback, this, _1, _2);
    dyn_reconf_server_->setCallback(dyn_reconf_callback_);
    
//=====新增=====    
    // 加速度限制参数 (可根据实际情况调整)
    controller_nh.param("max_accel_linear", max_accel_linear_, 1.0);   // 默认1.0 m/s²
    controller_nh.param("max_accel_angular", max_accel_angular_, 1.57); // 默认π/2 rad/s²

    // 初始化平滑指令
    filtered_cmd_vel_ = geometry_msgs::Twist();
    last_raw_cmd_vel_ = geometry_msgs::Twist();
    last_cmd_vel_time_ = ros::Time::now();
//==============    

    return true;
}

void SentryChassisController::starting(const ros::Time& time) {
    pid_wheel_fl_.reset();
    pid_wheel_fr_.reset();
    pid_wheel_bl_.reset();
    pid_wheel_br_.reset();
    
    pid_steer_fl_.reset();
    pid_steer_fr_.reset();
    pid_steer_bl_.reset();
    pid_steer_br_.reset();
    
    cmd_vel_buffer_.writeFromNonRT(geometry_msgs::Twist());
    
    odometry_.x = 0.0;
    odometry_.y = 0.0;
    odometry_.theta = 0.0;
    odometry_.vx = 0.0;
    odometry_.vy = 0.0;
    odometry_.vtheta = 0.0;
    
    last_odom_update_time_ = ros::Time::now();
    
    // 如果是GLOBAL_MODE，等待TF树建立
    if (control_mode_ == GLOBAL_MODE) {
        ROS_INFO("GLOBAL_MODE: Waiting for TF tree...");
        bool tf_ready = false;
        for (int i = 0; i < 20; ++i) {  // 等待最多2秒
            try {
                tf::StampedTransform transform;
                tf_listener_.lookupTransform(base_frame_, global_frame_, 
                                            ros::Time(0), transform);
                tf_ready = true;
                ROS_INFO("TF transform from %s to %s is ready!", 
                         global_frame_.c_str(), base_frame_.c_str());
                break;
            } catch (tf::TransformException &ex) {
                ROS_WARN_THROTTLE(0.5, "Waiting for TF... (%d/20): %s", 
                                 i+1, ex.what());
                ros::Duration(0.1).sleep();
            }
        }
        if (!tf_ready) {
            ROS_ERROR("TF transform not available after 2 seconds!");
        }
    }
}


/**
 * @brief 使用加速度限制对速度指令进行平滑处理
 * @param raw_cmd 新接收到的原始指令
 * @param period 控制周期
 */
void SentryChassisController::smoothVelocityCommand(const geometry_msgs::Twist& raw_cmd, const ros::Duration& period) {
    double dt = period.toSec();
    if (dt <= 0.0) {
        // 如果时间间隔无效，直接使用原始指令
        filtered_cmd_vel_ = raw_cmd;
        return;
    }

    // --- 线速度X轴的平滑 ---
    double delta_vx = raw_cmd.linear.x - filtered_cmd_vel_.linear.x;
    double max_delta_vx = max_accel_linear_ * dt;
    
    if (fabs(delta_vx) > max_delta_vx) {
        // 变化超过限制，使用最大变化率
        filtered_cmd_vel_.linear.x += (delta_vx > 0 ? max_delta_vx : -max_delta_vx);
    } else {
        // 变化在限制内，直接使用目标值
        filtered_cmd_vel_.linear.x = raw_cmd.linear.x;
    }

    // --- 线速度Y轴的平滑 ---
    double delta_vy = raw_cmd.linear.y - filtered_cmd_vel_.linear.y;
    double max_delta_vy = max_accel_linear_ * dt;
    
    if (fabs(delta_vy) > max_delta_vy) {
        filtered_cmd_vel_.linear.y += (delta_vy > 0 ? max_delta_vy : -max_delta_vy);
    } else {
        filtered_cmd_vel_.linear.y = raw_cmd.linear.y;
    }

    // --- 角速度Z轴的平滑 ---
    double delta_wz = raw_cmd.angular.z - filtered_cmd_vel_.angular.z;
    double max_delta_wz = max_accel_angular_ * dt;
    
    if (fabs(delta_wz) > max_delta_wz) {
        filtered_cmd_vel_.angular.z += (delta_wz > 0 ? max_delta_wz : -max_delta_wz);
    } else {
        filtered_cmd_vel_.angular.z = raw_cmd.angular.z;
    }

    // 可选：记录原始指令用于调试
    last_raw_cmd_vel_ = raw_cmd;
    last_cmd_vel_time_ = ros::Time::now();
}

void SentryChassisController::update(const ros::Time& time, const ros::Duration& period) {
    
    // ===== 新增1：实时性保护（互斥锁，避免多线程冲突）=====
    static boost::mutex control_mutex;
    boost::unique_lock<boost::mutex> lock(control_mutex);
/*    
    geometry_msgs::Twist cmd_vel_global = *(cmd_vel_buffer_.readFromRT());
    geometry_msgs::Twist cmd_vel_base;
*/
    geometry_msgs::Twist raw_cmd_vel = *(cmd_vel_buffer_.readFromRT());
    geometry_msgs::Twist cmd_vel_global;
    geometry_msgs::Twist cmd_vel_base;
    
    // ===== 新增关键部分：加速度限制（指令平滑）=====
    // 调用平滑函数，处理原始指令
    smoothVelocityCommand(raw_cmd_vel, period);
    // 使用平滑后的指令进行后续计算
    cmd_vel_global = filtered_cmd_vel_;
    
    
    // 调试信息
    static int count = 0;
    if (count++ % 100 == 0) {
        ROS_DEBUG("Mode: %s, Cmd: vx=%.2f, vy=%.2f, wz=%.2f",
                 control_mode_ == BASE_LINK_MODE ? "BASE" : "GLOBAL",
                 cmd_vel_global.linear.x, cmd_vel_global.linear.y, 
                 cmd_vel_global.angular.z);
    }
    
    if (control_mode_ == GLOBAL_MODE) {
        if (!transformVelocityToBaseLink(cmd_vel_global, cmd_vel_base)) {
            ROS_WARN_THROTTLE(1.0, "TF transform failed, using zero velocity");
            cmd_vel_base = geometry_msgs::Twist();
        }
    } else {
        cmd_vel_base = cmd_vel_global;
    }
    
    // 如果转换后的速度太小，也设置为零
    if (fabs(cmd_vel_base.linear.x) < 0.001 && 
        fabs(cmd_vel_base.linear.y) < 0.001 && 
        fabs(cmd_vel_base.angular.z) < 0.001) {
        cmd_vel_base = geometry_msgs::Twist();
    }
    
     // ===== 新增2：自锁功能（核心逻辑，不影响原有运动）=====
    static bool is_locked = false;          // 是否自锁
    static ros::Time last_move_time = time; // 上次运动时间
    const double lock_delay = 1.0;         // 1秒静止后自锁
    const double lock_speed_thresh = 0.001;// 静止阈值

    // 检测是否静止
    bool is_static = (fabs(cmd_vel_base.linear.x) < lock_speed_thresh &&
                      fabs(cmd_vel_base.linear.y) < lock_speed_thresh &&
                      fabs(cmd_vel_base.angular.z) < lock_speed_thresh);
    
    if (is_static) {
        // 静止超时，触发自锁
        if ((time - last_move_time).toSec() > lock_delay) {
            is_locked = true;
            // 自锁：强制全向轮角度为十字形（0°/90°/-90°/180°）
            double lock_angles[4] = {0.0, M_PI/2, -M_PI/2, M_PI};
            // 计算角度误差，锁定角度；车轮力矩归零
            double current_steer_pos_fl = angles::normalize_angle(steer_fl_.getPosition());
            double current_steer_pos_fr = angles::normalize_angle(steer_fr_.getPosition());
            double current_steer_pos_bl = angles::normalize_angle(steer_bl_.getPosition());
            double current_steer_pos_br = angles::normalize_angle(steer_br_.getPosition());

            double angle_error_fl = angles::shortest_angular_distance(current_steer_pos_fl, lock_angles[0]);
            double angle_error_fr = angles::shortest_angular_distance(current_steer_pos_fr, lock_angles[1]);
            double angle_error_bl = angles::shortest_angular_distance(current_steer_pos_bl, lock_angles[2]);
            double angle_error_br = angles::shortest_angular_distance(current_steer_pos_br, lock_angles[3]);
            

            // 舵机PID锁定角度，车轮力矩归零
            double steer_torque_fl = pid_steer_fl_.computeCommand(angle_error_fl, period);
            double steer_torque_fr = pid_steer_fr_.computeCommand(angle_error_fr, period);
            double steer_torque_bl = pid_steer_bl_.computeCommand(angle_error_bl, period);
            double steer_torque_br = pid_steer_br_.computeCommand(angle_error_br, period);
            
            
            

            // 新增：舵机力矩限幅（保护）
            const double max_steer_torque = 10.0; // 可配置，单位N·m
            steer_torque_fl = std::max(-max_steer_torque, std::min(steer_torque_fl, max_steer_torque));
            steer_torque_fr = std::max(-max_steer_torque, std::min(steer_torque_fr, max_steer_torque));
            steer_torque_bl = std::max(-max_steer_torque, std::min(steer_torque_bl, max_steer_torque));
            steer_torque_br = std::max(-max_steer_torque, std::min(steer_torque_br, max_steer_torque));

            // 下发指令：车轮归零，舵机锁定角度
            wheel_fl_.setCommand(0.0);
            wheel_fr_.setCommand(0.0);
            wheel_bl_.setCommand(0.0);
            wheel_br_.setCommand(0.0);

            steer_fl_.setCommand(steer_torque_fl);
            steer_fr_.setCommand(steer_torque_fr);
            steer_bl_.setCommand(steer_torque_bl);
            steer_br_.setCommand(steer_torque_br);

            ROS_INFO_THROTTLE(1.0, "全向轮自锁触发！角度锁定为十字形");
            computeOdometry(period);
            publishOdometry(time);
            return; // 跳过后续运动逻辑
        }
    } else {
        // 有运动，更新最后运动时间，解除自锁
        last_move_time = time;
        is_locked = false;
    }
    double wheel_angles[4];
    double wheel_speeds[4];
    computeWheelCommands(cmd_vel_base, wheel_angles, wheel_speeds);
    
    double wheel_velocities[4];
    for (int i = 0; i < 4; i++) {
        wheel_velocities[i] = wheel_speeds[i] / wheel_radius_;
    }
    
    double current_wheel_vel_fl = wheel_fl_.getVelocity();
    double current_wheel_vel_fr = wheel_fr_.getVelocity();
    double current_wheel_vel_bl = wheel_bl_.getVelocity();
    double current_wheel_vel_br = wheel_br_.getVelocity();
    
    // double current_steer_pos_fl = steer_fl_.getPosition();
    // double current_steer_pos_fr = steer_fr_.getPosition();
    // double current_steer_pos_bl = steer_bl_.getPosition();
    // double current_steer_pos_br = steer_br_.getPosition();

    // 建议修改为：
    double current_steer_pos_fl = angles::normalize_angle(steer_fl_.getPosition());
    double current_steer_pos_fr = angles::normalize_angle(steer_fr_.getPosition());
    double current_steer_pos_bl = angles::normalize_angle(steer_bl_.getPosition());
    double current_steer_pos_br = angles::normalize_angle(steer_br_.getPosition());
    
    for (int i = 0; i < 4; i++) {
        wheel_angles[i] = angles::normalize_angle(wheel_angles[i]);
    }
    
    double angle_error_fl = angles::shortest_angular_distance(current_steer_pos_fl, wheel_angles[0]);
    double angle_error_fr = angles::shortest_angular_distance(current_steer_pos_fr, wheel_angles[1]);
    double angle_error_bl = angles::shortest_angular_distance(current_steer_pos_bl, wheel_angles[2]);
    double angle_error_br = angles::shortest_angular_distance(current_steer_pos_br, wheel_angles[3]);
    
    // ===== 新增3：前馈控制（提升响应速度，解决上坡掉速）=====
    const double wheel_feedforward_gain = 0.8; // 前馈增益（可配置）
    const double steer_feedforward_gain = 0.8; // 前馈增益（可配置）

    // double wheel_torque_fl = pid_wheel_fl_.computeCommand(wheel_velocities[0] - current_wheel_vel_fl, period);
    // double wheel_torque_fr = pid_wheel_fr_.computeCommand(wheel_velocities[1] - current_wheel_vel_fr, period);
    // double wheel_torque_bl = pid_wheel_bl_.computeCommand(wheel_velocities[2] - current_wheel_vel_bl, period);
    // double wheel_torque_br = pid_wheel_br_.computeCommand(wheel_velocities[3] - current_wheel_vel_br, period);
    
    // double steer_torque_fl = pid_steer_fl_.computeCommand(angle_error_fl, period);
    // double steer_torque_fr = pid_steer_fr_.computeCommand(angle_error_fr, period);
    // double steer_torque_bl = pid_steer_bl_.computeCommand(angle_error_bl, period);
    // double steer_torque_br = pid_steer_br_.computeCommand(angle_error_br, period);
   
    // 车轮PID + 前馈（前馈基于目标速度）
    double wheel_torque_fl = wheel_feedforward_gain * wheel_velocities[0] +
                             pid_wheel_fl_.computeCommand(wheel_velocities[0] - current_wheel_vel_fl, period);
    double wheel_torque_fr = wheel_feedforward_gain * wheel_velocities[1] +
                             pid_wheel_fr_.computeCommand(wheel_velocities[1] - current_wheel_vel_fr, period);
    double wheel_torque_bl = wheel_feedforward_gain * wheel_velocities[2] +
                             pid_wheel_bl_.computeCommand(wheel_velocities[2] - current_wheel_vel_bl, period);
    double wheel_torque_br = wheel_feedforward_gain * wheel_velocities[3] +
                             pid_wheel_br_.computeCommand(wheel_velocities[3] - current_wheel_vel_br, period);

    // 舵机PID + 前馈（前馈基于角度误差）
    double steer_torque_fl = steer_feedforward_gain * angle_error_fl +
                             pid_steer_fl_.computeCommand(angle_error_fl, period);
    double steer_torque_fr = steer_feedforward_gain * angle_error_fr +
                             pid_steer_fr_.computeCommand(angle_error_fr, period);
    double steer_torque_bl = steer_feedforward_gain * angle_error_bl +
                             pid_steer_bl_.computeCommand(angle_error_bl, period);
    double steer_torque_br = steer_feedforward_gain * angle_error_br +
                             pid_steer_br_.computeCommand(angle_error_br, period);
  
     // ===== 新增4：上坡打滑抑制（轮速差检测）=====
    const double slip_thresh = 0.5;    // 打滑阈值（rad/s）
    const double slip_reduce_gain = 0.7; // 打滑降速30%
    
    // 检测左前轮打滑
    if (wheel_velocities[0] - current_wheel_vel_fl > slip_thresh) {
        wheel_torque_fl *= slip_reduce_gain; // 降速抑制打滑
        ROS_WARN_THROTTLE(1.0, "左前轮打滑！降速补偿");
    }
    if (wheel_velocities[1] - current_wheel_vel_fr > slip_thresh) {
        wheel_torque_fr *= slip_reduce_gain;
        ROS_WARN_THROTTLE(1.0, "右前轮打滑！降速补偿");
    }
    if (wheel_velocities[2] - current_wheel_vel_bl > slip_thresh) {
        wheel_torque_bl *= slip_reduce_gain;
        ROS_WARN_THROTTLE(1.0, "左后轮打滑！降速补偿");
    }
    if (wheel_velocities[3] - current_wheel_vel_br > slip_thresh) {
        wheel_torque_br *= slip_reduce_gain;
        ROS_WARN_THROTTLE(1.0, "右后轮打滑！降速补偿");
    }

     // ===== 新增5：力矩限幅（保护电机，上坡可增大）=====
    const double max_wheel_torque = 25.0; // 上坡增大力矩到25N·m（可配置）
    const double max_steer_torque = 10.0; // 舵机最大力矩

    // 车轮力矩限幅
    wheel_torque_fl = std::max(-max_wheel_torque, std::min(wheel_torque_fl, max_wheel_torque));
    wheel_torque_fr = std::max(-max_wheel_torque, std::min(wheel_torque_fr, max_wheel_torque));
    wheel_torque_bl = std::max(-max_wheel_torque, std::min(wheel_torque_bl, max_wheel_torque));
    wheel_torque_br = std::max(-max_wheel_torque, std::min(wheel_torque_br, max_wheel_torque));

    // 舵机力矩限幅
    steer_torque_fl = std::max(-max_steer_torque, std::min(steer_torque_fl, max_steer_torque));
    steer_torque_fr = std::max(-max_steer_torque, std::min(steer_torque_fr, max_steer_torque));
    steer_torque_bl = std::max(-max_steer_torque, std::min(steer_torque_bl, max_steer_torque));
    steer_torque_br = std::max(-max_steer_torque, std::min(steer_torque_br, max_steer_torque));

    wheel_fl_.setCommand(wheel_torque_fl);
    wheel_fr_.setCommand(wheel_torque_fr);
    wheel_bl_.setCommand(wheel_torque_bl);
    wheel_br_.setCommand(wheel_torque_br);
    
    steer_fl_.setCommand(steer_torque_fl);
    steer_fr_.setCommand(steer_torque_fr);
    steer_bl_.setCommand(steer_torque_bl);
    steer_br_.setCommand(steer_torque_br);
    
    computeOdometry(period);
    publishOdometry(time);
}

void SentryChassisController::stopping(const ros::Time& time) {
    wheel_fl_.setCommand(0.0);
    wheel_fr_.setCommand(0.0);
    wheel_bl_.setCommand(0.0);
    wheel_br_.setCommand(0.0);
    
    steer_fl_.setCommand(0.0);
    steer_fr_.setCommand(0.0);
    steer_bl_.setCommand(0.0);
    steer_br_.setCommand(0.0);
}

void SentryChassisController::computeWheelCommands(const geometry_msgs::Twist& cmd_vel,
                                                   double wheel_angles[4],
                                                   double wheel_speeds[4]) {
    double vx = cmd_vel.linear.x;
    double vy = cmd_vel.linear.y;
    double omega = cmd_vel.angular.z;
    
    double half_track = wheel_track_ / 2.0;
    double half_base = wheel_base_ / 2.0;
    
    double wheel_positions[4][2] = {
        {half_track,  half_base},
        {half_track, -half_base},
        {-half_track,  half_base},
        {-half_track, -half_base}
    };
    
    for (int i = 0; i < 4; i++) {
        double x = wheel_positions[i][0];
        double y = wheel_positions[i][1];
        
        double wheel_vx = vx - omega * y;
        double wheel_vy = vy + omega * x;
        
        wheel_angles[i] = atan2(wheel_vy, wheel_vx);
        wheel_speeds[i] = sqrt(wheel_vx * wheel_vx + wheel_vy * wheel_vy);
        
         // ===== 增量添加：鲁棒性优化（只加这部分）=====
        // 1. 角度归一化（避免角度超出[-π, π]，减少抖动）
        wheel_angles[i] = angles::normalize_angle(wheel_angles[i]);

        if (fabs(wheel_speeds[i]) < 0.01) {
            wheel_speeds[i] = 0.0;
            wheel_angles[i] = 0.0;
        }
        // 3. 最大轮速限制（防止超调）
        wheel_speeds[i] = std::max(-2.0, std::min(wheel_speeds[i], 2.0));
        // ===== 增量添加结束 =====
    }
}

void SentryChassisController::forwardKinematics(double wheel_angular_velocities[4], 
                                               double wheel_angles[4],
                                               double& vx, double& vy, double& omega) {
    double half_track = wheel_track_ / 2.0;
    double half_base = wheel_base_ / 2.0;
    
    double wheel_positions[4][2] = {
        {half_track,  half_base},
        {half_track, -half_base},
        {-half_track,  half_base},
        {-half_track, -half_base}
    };
    
    double wheel_vx[4], wheel_vy[4];
    for (int i = 0; i < 4; i++) {
        double wheel_speed = wheel_angular_velocities[i] * wheel_radius_;
        // ===== 新增1：角度归一化（避免轮角超出[-π,π]导致cos/sin计算误差）=====
        double normalized_angle = angles::normalize_angle(wheel_angles[i]);
        
         // ===== x/y在这里定义，作用域覆盖后续逻辑 =====
        double x = wheel_positions[i][0];
        double y = wheel_positions[i][1];

        // wheel_vx[i] = wheel_speed * cos(wheel_angles[i]);
        // wheel_vy[i] = wheel_speed * sin(wheel_angles[i]);

        wheel_vx[i] = wheel_speed * cos(normalized_angle); // 改用归一化角度
        wheel_vy[i] = wheel_speed * sin(normalized_angle); // 改用归一化角度

        
        // ===== 新增2：小速度过滤（单轮零漂不会被累加）=====
        const double speed_thresh = 0.001; // 小速度阈值
        if (fabs(wheel_speed) < speed_thresh) {
            wheel_vx[i] = 0.0;
            wheel_vy[i] = 0.0;
        }

       

        // ===== 新增3：轮速限幅（单轮打滑/异常值不会拉偏结果）=====
        const double max_wheel_speed = 2.0; // 最大轮速（m/s，和逆解一致）
        wheel_vx[i] = std::max(-max_wheel_speed, std::min(wheel_vx[i], max_wheel_speed));
        wheel_vy[i] = std::max(-max_wheel_speed, std::min(wheel_vy[i], max_wheel_speed));
    
    }
    
    vx = 0.0;
    vy = 0.0;
    omega = 0.0;
    
    for (int i = 0; i < 4; i++) {
        double x = wheel_positions[i][0];
        double y = wheel_positions[i][1];

         // ===== 新增4：分母保护（理论上x²+y²≠0，防止极端情况除以0）=====
        double denom = x*x + y*y;
        if (denom < 1e-6) { // 分母过小，跳过该轮（避免NaN）
            continue;
        }
        
        vx += wheel_vx[i];
        vy += wheel_vy[i];
        omega += (-wheel_vx[i] * y + wheel_vy[i] * x) / (x*x + y*y);
    }
    
    vx /= 4.0;
    vy /= 4.0;
    omega /= 4.0;

    // ===== 新增5：最终速度零漂过滤（里程计无漂移）=====
    const double final_thresh = 0.001;
    vx = (fabs(vx) < final_thresh) ? 0.0 : vx;
    vy = (fabs(vy) < final_thresh) ? 0.0 : vy;
    omega = (fabs(omega) < final_thresh) ? 0.0 : omega;
}

bool SentryChassisController::transformVelocityToBaseLink(const geometry_msgs::Twist& global_vel,
                                                          geometry_msgs::Twist& base_vel) {
    try {
        // 等待TF变换可用（最多0.1秒）
        if (!tf_listener_.waitForTransform(base_frame_, global_frame_, 
                                          ros::Time(0), ros::Duration(0.1))) {
            ROS_DEBUG("TF transform not available");
            return false;
        }
        
        tf::StampedTransform transform;
        tf_listener_.lookupTransform(base_frame_, global_frame_, ros::Time(0), transform);
        
        double yaw = tf::getYaw(transform.getRotation());
        
        // 速度变换：从全局坐标系到底盘坐标系
        base_vel.linear.x = global_vel.linear.x * cos(yaw) + global_vel.linear.y * sin(yaw);
        base_vel.linear.y = -global_vel.linear.x * sin(yaw) + global_vel.linear.y * cos(yaw);
        base_vel.angular.z = global_vel.angular.z;  // 角速度不变
        
        ROS_DEBUG_THROTTLE(1.0, "TF transform: global(%.2f,%.2f)->base(%.2f,%.2f), yaw=%.1f°",
                          global_vel.linear.x, global_vel.linear.y,
                          base_vel.linear.x, base_vel.linear.y,
                          yaw * 180.0 / M_PI);
        
        return true;
    } catch (tf::TransformException &ex) {
        ROS_WARN_THROTTLE(1.0, "TF transform error: %s", ex.what());
        return false;
    }
}

void SentryChassisController::computeOdometry(const ros::Duration& period) {
    double current_wheel_vel_fl = wheel_fl_.getVelocity();
    double current_wheel_vel_fr = wheel_fr_.getVelocity();
    double current_wheel_vel_bl = wheel_bl_.getVelocity();
    double current_wheel_vel_br = wheel_br_.getVelocity();
    
    double current_steer_pos_fl = steer_fl_.getPosition();
    double current_steer_pos_fr = steer_fr_.getPosition();
    double current_steer_pos_bl = steer_bl_.getPosition();
    double current_steer_pos_br = steer_br_.getPosition();
    
    double wheel_angular_velocities[4] = {
        current_wheel_vel_fl,
        current_wheel_vel_fr,
        current_wheel_vel_bl,
        current_wheel_vel_br
    };
    
    // double wheel_angles[4] = {
    //     current_steer_pos_fl,
    //     current_steer_pos_fr,
    //     current_steer_pos_bl,
    //     current_steer_pos_br

    double wheel_angles[4] = {
    angles::normalize_angle(current_steer_pos_fl),
    angles::normalize_angle(current_steer_pos_fr),
    angles::normalize_angle(current_steer_pos_bl),
    angles::normalize_angle(current_steer_pos_br)
    };
    
    
    double vx, vy, omega;
    forwardKinematics(wheel_angular_velocities, wheel_angles, vx, vy, omega);
    
    odometry_.vx = vx;
    odometry_.vy = vy;
    odometry_.vtheta = omega;
    
    double dt = period.toSec();
    double delta_x = (odometry_.vx * cos(odometry_.theta) - odometry_.vy * sin(odometry_.theta)) * dt;
    double delta_y = (odometry_.vx * sin(odometry_.theta) + odometry_.vy * cos(odometry_.theta)) * dt;
    double delta_theta = odometry_.vtheta * dt;
    
    odometry_.x += delta_x;
    odometry_.y += delta_y;
    odometry_.theta += delta_theta;
    odometry_.theta = angles::normalize_angle(odometry_.theta);
}

void SentryChassisController::publishOdometry(const ros::Time& time) {
    if (realtime_odom_publisher_->trylock()) {
        realtime_odom_publisher_->msg_.header.stamp = time;
        realtime_odom_publisher_->msg_.header.frame_id = "odom";
        realtime_odom_publisher_->msg_.child_frame_id = "base_link";
        
        realtime_odom_publisher_->msg_.pose.pose.position.x = odometry_.x;
        realtime_odom_publisher_->msg_.pose.pose.position.y = odometry_.y;
        realtime_odom_publisher_->msg_.pose.pose.position.z = 0.0;
        
        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(odometry_.theta);
        realtime_odom_publisher_->msg_.pose.pose.orientation = quat;
        
        realtime_odom_publisher_->msg_.twist.twist.linear.x = odometry_.vx;
        realtime_odom_publisher_->msg_.twist.twist.linear.y = odometry_.vy;
        realtime_odom_publisher_->msg_.twist.twist.angular.z = odometry_.vtheta;
        
        realtime_odom_publisher_->unlockAndPublish();
    }
    
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    
    odom_trans.transform.translation.x = odometry_.x;
    odom_trans.transform.translation.y = odometry_.y;
    odom_trans.transform.translation.z = 0.0;
    
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odometry_.theta);
    odom_trans.transform.rotation = odom_quat;
    
    tf_broadcaster_.sendTransform(odom_trans);
}

void SentryChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    cmd_vel_buffer_.writeFromNonRT(*msg);
}

void SentryChassisController::reconfigureCallback(SentryChassisConfig &config, uint32_t level) {
    pid_wheel_fl_.setGains(config.wheel_fl_p, config.wheel_fl_i, config.wheel_fl_d,
                           config.wheel_fl_i_max, config.wheel_fl_i_min);
    pid_wheel_fr_.setGains(config.wheel_fr_p, config.wheel_fr_i, config.wheel_fr_d,
                           config.wheel_fr_i_max, config.wheel_fr_i_min);
    pid_wheel_bl_.setGains(config.wheel_bl_p, config.wheel_bl_i, config.wheel_bl_d,
                           config.wheel_bl_i_max, config.wheel_bl_i_min);
    pid_wheel_br_.setGains(config.wheel_br_p, config.wheel_br_i, config.wheel_br_d,
                           config.wheel_br_i_max, config.wheel_br_i_min);
    
    pid_steer_fl_.setGains(config.steer_fl_p, config.steer_fl_i, config.steer_fl_d,
                           config.steer_fl_i_max, config.steer_fl_i_min);
    pid_steer_fr_.setGains(config.steer_fr_p, config.steer_fr_i, config.steer_fr_d,
                           config.steer_fr_i_max, config.steer_fr_i_min);
    pid_steer_bl_.setGains(config.steer_bl_p, config.steer_bl_i, config.steer_bl_d,
                           config.steer_bl_i_max, config.steer_bl_i_min);
    pid_steer_br_.setGains(config.steer_br_p, config.steer_br_i, config.steer_br_d,
                           config.steer_br_i_max, config.steer_br_i_min);
    
                           // ===== 新增：加载功能优化参数 =====
    lock_delay_ = config.lock_delay;
    lock_speed_thresh_ = config.lock_speed_thresh;
    wheel_feedforward_gain_ = config.wheel_feedforward_gain;
    steer_feedforward_gain_ = config.steer_feedforward_gain;
    slip_thresh_ = config.slip_thresh;
    slip_reduce_gain_ = config.slip_reduce_gain;
    max_wheel_torque_ = config.max_wheel_torque;
    max_steer_torque_ = config.max_steer_torque;
    wheel_radius_ = config.wheel_radius;
    wheel_track_ = config.wheel_track;
    wheel_base_ = config.wheel_base;
    
    if (config.control_mode == 0) {
        control_mode_ = BASE_LINK_MODE;
    } else {
        control_mode_ = GLOBAL_MODE;
    }
    
    ROS_INFO("Control mode changed to: %s", control_mode_ == BASE_LINK_MODE ? "BASE_LINK_MODE" : "GLOBAL_MODE");
    
//===========    新增：加速度参数可调
    max_accel_linear_ = config.max_accel_linear;
    max_accel_angular_ = config.max_accel_angular;
//==============
}

PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SentryChassisController,
                       controller_interface::ControllerBase)
}
