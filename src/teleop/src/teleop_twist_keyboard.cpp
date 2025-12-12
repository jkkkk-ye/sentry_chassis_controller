#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <map>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <cmath>

// 键值定义
#define KEYCODE_ESC 0x1B
#define KEYCODE_SPACE 0x20
#define KEYCODE_TAB 0x09
#define KEYCODE_ENTER 0x0A
#define KEYCODE_BACKSPACE 0x7F

class TeleopTwistKeyboard
{
private:
    ros::NodeHandle nh_;
    ros::Publisher twist_pub_;
    geometry_msgs::Twist twist_;
    geometry_msgs::TwistStamped twist_stamped_;
    bool use_stamped_;
    std::string frame_id_;
    
    // 运动控制参数
    double linear_x_, linear_y_, linear_z_;
    double angular_z_;
    double linear_vel_, angular_vel_;
    double linear_vel_step_, angular_vel_step_;
    double linear_vel_max_, angular_vel_max_;
    
    // 发布线程
    std::thread pub_thread_;
    std::mutex mutex_;
    std::condition_variable cond_;
    std::atomic<bool> running_;
    double publish_rate_;
    
    // 键盘读取
    struct termios cooked_, raw_;
    int kfd_;
    
    // 帮助信息计数器
    int status_counter_;
    
    // 键位绑定
    std::map<char, std::tuple<double, double, double, double>> move_bindings_;
    std::map<char, std::pair<double, double>> speed_bindings_;

public:
    TeleopTwistKeyboard():
        linear_x_(0), linear_y_(0), linear_z_(0), angular_z_(0),
        linear_vel_(0.5), angular_vel_(1.0),
        linear_vel_step_(0.1), angular_vel_step_(0.1),
        linear_vel_max_(1000.0), angular_vel_max_(1000.0),
        running_(true),
        publish_rate_(0.0),
        use_stamped_(false),
        kfd_(0),
        status_counter_(0)
    {
        // 初始化键位绑定
        initializeKeyBindings();
        
        // 从参数服务器获取参数
        ros::NodeHandle private_nh("~");
        private_nh.param("speed", linear_vel_, 0.5);
        private_nh.param("turn", angular_vel_, 1.0);
        private_nh.param("speed_step", linear_vel_step_, 0.1);
        private_nh.param("turn_step", angular_vel_step_, 0.1);
        private_nh.param("speed_limit", linear_vel_max_, 1000.0);
        private_nh.param("turn_limit", angular_vel_max_, 1000.0);
        private_nh.param("repeat_rate", publish_rate_, 0.0);
        private_nh.param("stamped", use_stamped_, false);
        private_nh.param<std::string>("frame_id", frame_id_, "");
        
        // 设置发布器
        if (use_stamped_) {
            twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1);
            twist_stamped_.header.frame_id = frame_id_;
        } else {
            twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        }
        
        // 保存终端设置并设置为原始模式
        tcgetattr(kfd_, &cooked_);
        memcpy(&raw_, &cooked_, sizeof(struct termios));
        raw_.c_lflag &= ~(ICANON | ECHO);
        raw_.c_cc[VEOL] = 1;
        raw_.c_cc[VEOF] = 2;
        raw_.c_cc[VMIN] = 1;
        raw_.c_cc[VTIME] = 0;
        tcsetattr(kfd_, TCSANOW, &raw_);
        
        // 启动发布线程
        pub_thread_ = std::thread(&TeleopTwistKeyboard::publishLoop, this);
        
        // 等待订阅者连接
        waitForSubscribers();
        
        // 打印帮助信息
        printHelp();
    }
    
    void initializeKeyBindings()
    {
        // 运动绑定 - 与Python版本完全一致
        move_bindings_['i'] = std::make_tuple(1, 0, 0, 0);
        move_bindings_['o'] = std::make_tuple(1, 0, 0, -1);
        move_bindings_['j'] = std::make_tuple(0, 0, 0, 1);
        move_bindings_['l'] = std::make_tuple(0, 0, 0, -1);
        move_bindings_['u'] = std::make_tuple(1, 0, 0, 1);
        move_bindings_[','] = std::make_tuple(-1, 0, 0, 0);
        move_bindings_['.'] = std::make_tuple(-1, 0, 0, 1);
        move_bindings_['m'] = std::make_tuple(-1, 0, 0, 0);
        
        // 全向移动（需要Shift键）
        move_bindings_['I'] = std::make_tuple(1, 0, 0, 0);
        move_bindings_['O'] = std::make_tuple(1, -1, 0, 0);
        move_bindings_['J'] = std::make_tuple(0, 1, 0, 0);
        move_bindings_['L'] = std::make_tuple(0, -1, 0, 0);
        move_bindings_['U'] = std::make_tuple(1, 1, 0, 0);
        move_bindings_['<'] = std::make_tuple(-1, 0, 0, 0);
        move_bindings_['>'] = std::make_tuple(-1, -1, 0, 0);
        move_bindings_['M'] = std::make_tuple(-1, 1, 0, 0);
        
        // 垂直移动
        move_bindings_['t'] = std::make_tuple(0, 0, 1, 0);
        move_bindings_['b'] = std::make_tuple(0, 0, -1, 0);
        
        // 速度绑定
        speed_bindings_['q'] = std::make_pair(1.1, 1.1);
        speed_bindings_['z'] = std::make_pair(0.9, 0.9);
        speed_bindings_['w'] = std::make_pair(1.1, 1.0);
        speed_bindings_['x'] = std::make_pair(0.9, 1.0);
        speed_bindings_['e'] = std::make_pair(1.0, 1.1);
        speed_bindings_['c'] = std::make_pair(1.0, 0.9);
    }
    
    ~TeleopTwistKeyboard()
    {
        running_ = false;
        cond_.notify_one();
        if (pub_thread_.joinable()) {
            pub_thread_.join();
        }
        
        // 恢复终端设置
        tcsetattr(kfd_, TCSANOW, &cooked_);
        
        // 发布停止消息
        stopRobot();
    }
    
    void waitForSubscribers()
    {
        int i = 0;
        while (ros::ok() && twist_pub_.getNumSubscribers() == 0) {
            if (i == 4) {
                ROS_INFO("Waiting for subscriber to connect to cmd_vel topic...");
            }
            ros::Duration(0.5).sleep();
            i = (i + 1) % 5;
        }
        if (!ros::ok()) {
            throw std::runtime_error("ROS shutdown requested before subscribers connected");
        }
    }
    
    void printHelp()
    {
        std::cout << R"(
Reading from the keyboard and Publishing to Twist!
--------------------------------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
-------------------------------------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
)";
        printCurrentSpeed();
    }
    
    void printCurrentSpeed()
    {
        std::cout << "\nCurrently:\tspeed " << linear_vel_ << "\tturn " << angular_vel_ << std::endl;
    }
    
    void stopRobot()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        linear_x_ = linear_y_ = linear_z_ = 0;
        angular_z_ = 0;
        
        if (use_stamped_) {
            twist_stamped_.header.stamp = ros::Time::now();
            twist_stamped_.twist.linear.x = 0;
            twist_stamped_.twist.linear.y = 0;
            twist_stamped_.twist.linear.z = 0;
            twist_stamped_.twist.angular.x = 0;
            twist_stamped_.twist.angular.y = 0;
            twist_stamped_.twist.angular.z = 0;
            twist_pub_.publish(twist_stamped_);
        } else {
            twist_.linear.x = 0;
            twist_.linear.y = 0;
            twist_.linear.z = 0;
            twist_.angular.x = 0;
            twist_.angular.y = 0;
            twist_.angular.z = 0;
            twist_pub_.publish(twist_);
        }
    }
    
    void publishLoop()
    {
        ros::Rate rate(publish_rate_ > 0.0 ? publish_rate_ : 100);
        
        while (running_ && ros::ok()) {
            std::unique_lock<std::mutex> lock(mutex_);
            
            // 如果publish_rate > 0，使用超时等待
            if (publish_rate_ > 0.0) {
                if (cond_.wait_for(lock, 
                    std::chrono::milliseconds(static_cast<int>(1000.0/publish_rate_))) 
                    == std::cv_status::timeout) {
                    // 超时，继续发布当前速度
                }
            } else {
                // 事件驱动，等待通知
                cond_.wait(lock);
            }
            
            // 发布消息
            if (use_stamped_) {
                twist_stamped_.header.stamp = ros::Time::now();
                twist_stamped_.twist.linear.x = linear_x_ * linear_vel_;
                twist_stamped_.twist.linear.y = linear_y_ * linear_vel_;
                twist_stamped_.twist.linear.z = linear_z_ * linear_vel_;
                twist_stamped_.twist.angular.x = 0;
                twist_stamped_.twist.angular.y = 0;
                twist_stamped_.twist.angular.z = angular_z_ * angular_vel_;
                twist_pub_.publish(twist_stamped_);
            } else {
                twist_.linear.x = linear_x_ * linear_vel_;
                twist_.linear.y = linear_y_ * linear_vel_;
                twist_.linear.z = linear_z_ * linear_vel_;
                twist_.angular.x = 0;
                twist_.angular.y = 0;
                twist_.angular.z = angular_z_ * angular_vel_;
                twist_pub_.publish(twist_);
            }
            
            lock.unlock();
            
            if (publish_rate_ > 0.0) {
                rate.sleep();
            }
        }
    }
    
    void updateMotion(char key)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // 检查是否为运动绑定键
        auto move_it = move_bindings_.find(key);
        if (move_it != move_bindings_.end()) {
            // 更新运动方向
            auto& binding = move_it->second;
            linear_x_ = std::get<0>(binding);
            linear_y_ = std::get<1>(binding);
            linear_z_ = std::get<2>(binding);
            angular_z_ = std::get<3>(binding);
        } else {
            // 对于所有其他键，停止机器人（包括k键）
            linear_x_ = 0;
            linear_y_ = 0;
            linear_z_ = 0;
            angular_z_ = 0;
        }
        
        cond_.notify_one();
    }
    
    void updateSpeed(char key)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        auto speed_it = speed_bindings_.find(key);
        if (speed_it != speed_bindings_.end()) {
            auto& factors = speed_it->second;
            double linear_factor = factors.first;
            double angular_factor = factors.second;
            
            // 更新线速度
            linear_vel_ *= linear_factor;
            if (linear_vel_ > linear_vel_max_) {
                linear_vel_ = linear_vel_max_;
                std::cout << "\nLinear speed limit reached!" << std::endl;
            }
            
            // 更新角速度
            angular_vel_ *= angular_factor;
            if (angular_vel_ > angular_vel_max_) {
                angular_vel_ = angular_vel_max_;
                std::cout << "\nAngular speed limit reached!" << std::endl;
            }
            
            printCurrentSpeed();
            
            // 更新状态计数器，每15次显示一次帮助
            status_counter_++;
            if (status_counter_ >= 15) {
                std::cout << std::endl;
                printHelp();
                status_counter_ = 0;
            }
        }
    }
    
    void run()
    {
        std::cout << "Press any key to start controlling. Press '?' for help, CTRL-C to quit.\n";
        
        char c;
        while (running_ && ros::ok()) {
            // 非阻塞读取键盘
            fd_set readfds;
            struct timeval timeout;
            
            FD_ZERO(&readfds);
            FD_SET(kfd_, &readfds);
            
            // 设置超时（0.5秒）
            timeout.tv_sec = 0;
            timeout.tv_usec = 500000; // 0.5秒
            
            int ret = select(kfd_ + 1, &readfds, NULL, NULL, &timeout);
            
            if (ret == -1) {
                perror("select():");
                break;
            } else if (ret == 0) {
                // 超时，继续循环
                continue;
            }
            
            // 有键盘输入
            if (read(kfd_, &c, 1) < 0) {
                perror("read():");
                break;
            }
            
            // 处理特殊键
            if (c == '\x03') { // CTRL-C
                break;
            }
            
            // 显示帮助
            if (c == '?' || c == 'h') {
                printHelp();
                continue;
            }
            
            // 检查是否为速度调整键
            auto speed_it = speed_bindings_.find(c);
            if (speed_it != speed_bindings_.end()) {
                updateSpeed(c);
                continue;
            }
            
            // 处理运动键或其他键（包括k键）
            updateMotion(c);
        }
    }
};

// 全局变量用于信号处理
TeleopTwistKeyboard* teleop = nullptr;

void quit(int sig)
{
    (void)sig;
    std::cout << "\nShutting down..." << std::endl;
    if (teleop != nullptr) {
        delete teleop;
        teleop = nullptr;
    }
    ros::shutdown();
    exit(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_twist_keyboard_cpp");
    
    // 设置信号处理
    signal(SIGINT, quit);
    
    try {
        teleop = new TeleopTwistKeyboard();
        
        // 运行主循环
        teleop->run();
        
        delete teleop;
        teleop = nullptr;
    }
    catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        if (teleop != nullptr) {
            delete teleop;
            teleop = nullptr;
        }
        return 1;
    }
    catch (...) {
        ROS_ERROR("Unknown exception");
        if (teleop != nullptr) {
            delete teleop;
            teleop = nullptr;
        }
        return 1;
    }
    
    return 0;
}
