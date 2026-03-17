#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <vector>
#include <algorithm>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

class SafetyCerebellumNode : public rclcpp::Node {
public:
    SafetyCerebellumNode() : Node("safety_cerebellum_node") {
        this->declare_parameter<bool>("dry_run_mode", true);
        
        // --- 1. 参数初始化 ---
        // 前 4 个为轮速变化率，后 4 个为 EHA 腿部位姿变化率 (8维)
        max_action_delta_ = {0.005, 0.005, 0.005, 0.005, 0.0001, 0.0001, 0.0001, 0.0001}; 
        current_safe_action_.resize(8, 0.0);
        
        e_stop_active_ = false;
        proj_grav_z_ = -1.0; 
        cmd_is_zero_ = true;

        // --- 2. 订阅器 ---
        sub_ai_action_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/robot/action_command_raw", 10, 
            std::bind(&SafetyCerebellumNode::aiActionCallback, this, std::placeholders::_1));

        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", 10,
            std::bind(&SafetyCerebellumNode::imuCallback, this, std::placeholders::_1));

        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&SafetyCerebellumNode::cmdVelCallback, this, std::placeholders::_1));

        sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&SafetyCerebellumNode::joyCallback, this, std::placeholders::_1));

        // --- 3. 发布器 ---
        pub_hw_command_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/robot/hardware_command", 10);

        // --- 4. 看门狗与控制循环 (50Hz) ---
        last_ai_time_ = this->get_clock()->now();
        last_imu_time_ = this->get_clock()->now();
        timer_ = this->create_wall_timer(20ms, std::bind(&SafetyCerebellumNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "🛡️ 硬件安全小脑(8维架构)已启动！守护神已就位。");
    }

private:
    std::vector<double> max_action_delta_;
    std::vector<double> current_safe_action_;
    std::vector<double> target_ai_action_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
    
    bool e_stop_active_;
    double proj_grav_z_;
    bool cmd_is_zero_;

    rclcpp::Time last_ai_time_;
    rclcpp::Time last_imu_time_;

    void aiActionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() == 8) {
            for(size_t i=0; i<8; ++i) target_ai_action_[i] = msg->data[i];
            last_ai_time_ = this->get_clock()->now();
        }
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        proj_grav_z_ = -cos(roll) * cos(pitch); 
        last_imu_time_ = this->get_clock()->now();
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (std::abs(msg->linear.x) < 0.01 && std::abs(msg->angular.z) < 0.01) {
            cmd_is_zero_ = true;
        } else {
            cmd_is_zero_ = false;
        }
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->buttons.size() > 7 && msg->buttons[7] == 1) {
            if (!e_stop_active_) {
                RCLCPP_ERROR(this->get_logger(), "🛑 触发硬件级物理急停！切断所有动力！");
                e_stop_active_ = true;
            }
        }
    }

    void controlLoop() {
        std_msgs::msg::Float32MultiArray hw_msg;
        hw_msg.data.resize(8, 0.0);

        // 【防线 1】：最高优先级 - 硬件急停 (E-Stop)
        if (e_stop_active_) {
            pub_hw_command_->publish(hw_msg);
            return; 
        }

        // 【防线 2】：看门狗 (Watchdog) 
        auto now = this->get_clock()->now();
        if ((now - last_ai_time_).seconds() > 0.1 || (now - last_imu_time_).seconds() > 0.1) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "⚠️ 看门狗触发！传感器或 AI 指令超时，强制停车！");
            pub_hw_command_->publish(hw_msg); 
            return;
        }

        // 【防线 3】：倾覆保护 (Rollover Prevention)
        if (proj_grav_z_ > -0.7) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "🚨 姿态异常！触发倾覆保护！");
            pub_hw_command_->publish(hw_msg); 
            return;
        }

        // 【防线 4】：无输入死区屏蔽 (Zero-Command Deadband)
        if (cmd_is_zero_) {
            // 【核心修复】：由于现在前 4 个维度全部是轮子，需要把 4 个轮速都强行置 0
            target_ai_action_[0] = 0.0; // FL_wheel
            target_ai_action_[1] = 0.0; // FR_wheel
            target_ai_action_[2] = 0.0; // RL_wheel
            target_ai_action_[3] = 0.0; // RR_wheel
            // 注：不清理 target_ai_action_[4~7]，保留 4 条 EHA 腿的位姿调整能力
        }

        // 【防线 5】：动作变化率限幅 (Slew Rate Limiting)
        // 注意：循环范围现在严格是 8
        for (size_t i = 0; i < 8; ++i) {
            double delta = target_ai_action_[i] - current_safe_action_[i];
            
            if (delta > max_action_delta_[i]) {
                current_safe_action_[i] += max_action_delta_[i];
            } else if (delta < -max_action_delta_[i]) {
                current_safe_action_[i] -= max_action_delta_[i];
            } else {
                current_safe_action_[i] = target_ai_action_[i];
            }
        }

        // 【防线 6 - 诊断模式/干跑模式】
        bool is_dry_run = this->get_parameter("dry_run_mode").as_bool();

        if (is_dry_run) {
            // 【核心修复】：将打印信息扩展到 8 维，且中间用 | 隔开轮子和腿，方便人类阅读
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "🩺 [干跑锁死中] AI期望 [轮: %.2f %.2f %.2f %.2f | 腿: %.2f %.2f %.2f %.2f]  -->  限幅输出 [轮: %.2f %.2f %.2f %.2f | 腿: %.2f %.2f %.2f %.2f]",
                target_ai_action_[0], target_ai_action_[1], target_ai_action_[2], target_ai_action_[3], 
                target_ai_action_[4], target_ai_action_[5], target_ai_action_[6], target_ai_action_[7],
                current_safe_action_[0], current_safe_action_[1], current_safe_action_[2], current_safe_action_[3], 
                current_safe_action_[4], current_safe_action_[5], current_safe_action_[6], current_safe_action_[7]);
            
            std::fill(hw_msg.data.begin(), hw_msg.data.end(), 0.0);
        } else {
            // 【核心修复】：真实运行模式下，装载小脑计算出的安全动作，必须循环到 8！(原本是 6)
            for (size_t i = 0; i < 8; ++i) hw_msg.data[i] = current_safe_action_[i];
        }

        pub_hw_command_->publish(hw_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_ai_action_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_hw_command_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SafetyCerebellumNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}