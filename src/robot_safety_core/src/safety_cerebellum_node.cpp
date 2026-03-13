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
        // 【新增】：声明干跑模式参数，默认设为 true（开启诊断锁死）
        this->declare_parameter<bool>("dry_run_mode", true);
        // --- 1. 参数初始化 ---
        // 150kg 机体惯性极大，限制单次循环(0.02s)内的最大动作变化量
        // 前两个为轮速变化率，后四个为 EHA 腿部位姿变化率
        max_action_delta_ = {0.15, 0.15, 0.05, 0.05, 0.05, 0.05}; 
        current_safe_action_.resize(6, 0.0);
        
        e_stop_active_ = false;
        proj_grav_z_ = -1.0; // 默认平稳停放
        cmd_is_zero_ = true;

        // --- 2. 订阅器 ---
        // 订阅 AI 大脑输出的原始动作
        sub_ai_action_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/robot/action_command_raw", 10, 
            std::bind(&SafetyCerebellumNode::aiActionCallback, this, std::placeholders::_1));

        // 订阅硬件 IMU (用于倾覆保护)
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", 10,
            std::bind(&SafetyCerebellumNode::imuCallback, this, std::placeholders::_1));

        // 订阅遥控器/手柄指令 (用于无输入死区和急停)
        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&SafetyCerebellumNode::cmdVelCallback, this, std::placeholders::_1));

        // 订阅实体遥控器的物理急停开关 (例如通过 Joy 话题)
        sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&SafetyCerebellumNode::joyCallback, this, std::placeholders::_1));

        // --- 3. 发布器 ---
        // 发布经过安全过滤的最终硬件指令 (给到 EHA 和电机驱动)
        pub_hw_command_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/robot/hardware_command", 10);

        // --- 4. 看门狗与控制循环 (50Hz) ---
        last_ai_time_ = this->get_clock()->now();
        last_imu_time_ = this->get_clock()->now();
        timer_ = this->create_wall_timer(20ms, std::bind(&SafetyCerebellumNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "🛡️ 硬件安全小脑已启动！守护神已就位。");
    }

private:
    // --- 状态缓存 ---
    std::vector<double> max_action_delta_;
    std::vector<double> current_safe_action_;
    std::vector<double> target_ai_action_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    bool e_stop_active_;
    double proj_grav_z_;
    bool cmd_is_zero_;

    rclcpp::Time last_ai_time_;
    rclcpp::Time last_imu_time_;

    // --- 回调函数 ---
    void aiActionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() == 6) {
            for(size_t i=0; i<6; ++i) target_ai_action_[i] = msg->data[i];
            last_ai_time_ = this->get_clock()->now();
        }
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // 计算重力向量在机体坐标系下的投影 (Z轴分量)
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        // 简化的投影计算：正常平放时 Z 应该是 -1.0 左右
        proj_grav_z_ = -cos(roll) * cos(pitch); 
        last_imu_time_ = this->get_clock()->now();
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // 判断是否完全没有输入 (死区判定)
        if (std::abs(msg->linear.x) < 0.01 && std::abs(msg->angular.z) < 0.01) {
            cmd_is_zero_ = true;
        } else {
            cmd_is_zero_ = false;
        }
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // 假设手柄的第 7 号按键是物理急停开关
        if (msg->buttons.size() > 7 && msg->buttons[7] == 1) {
            if (!e_stop_active_) {
                RCLCPP_ERROR(this->get_logger(), "🛑 触发硬件级物理急停！切断所有动力！");
                e_stop_active_ = true;
            }
        }
    }

    // --- 核心安全防线 ---
    void controlLoop() {
        std_msgs::msg::Float32MultiArray hw_msg;
        hw_msg.data.resize(6, 0.0);

        // 【防线 1】：最高优先级 - 硬件急停 (E-Stop)
        if (e_stop_active_) {
            // 输出全 0，或者下发给 EHA 的特定抱死状态指令
            pub_hw_command_->publish(hw_msg);
            return; 
        }

        // 【防线 2】：看门狗 (Watchdog) - AI 节点崩溃或 IMU 掉线
        auto now = this->get_clock()->now();
        if ((now - last_ai_time_).seconds() > 0.1 || (now - last_imu_time_).seconds() > 0.1) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "⚠️ 看门狗触发！传感器或 AI 指令超时，强制停车！");
            pub_hw_command_->publish(hw_msg); // 强制停车
            return;
        }

        // 【防线 3】：倾覆保护 (Rollover Prevention)
        // 超过约 45 度 (cos(45) ≈ 0.707) 则判定为即将翻车
        if (proj_grav_z_ > -0.7) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "🚨 姿态异常！触发倾覆保护！");
            // 这里根据你的 EHA 特性，可以让腿部缓慢收缩到最低安全位置，这里演示全部切断
            pub_hw_command_->publish(hw_msg); 
            return;
        }

        // 【防线 4】：无输入死区屏蔽 (Zero-Command Deadband)
        if (cmd_is_zero_) {
            // 当没有摇杆输入时，强制剥夺 AI 对轮子前进/转向的控制权，避免漂移
            target_ai_action_[0] = 0.0; // 轮子线速度目标置 0
            target_ai_action_[1] = 0.0; // 轮子角速度目标置 0
            // 注：此处不强制清零 target_ai_action_[2~5]，保留 EHA 维持站立的位姿调整
        }

        // 【防线 5】：动作变化率限幅 (Slew Rate Limiting)
        // 极其关键：防止高频抖动打坏减速箱和产生液压水锤效应
        for (size_t i = 0; i < 6; ++i) {
            double delta = target_ai_action_[i] - current_safe_action_[i];
            
            // 限制单次增量/减量
            if (delta > max_action_delta_[i]) {
                current_safe_action_[i] += max_action_delta_[i];
            } else if (delta < -max_action_delta_[i]) {
                current_safe_action_[i] -= max_action_delta_[i];
            } else {
                current_safe_action_[i] = target_ai_action_[i];
            }
            
            // 最终硬件物理限幅（根据你底层实际的物理量纲设定）
            // 例如轮速不能超过某绝对值，EHA 行程不能超过某极限
            //hw_msg.data[i] = current_safe_action_[i]; 
        }
        // 【新增：防线 6 - 诊断模式/干跑模式】
        bool is_dry_run = this->get_parameter("dry_run_mode").as_bool();

        if (is_dry_run) {
            // 每秒打印一次对比信息，防止刷屏
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "🩺 [诊断锁死中] AI原始指令: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] | 小脑限幅后试图输出: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                target_ai_action_[0], target_ai_action_[1], target_ai_action_[2], target_ai_action_[3], target_ai_action_[4], target_ai_action_[5],
                current_safe_action_[0], current_safe_action_[1], current_safe_action_[2], current_safe_action_[3], current_safe_action_[4], current_safe_action_[5]);
            
            // 强行将实际下发给硬件的指令覆写为绝对安全的 0
            std::fill(hw_msg.data.begin(), hw_msg.data.end(), 0.0);
        } else {
            // 真实运行模式：装载小脑计算出的安全动作
            for (size_t i = 0; i < 6; ++i) hw_msg.data[i] = current_safe_action_[i];
        }

        // 历经所有防线，下发安全指令
        pub_hw_command_->publish(hw_msg);
    }

    // --- ROS 2 句柄声明 ---
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
    
    // 设置高优先级实时调度 (在 Jetson Orin 上建议配合 PREEMPT_RT 内核使用)
    // sched_param param;
    // param.sched_priority = 80;
    // sched_setscheduler(0, SCHED_FIFO, &param);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}