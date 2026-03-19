#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp> 
#include <sensor_msgs/msg/imu.hpp> // 【新增 1】：引入 IMU 消息头文件
#include <iostream>
#include <vector>
#include <cstring>
#include <thread>
#include <atomic>

// Linux Socket 头文件
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

class UdpBridgeNode : public rclcpp::Node {
public:
    UdpBridgeNode() : Node("udp_bridge_node"), running_(true) {
        this->declare_parameter<std::string>("target_ip", "192.168.1.5"); 
        this->declare_parameter<int>("target_port", 25000);                
        this->declare_parameter<int>("local_port", 25001);                 
        this->declare_parameter<int>("target_imu_port", 1024); // 【新增 2】：用于发送 IMU 数据的 Speedgoat 新端口

        std::string target_ip = this->get_parameter("target_ip").as_string();
        int target_port = this->get_parameter("target_port").as_int();
        int local_port = this->get_parameter("local_port").as_int();
        int target_imu_port = this->get_parameter("target_imu_port").as_int();

        sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ UDP Socket 创建失败!");
            rclcpp::shutdown();
            return;
        }

        struct sockaddr_in local_addr;
        memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_family = AF_INET;
        local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        local_addr.sin_port = htons(local_port);

        if (bind(sock_fd_, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ UDP 端口 %d 绑定失败，可能被占用!", local_port);
            close(sock_fd_);
            rclcpp::shutdown();
            return;
        }

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 100000; 
        setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        // 目标硬件指令地址 (原有的)
        memset(&target_addr_, 0, sizeof(target_addr_));
        target_addr_.sin_family = AF_INET;
        target_addr_.sin_port = htons(target_port);
        target_addr_.sin_addr.s_addr = inet_addr(target_ip.c_str());

        // 【新增 3】：目标 IMU 数据地址
        memset(&target_imu_addr_, 0, sizeof(target_imu_addr_));
        target_imu_addr_.sin_family = AF_INET;
        target_imu_addr_.sin_port = htons(target_imu_port);
        target_imu_addr_.sin_addr.s_addr = inet_addr(target_ip.c_str());

        // 订阅硬件指令
        sub_hw_cmd_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/robot/hardware_command", 
            rclcpp::SensorDataQoS(), 
            std::bind(&UdpBridgeNode::commandCallback, this, std::placeholders::_1)
        );

        // 【新增 4】：订阅 IMU 数据 (请确认实际的 IMU 话题名称，这里默认使用 /imu/data)
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", 
            10, 
            std::bind(&UdpBridgeNode::imuCallback, this, std::placeholders::_1)
        );

        pub_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/robot/joint_states", 
            10
        );

        joint_names_ = {"LB_leg", "LF_leg", "RF_leg", "RB_leg"};

        recv_thread_ = std::thread(&UdpBridgeNode::receiveThreadLoop, this);

        RCLCPP_INFO(this->get_logger(), "🚀 双向 UDP 桥接已启动 (支持硬件指令转发 & IMU直传)!");
    }

    ~UdpBridgeNode() {
        running_ = false;
        if (recv_thread_.joinable()) recv_thread_.join();
        if (sock_fd_ >= 0) close(sock_fd_);
    }

private:
    int sock_fd_;
    struct sockaddr_in target_addr_;
    struct sockaddr_in target_imu_addr_; // 【新增变量】：IMU专属目标地址
    std::atomic<bool> running_;
    std::thread recv_thread_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_hw_cmd_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_; // 【新增变量】：IMU订阅者
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
    std::vector<std::string> joint_names_;

    // --- 【新增 5】：处理 IMU 数据并直接发送给 Speedgoat ---
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // 将 IMU 数据打包为 float 数组。
        // 这里提供 10 个 float 的格式 (40 字节)：
        // [0-3]: 四元数方向 (x, y, z, w)
        // [4-6]: 角速度 (x, y, z)
        // [7-9]: 线加速度 (x, y, z)
        float payload[10];
        
        payload[0] = static_cast<float>(msg->orientation.x);
        payload[1] = static_cast<float>(msg->orientation.y);
        payload[2] = static_cast<float>(msg->orientation.z);
        payload[3] = static_cast<float>(msg->orientation.w);
        
        payload[4] = static_cast<float>(msg->angular_velocity.x);
        payload[5] = static_cast<float>(msg->angular_velocity.y);
        payload[6] = static_cast<float>(msg->angular_velocity.z);
        
        payload[7] = static_cast<float>(msg->linear_acceleration.x);
        payload[8] = static_cast<float>(msg->linear_acceleration.y);
        payload[9] = static_cast<float>(msg->linear_acceleration.z);

        // 发送 40 字节给 Speedgoat 的专属 IMU 端口
        sendto(sock_fd_, payload, sizeof(payload), 0,
               (struct sockaddr*)&target_imu_addr_, sizeof(target_imu_addr_));
    }

    // --- 【下发】：ROS 2 -> Speedgoat (只剥离腿部发过去) ---
    void commandCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() != 8) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "⚠️ hardware_command 维度不对！预期 8，实际 %zu", msg->data.size());
            return;
        }

        float payload[4];
        for (int i = 0; i < 4; ++i) {
            payload[i] = msg->data[i + 4]; 
        }

        sendto(sock_fd_, payload, sizeof(payload), 0,
               (struct sockaddr*)&target_addr_, sizeof(target_addr_));
    }

    // --- 【接收】：Speedgoat -> ROS 2 (只处理腿部状态) ---
    void receiveThreadLoop() {
        float recv_buffer[8]; 
        struct sockaddr_in sender_addr;
        socklen_t sender_len = sizeof(sender_addr);

        while (running_) {
            ssize_t bytes_received = recvfrom(
                sock_fd_, recv_buffer, sizeof(recv_buffer), 0,
                (struct sockaddr*)&sender_addr, &sender_len
            );

            if (bytes_received == sizeof(recv_buffer)) {
                auto msg = sensor_msgs::msg::JointState();
                msg.header.stamp = this->get_clock()->now();
                msg.name = joint_names_;

                msg.position.assign(recv_buffer, recv_buffer + 4);
                msg.velocity.assign(recv_buffer + 4, recv_buffer + 8);
                msg.effort = std::vector<double>(4, 0.0); 

                pub_joint_states_->publish(msg);
            } 
            else if (bytes_received > 0) {
                RCLCPP_WARN(this->get_logger(), "⚠️ 收到残缺 UDP 数据包: %zd 字节 (预期 %zu)", 
                            bytes_received, sizeof(recv_buffer));
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UdpBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}