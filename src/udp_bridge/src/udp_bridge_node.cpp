#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp> 
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
        this->declare_parameter<std::string>("target_ip", "192.168.2.20"); 
        this->declare_parameter<int>("target_port", 25000);                
        this->declare_parameter<int>("local_port", 25001);                 

        std::string target_ip = this->get_parameter("target_ip").as_string();
        int target_port = this->get_parameter("target_port").as_int();
        int local_port = this->get_parameter("local_port").as_int();

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

        memset(&target_addr_, 0, sizeof(target_addr_));
        target_addr_.sin_family = AF_INET;
        target_addr_.sin_port = htons(target_port);
        target_addr_.sin_addr.s_addr = inet_addr(target_ip.c_str());

        sub_hw_cmd_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/robot/hardware_command", 
            rclcpp::SensorDataQoS(), 
            std::bind(&UdpBridgeNode::commandCallback, this, std::placeholders::_1)
        );

        // 【修改 1】：专门发布腿部状态到 /robot/joint_states
        pub_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/robot/joint_states", 
            10
        );

        // 【修改 2】：只保留腿部的 4 个关节名称
        joint_names_ = {"FL_leg", "FR_leg", "RL_leg", "RR_leg"};

        recv_thread_ = std::thread(&UdpBridgeNode::receiveThreadLoop, this);

        RCLCPP_INFO(this->get_logger(), "🚀 双向 UDP 桥接已启动 (仅转发腿部 EHA 数据)!");
    }

    ~UdpBridgeNode() {
        running_ = false;
        if (recv_thread_.joinable()) recv_thread_.join();
        if (sock_fd_ >= 0) close(sock_fd_);
    }

private:
    int sock_fd_;
    struct sockaddr_in target_addr_;
    std::atomic<bool> running_;
    std::thread recv_thread_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_hw_cmd_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
    std::vector<std::string> joint_names_;

    // --- 【下发】：ROS 2 -> Speedgoat (只剥离腿部发过去) ---
    void commandCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        // 【核心修改 3】：现在总指令是 8 维 (前 4 是轮子，后 4 是腿)
        if (msg->data.size() != 8) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "⚠️ hardware_command 维度不对！预期 8，实际 %zu", msg->data.size());
            return;
        }

        // 我们只提取后 4 个数据 (索引 4, 5, 6, 7) 发给 Speedgoat
        float payload[4];
        for (int i = 0; i < 4; ++i) {
            payload[i] = msg->data[i + 4]; 
        }

        // 发送 16 字节 (4 个 float)
        sendto(sock_fd_, payload, sizeof(payload), 0,
               (struct sockaddr*)&target_addr_, sizeof(target_addr_));
    }

    // --- 【接收】：Speedgoat -> ROS 2 (只处理腿部状态) ---
    void receiveThreadLoop() {
        // 【核心修改 4】：只期望收到 4位置 + 4速度 = 8 个 float (32 字节)
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

                // 前 4 个 float 是位置 (Position)
                msg.position.assign(recv_buffer, recv_buffer + 4);
                
                // 后 4 个 float 是速度 (Velocity)
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