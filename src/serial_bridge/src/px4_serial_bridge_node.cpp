#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp> // 新增：用于发布 cmd_vel 话题
#include <iostream>
#include <vector>
#include <cstring>
#include <thread>
#include <atomic>

// Linux 串口头文件
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

class Px4SerialBridgeNode : public rclcpp::Node {
public:
    Px4SerialBridgeNode() : Node("px4_serial_bridge_node"), running_(true), serial_fd_(-1) {
        // 1. 声明串口参数
        this->declare_parameter<std::string>("serial_port", "/dev/ttyTHS0"); 
        this->declare_parameter<int>("baud_rate", 921600); 

        std::string port_name = this->get_parameter("serial_port").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();

        // 2. 初始化串口
        if (!initSerialPort(port_name, baud_rate)) {
            RCLCPP_ERROR(this->get_logger(), "❌ 串口 %s 打开失败! 请检查权限或连线。", port_name.c_str());
            rclcpp::shutdown();
            return;
        }

        // 3. 订阅器：接收 AI 大脑/安全小脑的最终动作指令
        sub_hw_cmd_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/robot/hardware_command", 
            rclcpp::SensorDataQoS(), 
            std::bind(&Px4SerialBridgeNode::commandCallback, this, std::placeholders::_1)
        );

        // 4. 发布器：发布轮子状态
        pub_wheel_states_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/robot/wheel_states", 10
        );

        // 5. 【新增】发布器：发布遥控器 cmd 指令
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10
        );

        // 严格对应 AI 大脑的期望名称
        wheel_names_ = {"LB_wheel", "LF_wheel", "RF_wheel", "RB_wheel"};

        // 6. 启动后台接收子线程
        recv_thread_ = std::thread(&Px4SerialBridgeNode::receiveThreadLoop, this);

        RCLCPP_INFO(this->get_logger(), "🚀 PX4 串口桥接已升级启动!");
        RCLCPP_INFO(this->get_logger(), "   => 接收格式: 4个轮速 + 2个CMD (共24字节)");
    }

    ~Px4SerialBridgeNode() {
        running_ = false;
        if (recv_thread_.joinable()) recv_thread_.join();
        if (serial_fd_ >= 0) close(serial_fd_);
    }

private:
    int serial_fd_;
    std::atomic<bool> running_;
    std::thread recv_thread_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_hw_cmd_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_wheel_states_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_; // 新增的发布器
    std::vector<std::string> wheel_names_;

    bool initSerialPort(const std::string& port_name, int baud_rate) {
        serial_fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0) return false;

        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) return false;

        speed_t speed = (baud_rate == 921600) ? B921600 : B115200;
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        tty.c_oflag &= ~OPOST;

        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 1;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) return false;
        return true;
    }

    void commandCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() != 8) return; 

        // 提取前 4 个数据 (索引 0, 1, 2, 3) 对应的轮速指令下发给 PX4
        float payload[4];
        for (int i = 0; i < 4; ++i) {
            payload[i] = msg->data[i]; 
        }

        write(serial_fd_, payload, sizeof(payload));
    }

    // --- 【修改核心】：解析新的 6-float 格式 ---
    void receiveThreadLoop() {
        // 期望从 PX4 收到: 4个轮速 + 1个线速度X + 1个角速度Z = 6 个 float (24 字节)
        float recv_buffer[6]; 
        uint8_t* raw_buffer = reinterpret_cast<uint8_t*>(recv_buffer);
        int expected_bytes = sizeof(recv_buffer);
        
        while (running_) {
            int bytes_read = 0;
            while (bytes_read < expected_bytes && running_) {
                int n = read(serial_fd_, raw_buffer + bytes_read, expected_bytes - bytes_read);
                if (n > 0) {
                    bytes_read += n;
                }
            }

            if (bytes_read == expected_bytes) {
                auto now = this->get_clock()->now();

                // 1. 打包并发布轮子状态 (JointState)
                auto wheel_msg = sensor_msgs::msg::JointState();
                wheel_msg.header.stamp = now;
                wheel_msg.name = wheel_names_;
                // 既然 PX4 不发位置了，我们给位置填充 0.0，避免 AI 节点数组越界
                wheel_msg.position = std::vector<double>(4, 0.0);
                wheel_msg.velocity.assign(recv_buffer, recv_buffer + 4);
                wheel_msg.effort = std::vector<double>(4, 0.0); 
                pub_wheel_states_->publish(wheel_msg);

                // 2. 打包并发布遥控指令 (Twist)
                auto cmd_msg = geometry_msgs::msg::Twist();
                // 假设第 5 个 float 是前进线速度 (x)，第 6 个 float 是转向角速度 (z)
                cmd_msg.linear.x = recv_buffer[4];
                cmd_msg.angular.z = recv_buffer[5];
                pub_cmd_vel_->publish(cmd_msg);
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Px4SerialBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}