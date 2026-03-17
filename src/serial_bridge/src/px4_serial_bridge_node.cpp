#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
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
        // 默认使用 AGX Orin 上常用的 GPIO 串口 (或 /dev/ttyUSB0)
        this->declare_parameter<std::string>("serial_port", "/dev/ttyTHS0"); 
        this->declare_parameter<int>("baud_rate", 921600); // 建议使用高波特率以保证 50Hz+ 的通讯频率

        std::string port_name = this->get_parameter("serial_port").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();

        // 2. 初始化串口
        if (!initSerialPort(port_name, baud_rate)) {
            RCLCPP_ERROR(this->get_logger(), "❌ 串口 %s 打开失败! 请检查权限或连线。", port_name.c_str());
            rclcpp::shutdown();
            return;
        }

        // 3. ROS 2 订阅器 (接收安全小脑的最终指令)
        sub_hw_cmd_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/robot/hardware_command", 
            rclcpp::SensorDataQoS(), 
            std::bind(&Px4SerialBridgeNode::commandCallback, this, std::placeholders::_1)
        );

        // 4. ROS 2 发布器 (发布轮子状态给 AI 大脑)
        pub_wheel_states_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/robot/wheel_states", 
            10
        );

        // 严格对应 AI 大脑的期望名称
        wheel_names_ = {"FL_wheel", "FR_wheel", "RL_wheel", "RR_wheel"};

        // 5. 启动后台接收子线程
        recv_thread_ = std::thread(&Px4SerialBridgeNode::receiveThreadLoop, this);

        RCLCPP_INFO(this->get_logger(), "🚀 PX4 串口桥接已启动!");
        RCLCPP_INFO(this->get_logger(), "   => 端口: %s | 波特率: %d", port_name.c_str(), baud_rate);
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
    std::vector<std::string> wheel_names_;

    // --- 串口初始化配置 ---
    bool initSerialPort(const std::string& port_name, int baud_rate) {
        serial_fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0) return false;

        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) return false;

        // 设置波特率 (这里列出最常用的两种)
        speed_t speed = (baud_rate == 921600) ? B921600 : B115200;
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        // 8N1 (8个数据位，无校验，1个停止位)，无硬件流控
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        // 纯 Raw 模式，不处理回车换行等控制字符
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        tty.c_oflag &= ~OPOST;

        // 读取阻塞设置 (VMIN = 0, VTIME = 1 表示 0.1秒超时)
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 1;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) return false;
        return true;
    }

    // --- 【下发】：ROS 2 -> PX4 (只提取前 4 个轮子指令) ---
    void commandCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() != 8) return; // 确保是完整的 8 维数据

        // 提取前 4 个数据 (索引 0, 1, 2, 3) 对应的轮速
        float payload[4];
        for (int i = 0; i < 4; ++i) {
            payload[i] = msg->data[i]; 
        }

        // 向串口写入 16 个字节的二进制流
        write(serial_fd_, payload, sizeof(payload));
    }

    // --- 【接收】：PX4 -> ROS 2 (打包并发布轮子状态) ---
    void receiveThreadLoop() {
        // 期望从 PX4 收到 4个位置 + 4个速度 = 8 个 float (32 字节)
        float recv_buffer[8]; 
        uint8_t* raw_buffer = reinterpret_cast<uint8_t*>(recv_buffer);
        int expected_bytes = sizeof(recv_buffer);
        
        while (running_) {
            int bytes_read = 0;
            // 循环读取，直到凑够 32 个字节（串口数据可能会被切片到达）
            while (bytes_read < expected_bytes && running_) {
                int n = read(serial_fd_, raw_buffer + bytes_read, expected_bytes - bytes_read);
                if (n > 0) {
                    bytes_read += n;
                }
            }

            if (bytes_read == expected_bytes) {
                auto msg = sensor_msgs::msg::JointState();
                msg.header.stamp = this->get_clock()->now();
                msg.name = wheel_names_;

                // 假设 PX4 发回来的也是前 4 个为位置，后 4 个为速度
                msg.position.assign(recv_buffer, recv_buffer + 4);
                msg.velocity.assign(recv_buffer + 4, recv_buffer + 8);
                msg.effort = std::vector<double>(4, 0.0); 

                pub_wheel_states_->publish(msg);
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