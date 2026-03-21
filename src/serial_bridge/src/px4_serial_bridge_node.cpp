#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <vector>
#include <cstring>
#include <thread>
#include <atomic>

// Linux 串口头文件
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// 引入 MAVLink V2.0 官方头文件库
#include <mavlink/v2.0/common/mavlink.h>

class Px4SerialBridgeNode : public rclcpp::Node {
public:
    Px4SerialBridgeNode() : Node("px4_serial_bridge_node"), running_(true), serial_fd_(-1) {
        // 依然保留 USB 虚拟串口和波特率配置
        this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0"); 
        this->declare_parameter<int>("baud_rate", 115200); 

        std::string port_name = this->get_parameter("serial_port").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();

        if (!initSerialPort(port_name, baud_rate)) {
            RCLCPP_ERROR(this->get_logger(), "❌ USB 串口 %s 打开失败!", port_name.c_str());
            rclcpp::shutdown();
            return;
        }

        sub_hw_cmd_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/robot/hardware_command", 
            10, 
            std::bind(&Px4SerialBridgeNode::commandCallback, this, std::placeholders::_1)
        );

        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10
        );

        recv_thread_ = std::thread(&Px4SerialBridgeNode::receiveThreadLoop, this);

        RCLCPP_INFO(this->get_logger(), "🛸 MAVLink 协议桥接已启动!");
        RCLCPP_INFO(this->get_logger(), "   => 监听端口: %s (115200)", port_name.c_str());
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
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

    bool initSerialPort(const std::string& port_name, int baud_rate) {
        serial_fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0) return false;

        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) return false;

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

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

    // --- 【核心修改：使用 MAVLink 打包发送】 ---
    void commandCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() != 8) return; 

        // 提取 4 个轮速指令
        float controls[8] = {0.0f}; // MAVLink 的 actuator_control 数组固定为 8 个
        controls[0] = msg->data[0]; // LF
        controls[1] = msg->data[1]; // RF
        controls[2] = msg->data[2]; // LR
        controls[3] = msg->data[3]; // RR

        mavlink_message_t mav_msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

        // 构建标准的电机控制消息 (SET_ACTUATOR_CONTROL_TARGET, msgid = 139)
        mavlink_msg_set_actuator_control_target_pack(
            1,                              // System ID (代表 Jetson AGX)
            MAV_COMP_ID_ONBOARD_COMPUTER,   // Component ID
            &mav_msg,
            this->get_clock()->now().nanoseconds() / 1000, // 时间戳 (微秒)
            0,                              // group_mlx (控制组 0)
            1,                              // target_system (PX4 默认是 1)
            1,                              // target_component (飞控组件默认 1)
            controls                        // 8 个浮点数的数组
        );

        // 序列化为字节流并发送
        uint16_t len = mavlink_msg_to_send_buffer(buffer, &mav_msg);
        write(serial_fd_, buffer, len);
    }

    // --- 【核心修改：使用 MAVLink 自动解包】 ---
    void receiveThreadLoop() {
        mavlink_message_t mav_msg;
        mavlink_status_t status;
        
        while (running_) {
            uint8_t byte;
            // 逐字节读取
            if (read(serial_fd_, &byte, 1) > 0) {
                // 强大的 MAVLink 官方解析函数：自动寻头、算长度、校验 CRC
                if (mavlink_parse_char(MAVLINK_COMM_0, byte, &mav_msg, &status)) {
                    
                    // 假设 PX4 端使用 DEBUG_FLOAT_ARRAY 消息 (msgid = 350) 将线速度和角速度传回来
                    if (mav_msg.msgid == MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY) {
                        mavlink_debug_float_array_t debug_msg;
                        mavlink_msg_debug_float_array_decode(&mav_msg, &debug_msg);

                        // 核对数组的 ID，确保这是我们自定义的速度反馈
                        if (debug_msg.array_id == 100) { 
                            auto cmd_msg = geometry_msgs::msg::Twist();
                            cmd_msg.linear.x = debug_msg.data[0];  // 线速度
                            cmd_msg.angular.z = debug_msg.data[1]; // 角速度
                            pub_cmd_vel_->publish(cmd_msg);
                        }
                    }
                    // 或者监听标准的里程计反馈 (msgid = 331) 作为备用选择
                    else if (mav_msg.msgid == MAVLINK_MSG_ID_ODOMETRY) {
                        mavlink_odometry_t odom;
                        mavlink_msg_odometry_decode(&mav_msg, &odom);

                        auto cmd_msg = geometry_msgs::msg::Twist();
                        cmd_msg.linear.x = odom.vx;
                        cmd_msg.angular.z = odom.yawspeed;
                        pub_cmd_vel_->publish(cmd_msg);
                    }
                }
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