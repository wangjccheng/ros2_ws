#include <rclcpp/rclcpp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_interfaces/msg/custom_msg.hpp>
#include <Eigen/Dense>
#include <mutex>
#include <cmath>
#include <algorithm> // 🚨 新增：用于对点云时间戳排序
// 新增的 TF2 依赖头文件
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

class RobustLivoxMergeNode : public rclcpp::Node {
public:
    RobustLivoxMergeNode() : Node("robust_livox_merge_node") {
        pub_merged_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("/merged_livox_cloud", 10);

        sub_mid360_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            "/livox/lidar", 10, std::bind(&RobustLivoxMergeNode::mid360Callback, this, std::placeholders::_1));
            
        sub_avia_ = this->create_subscription<livox_interfaces::msg::CustomMsg>(
            "/avia/lidar", 10, std::bind(&RobustLivoxMergeNode::aviaCallback, this, std::placeholders::_1));

        // 初始化 TF Buffer 和 Listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 默认初始化为单位矩阵，但在成功获取 TF 前，我们用 tf_ready_ 标志位来拦截错误计算
        R_avia_to_mid360_ = Eigen::Matrix3f::Identity(); 
        t_avia_to_mid360_ = Eigen::Vector3f(0.0, 0.0, 0.0); 
        tf_ready_ = false;
        
        last_mid360_time_ = this->now();

        // 请务必将这两个字符串替换为你系统中真实的 TF frame_id
        mid360_frame_id_ = "livox_frame"; 
        avia_frame_id_ = "avia_frame";

        RCLCPP_INFO(this->get_logger(), "Robust Merge Node with Dynamic TF Started.");
    }

private:
    std::mutex data_mutex_;
    livox_interfaces::msg::CustomMsg::SharedPtr latest_avia_msg_ = nullptr;
    rclcpp::Time last_mid360_time_;
    const double TOLERANCE_SEC = 0.05;

    // TF 相关成员
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string mid360_frame_id_;
    std::string avia_frame_id_;
    bool tf_ready_;
    Eigen::Matrix3f R_avia_to_mid360_;
    Eigen::Vector3f t_avia_to_mid360_;

    // 尝试更新 TF (核心容错逻辑)
    void updateTransform() {
        try {
            // 查询从 Avia 到 Mid360 的坐标变换
            // TimePointZero 表示获取最新可用的变换
            geometry_msgs::msg::TransformStamped transform_stamped = 
                tf_buffer_->lookupTransform(mid360_frame_id_, avia_frame_id_, tf2::TimePointZero);

            // 将 ROS 的 Transform 转换为 Eigen Isometry3d，然后提取 R 和 t
            Eigen::Isometry3d eigen_transform = tf2::transformToEigen(transform_stamped);
            R_avia_to_mid360_ = eigen_transform.rotation().cast<float>();
            t_avia_to_mid360_ = eigen_transform.translation().cast<float>();

            if (!tf_ready_) {
                RCLCPP_INFO(this->get_logger(), "Successfully obtained TF from %s to %s!", 
                            avia_frame_id_.c_str(), mid360_frame_id_.c_str());
                tf_ready_ = true;
            }
        } catch (const tf2::TransformException & ex) {
            if (!tf_ready_) {
                // 如果从来没拿到过 TF，报警告
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                    "Waiting for TF... Error: %s", ex.what());
            } else {
                // 如果之前拿到过，只是偶尔一次没拿到，不用慌，静默使用缓存的 R 和 t
            }
        }
    }

    void aviaCallback(const livox_interfaces::msg::CustomMsg::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_avia_msg_ = msg;

        if ((this->now() - last_mid360_time_).seconds() > 0.2) {
            publishPointcloud(nullptr, latest_avia_msg_);
        }
    }

    void mid360Callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_mid360_time_ = this->now();

        livox_interfaces::msg::CustomMsg::SharedPtr avia_to_merge = nullptr;

        if (latest_avia_msg_ != nullptr) {
            rclcpp::Time mid_time(msg->header.stamp);
            rclcpp::Time avia_time(latest_avia_msg_->header.stamp);
            
            if (std::abs((mid_time - avia_time).seconds()) < TOLERANCE_SEC) {
                avia_to_merge = latest_avia_msg_;
            }
        }

        publishPointcloud(msg, avia_to_merge);
    }
    void publishPointcloud(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg_mid360,
                           const livox_interfaces::msg::CustomMsg::SharedPtr msg_avia) {
        
        updateTransform();

        // 以 Mid360 的存在作为触发合并的核心
        if (!msg_mid360 && !msg_avia) return;

        livox_ros_driver2::msg::CustomMsg merged_msg;
        
        // 🚨 防御1：确立绝对主时间线，绝不能向后退
        uint64_t base_time = 0;
        if (msg_mid360) {
            merged_msg.header = msg_mid360->header;
            merged_msg.lidar_id = msg_mid360->lidar_id;
            base_time = msg_mid360->timebase;
        } else {
            merged_msg.header = msg_avia->header;
            merged_msg.lidar_id = msg_avia->lidar_id;
            base_time = msg_avia->timebase;
        }
        merged_msg.timebase = base_time;

        // ==========================================
        // 处理 Mid360 点云
        // ==========================================
        if (msg_mid360) {
            for (const auto& pt : msg_mid360->points) {
                // 🚨 防御2：空间电子围栏 (阻断 VoxelGrid 爆炸)
                if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
                float dist2 = pt.x*pt.x + pt.y*pt.y + pt.z*pt.z;
                if (dist2 < 0.1 || dist2 > 10000.0) continue; // 丢弃极近噪点和 >100m 外的离谱点

                livox_ros_driver2::msg::CustomPoint new_pt = pt;
                new_pt.offset_time = pt.offset_time;
                merged_msg.points.push_back(new_pt);
            }
        }

        // ==========================================
        // 处理 Avia 点云并做外参变换
        // ==========================================
        if (msg_avia && tf_ready_) {
            for (const auto& avia_pt : msg_avia->points) {
                // 计算 Avia 点的绝对物理时间
                uint64_t absolute_time = msg_avia->timebase + avia_pt.offset_time;
                
                // 🚨 防御3：时间硬截断！(解决飞点和崩溃的核心)
                // 绝对不允许任何一个点的时间早于当前合并帧的 base_time
                if (absolute_time < base_time) continue; 
                
                // 同时，不允许点超过正常一帧的时间跨度 (120ms宽限)
                uint64_t offset = absolute_time - base_time;
                if (offset > 120000000ULL) continue; 

                // 🚨 防御4：过滤 Avia 的内部玻璃罩反射噪点 (必须在乘外参之前过滤！)
                if (!std::isfinite(avia_pt.x) || !std::isfinite(avia_pt.y) || !std::isfinite(avia_pt.z)) continue;
                float dist2 = avia_pt.x*avia_pt.x + avia_pt.y*avia_pt.y + avia_pt.z*avia_pt.z;
                // Avia 的内部反光较严重，在此直接过滤掉距离传感器 0.5m 内的所有点
                if (dist2 < 0.25 || dist2 > 10000.0) continue; 

                livox_ros_driver2::msg::CustomPoint new_pt;
                new_pt.offset_time = offset;

                Eigen::Vector3f pt_in_avia(avia_pt.x, avia_pt.y, avia_pt.z);
                Eigen::Vector3f pt_in_mid360 = R_avia_to_mid360_ * pt_in_avia + t_avia_to_mid360_;

                new_pt.x = pt_in_mid360.x();
                new_pt.y = pt_in_mid360.y();
                new_pt.z = pt_in_mid360.z();
                new_pt.reflectivity = avia_pt.reflectivity;
                new_pt.tag = avia_pt.tag;
                new_pt.line = avia_pt.line;

                merged_msg.points.push_back(new_pt);
            }
        }

        // 🚨 防御5：全局严格排序 (保证时间轴纯粹单调递增)
        if (!merged_msg.points.empty()) {
            std::sort(merged_msg.points.begin(), merged_msg.points.end(),
                      [](const livox_ros_driver2::msg::CustomPoint& a, const livox_ros_driver2::msg::CustomPoint& b) {
                          return a.offset_time < b.offset_time;
                      });
        }

        merged_msg.point_num = merged_msg.points.size();
        if (merged_msg.point_num > 0) {
            pub_merged_->publish(merged_msg);
        }
    }
/*
    void publishPointcloud(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg_mid360,
                           const livox_interfaces::msg::CustomMsg::SharedPtr msg_avia) {
        
        // 每次发布前尝试更新一下外参缓存
        updateTransform();

        livox_ros_driver2::msg::CustomMsg merged_msg;
        uint64_t safe_timebase = 0;
        
        bool use_mid360 = msg_mid360 != nullptr;
        bool use_avia = msg_avia != nullptr;

        // ==========================================
        // 🚨 新增：PTP 时钟防爆盾 🚨
        // ==========================================
        if (use_mid360 && use_avia) {
            // 计算两个雷达 timebase 的绝对差值
            uint64_t time_diff = (msg_mid360->timebase > msg_avia->timebase) ? 
                                 (msg_mid360->timebase - msg_avia->timebase) : 
                                 (msg_avia->timebase - msg_mid360->timebase);
            
            // 如果时间基准相差超过 0.1 秒 (100,000,000 纳秒)
            // 说明至少有一个雷达没有 PTP 锁，强行融合会导致 offset_time 溢出
            if (time_diff > 100000000ULL) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                    "[WARNING] TIMEBASE MISMATCH! Mid360: %lu, Avia: %lu. Dropping Avia to protect FAST-LIO.", 
                    msg_mid360->timebase, msg_avia->timebase);
                
                // 强制切断未同步的副雷达，降级为单雷达模式
                use_avia = false; 
            }
        }
        // ==========================================

        if (use_mid360 && use_avia) {
            merged_msg.header = msg_mid360->header;
            merged_msg.lidar_id = msg_mid360->lidar_id;
            safe_timebase = std::min(msg_mid360->timebase, msg_avia->timebase);
        } else if (use_mid360) {
            merged_msg.header = msg_mid360->header;
            merged_msg.lidar_id = msg_mid360->lidar_id;
            safe_timebase = msg_mid360->timebase;
        } else if (use_avia) {
            merged_msg.header = msg_avia->header;
            merged_msg.lidar_id = msg_avia->lidar_id;
            safe_timebase = msg_avia->timebase;
        } else {
            return;
        }
        
        merged_msg.timebase = safe_timebase;

        // ==========================================
        // 处理 Mid360 点云（增加脏点过滤）
        // ==========================================
        if (msg_mid360) {
            for (const auto& pt : msg_mid360->points) {
                // 🚨 拦截 NaN 和 Inf
                if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
                // 🚨 拦截距离雷达中心极近的失效原点 (小于 10cm)
                if (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z < 0.01) continue;

                livox_ros_driver2::msg::CustomPoint new_pt = pt;
                new_pt.offset_time = (msg_mid360->timebase + pt.offset_time) - safe_timebase;
                merged_msg.points.push_back(new_pt);
            }
        }

        // ==========================================
        // 处理 Avia 点云并做外参变换（增加脏点过滤）
        // ==========================================
        if (msg_avia) {
            if (!tf_ready_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                    "Skipping Avia merge because TF is not ready yet.");
            } else {
                for (const auto& avia_pt : msg_avia->points) {
                    // 🚨 必须在外参变换前拦截！否则 (0,0,0) 会被平移成正常坐标
                    if (!std::isfinite(avia_pt.x) || !std::isfinite(avia_pt.y) || !std::isfinite(avia_pt.z)) continue;
                    if (avia_pt.x * avia_pt.x + avia_pt.y * avia_pt.y + avia_pt.z * avia_pt.z < 0.01) continue;

                    livox_ros_driver2::msg::CustomPoint new_pt;
                    new_pt.offset_time = (msg_avia->timebase + avia_pt.offset_time) - safe_timebase;

                    Eigen::Vector3f pt_in_avia(avia_pt.x, avia_pt.y, avia_pt.z);
                    Eigen::Vector3f pt_in_mid360 = R_avia_to_mid360_ * pt_in_avia + t_avia_to_mid360_;

                    new_pt.x = pt_in_mid360.x();
                    new_pt.y = pt_in_mid360.y();
                    new_pt.z = pt_in_mid360.z();
                    
                    new_pt.reflectivity = avia_pt.reflectivity;
                    new_pt.tag = avia_pt.tag;
                    new_pt.line = avia_pt.line;

                    merged_msg.points.push_back(new_pt);
                }
            }
        }

        merged_msg.point_num = merged_msg.points.size();

        // ... (上面处理 msg_mid360 和 msg_avia 的循环代码保持原样) ...
        
        // 🚨 新增 1：远距离和近距离脏点强化过滤
        // 在上面的遍历代码中，除了拦截 NaN 和 <0.01，再加上最大距离限制：
        // if (pt.x*pt.x + pt.y*pt.y + pt.z*pt.z > 10000.0) continue; // 超过100米直接丢弃

        // 🚨 新增 2：全局时间戳严格排序 (解决时间倒流导致的飞点)
        if (!merged_msg.points.empty()) {
            std::sort(merged_msg.points.begin(), merged_msg.points.end(),
                      [](const livox_ros_driver2::msg::CustomPoint& a, const livox_ros_driver2::msg::CustomPoint& b) {
                          return a.offset_time < b.offset_time;
                      });
        }

        merged_msg.point_num = merged_msg.points.size();
        

        // 如果因为 TF 没好导致里面一个点都没有，就别发了，避免 FAST-LIO 收到空包
        if (merged_msg.point_num > 0) {
            pub_merged_->publish(merged_msg);
        }
        
    }*/

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_mid360_;
    rclcpp::Subscription<livox_interfaces::msg::CustomMsg>::SharedPtr sub_avia_;
    rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub_merged_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<RobustLivoxMergeNode>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}