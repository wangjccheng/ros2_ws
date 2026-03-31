import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import csv
import math
import time

class MultiStreamLogger(Node):
    def __init__(self):
        super().__init__('multi_stream_logger')
        
        self.is_recording = False
        self.csv_file = None
        self.csv_writer = None
        
        # 数据缓存字典：新增了 imu_w_x, imu_w_y, imu_w_z
        self.data_cache = {
            'ekf_v_x': 0.0, 'ekf_w_z': 0.0,
            'lio_v_x': 0.0, 'lio_v_y': 0.0, 'lio_w_z': 0.0,
            'imu_roll': 0.0, 'imu_pitch': 0.0, 'imu_z_jerk': 0.0,
            'imu_w_x': 0.0, 'imu_w_y': 0.0, 'imu_w_z': 0.0
        }
        
        self.last_z_accel = 0.0
        self.last_imu_time = None
        
        # 1. 订阅三个数据源
        self.ekf_sub = self.create_subscription(Odometry, '/odometry/filtered', self.ekf_callback, 10)
        self.lio_sub = self.create_subscription(Odometry, '/Odometry', self.lio_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/livox/imu_filtered', self.imu_callback, 10)
        
        # 2. 启停服务
        self.srv = self.create_service(SetBool, '/test/logger', self.toggle_logging_callback)
        
        # 3. 创建定时器，以 50Hz (0.02s) 的频率将缓存数据写入 CSV
        self.record_timer = self.create_timer(0.02, self.timer_callback)
        
        self.get_logger().info("Logger Ready. Call /test/logger to start/stop.")

    def ekf_callback(self, msg):
        self.data_cache['ekf_v_x'] = msg.twist.twist.linear.x
        self.data_cache['ekf_w_z'] = msg.twist.twist.angular.z

    def lio_callback(self, msg):
        self.data_cache['lio_v_x'] = msg.twist.twist.linear.x
        self.data_cache['lio_v_y'] = msg.twist.twist.linear.y
        self.data_cache['lio_w_z'] = msg.twist.twist.angular.z

    def imu_callback(self, msg):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # 提取并缓存原始角速度
        self.data_cache['imu_w_x'] = msg.angular_velocity.x
        self.data_cache['imu_w_y'] = msg.angular_velocity.y
        self.data_cache['imu_w_z'] = msg.angular_velocity.z
        
        # 提取姿态四元数计算 Roll 和 Pitch
        q = msg.orientation
        self.data_cache['imu_roll'] = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x**2 + q.y**2))
        sinp = 2 * (q.w * q.y - q.z * q.x)
        self.data_cache['imu_pitch'] = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

        # 计算 Z轴 Jerk
        current_z_accel = msg.linear_acceleration.z
        if self.last_imu_time is not None:
            dt = current_time - self.last_imu_time
            if dt > 0:
                self.data_cache['imu_z_jerk'] = (current_z_accel - self.last_z_accel) / dt
                
        self.last_z_accel = current_z_accel
        self.last_imu_time = current_time

    def timer_callback(self):
        if self.is_recording and self.csv_writer:
            self.csv_writer.writerow([
                f"{time.time():.4f}",
                f"{self.data_cache['ekf_v_x']:.4f}",
                f"{self.data_cache['ekf_w_z']:.4f}",
                f"{self.data_cache['lio_v_x']:.4f}",
                f"{self.data_cache['lio_v_y']:.4f}",
                f"{self.data_cache['lio_w_z']:.4f}",
                f"{self.data_cache['imu_roll']:.4f}",
                f"{self.data_cache['imu_pitch']:.4f}",
                f"{self.data_cache['imu_z_jerk']:.4f}",
                f"{self.data_cache['imu_w_x']:.4f}",   # 写入 IMU 原始角速度 X
                f"{self.data_cache['imu_w_y']:.4f}",   # 写入 IMU 原始角速度 Y
                f"{self.data_cache['imu_w_z']:.4f}"    # 写入 IMU 原始角速度 Z
            ])

    def toggle_logging_callback(self, request, response):
        if request.data and not self.is_recording:
            filename = f"speed_comparison_{int(time.time())}.csv"
            self.csv_file = open(filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            
            # 更新表头，加入 IMU 原始角速度的三列
            self.csv_writer.writerow([
                'Timestamp', 'EKF_Vx', 'EKF_Wz', 
                'LIO_Vx', 'LIO_Vy', 'LIO_Wz', 
                'Roll', 'Pitch', 'Z_Jerk', 
                'IMU_Wx', 'IMU_Wy', 'IMU_Wz'
            ])
            self.is_recording = True
            response.message = f"Started recording to {filename}"
        elif not request.data and self.is_recording:
            self.is_recording = False
            if self.csv_file:
                self.csv_file.close()
            response.message = "Stopped recording."
            
        response.success = True
        self.get_logger().info(response.message)
        return response

def main():
    rclpy.init()
    node = MultiStreamLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()