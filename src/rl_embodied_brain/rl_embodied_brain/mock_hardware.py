import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray
import numpy as np

class MockHardwareNode(Node):
    def __init__(self):
        super().__init__('mock_hardware_node')
        # 建立假冒的传感器发布器
        self.pub_imu = self.create_publisher(Imu, '/livox/imu', 1)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 1)
        self.pub_odom = self.create_publisher(Odometry, '/Odometry', 1)
        
        # 【核心修改 1】：把关节发布器拆分为腿部和轮子两个
        self.pub_legs = self.create_publisher(JointState, '/robot/joint_states', 1)
        self.pub_wheels = self.create_publisher(JointState, '/robot/wheel_states', 1)
        
        # 注意：这里的话题名与你 policy_inference_node 中的订阅保持一致
        self.pub_map = self.create_publisher(GridMap, '/elevation_mapping_node/elevation_map_raw', 1)

        # 以 50Hz (0.02秒) 的极高频率疯狂发送假数据
        self.timer = self.create_timer(0.02, self.publish_mock_data)
        self.get_logger().info("🚧 硬件欺骗节点已升级！正在向 AI 大脑注入拆分后的腿部和轮子虚拟数据...")

    def publish_mock_data(self):
        now = self.get_clock().now().to_msg()

        # 1. 伪造 IMU (绝对水平，没有任何旋转和角速度)
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.orientation.w = 1.0 
        self.pub_imu.publish(imu_msg)

        # 2. 伪造手柄指令 (无输入，期望线速度和角速度全为 0)
        self.pub_cmd.publish(Twist())

        # 3. 伪造里程计 (假设机器人在架子上，底盘中心距离地面 0.3 米)
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.pose.pose.position.z = 0.3 
        self.pub_odom.publish(odom_msg)

        # 4. 【核心修改 2】：单独伪造腿部状态 (4条 EHA 腿)
        leg_msg = JointState()
        leg_msg.header.stamp = now
        leg_msg.name = ['FL_leg', 'FR_leg', 'RL_leg', 'RR_leg']
        leg_msg.position = [0.0] * 4
        leg_msg.velocity = [0.0] * 4
        self.pub_legs.publish(leg_msg)

        # 5. 【核心修改 3】：单独伪造轮子状态 (4个轮子)
        wheel_msg = JointState()
        wheel_msg.header.stamp = now
        wheel_msg.name = ['FL_wheel', 'FR_wheel', 'RL_wheel', 'RR_wheel']
        # 为了防止 JointState 数组长度不对齐导致的 IndexError，把 position 和 velocity 都填满 4 个 0
        wheel_msg.position = [0.0] * 4
        wheel_msg.velocity = [0.0] * 4
        self.pub_wheels.publish(wheel_msg)

        # 6. 伪造 GPU 高程图 (一张绝对平坦的地毯)
        map_msg = GridMap()
        map_msg.info.header.stamp = now
        map_msg.layers = ['elevation']
        layer_data = Float32MultiArray()
        # 发送 625 个点 (25x25) 的平坦地形，高度与里程计底盘一致 (0.3m)
        layer_data.data = [0.3] * 625  
        map_msg.data = [layer_data]
        self.pub_map.publish(map_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockHardwareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()