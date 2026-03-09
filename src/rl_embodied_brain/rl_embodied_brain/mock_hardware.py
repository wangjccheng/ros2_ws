import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray

class MockHardwareNode(Node):
    def __init__(self):
        super().__init__('mock_hardware_node')
        # 建立四个假冒的传感器发布器
        self.pub_imu = self.create_publisher(Imu, '/livox/imu', 1)
        self.pub_joints = self.create_publisher(JointState, '/robot/joint_states', 1)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 1)
        self.pub_odom = self.create_publisher(Odometry, '/Odometry', 1)
        self.pub_map = self.create_publisher(GridMap, '/elevation_mapping/elevation_map_raw', 1)

        # 以 50Hz (0.02秒) 的极高频率疯狂发送假数据
        self.timer = self.create_timer(0.02, self.publish_mock_data)
        self.get_logger().info("🚧 硬件欺骗节点已启动！正在向 AI 大脑注入虚拟台架数据...")

    def publish_mock_data(self):
        # 1. 伪造 IMU (绝对水平，没有任何旋转和角速度)
        imu_msg = Imu()
        imu_msg.orientation.w = 1.0 
        self.pub_imu.publish(imu_msg)

        # 2. 伪造手柄指令 (无输入，期望线速度和角速度全为 0)
        self.pub_cmd.publish(Twist())

        # 3. 伪造里程计 (假设机器人在架子上，底盘中心距离地面 0.3 米)
        odom_msg = Odometry()
        odom_msg.pose.pose.position.z = 0.3 
        self.pub_odom.publish(odom_msg)

        # 4. 伪造关节状态 (4个轮子 + 4条 EHA 腿，全部静止在 0 位置)
        joint_msg = JointState()
        joint_msg.name = ['FL_wheel', 'FR_wheel', 'RL_wheel', 'RR_wheel', 'FL_leg', 'FR_leg', 'RL_leg', 'RR_leg']
        joint_msg.position = [0.0] * 8
        joint_msg.velocity = [0.0] * 8
        self.pub_joints.publish(joint_msg)

        # 5. 伪造 GPU 高程图 (极其关键：一张绝对平坦的 2.5x2.5 地毯)
        map_msg = GridMap()
        map_msg.layers = ['elevation']
        flat_data = Float32MultiArray()
        # 625 个格子的高度全部是 0.0 米 (绝对高度)
        flat_data.data = [0.0] * 441 
        map_msg.data = [flat_data]
        self.pub_map.publish(map_msg)

def main():
    rclpy.init()
    node = MockHardwareNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()