import rclpy
from rclpy.node import Node
import torch
import numpy as np

# ROS 2 消息类型
from grid_map_msgs.msg import GridMap
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from rclpy.qos import qos_profile_sensor_data
class EmbodiedBrainNode(Node):
    def __init__(self):
        super().__init__('policy_inference_node')
        self.device = torch.device('cuda:0')
        self.model = torch.jit.load('/home/agx/ros2_ws/src/rl_embodied_brain/rl_embodied_brain/policy_5.pt').to(self.device)
        self.model.eval()
        
        try:
            self.model.flatten_parameters()
        except Exception:
            pass 

        self.hidden_state = torch.zeros((1, 1, 128), device=self.device).contiguous() 

        # ====== 内部状态缓存 ======
        self.base_ang_vel = np.zeros(3)
        self.proj_grav = np.array([0.0, 0.0, -1.0])
        self.cmd_vw = np.zeros(2)       
        self.wheel_vel = np.zeros(4)    
        self.leg_pos = np.zeros(4)      
        self.leg_vel = np.zeros(4)      
        self.prev_action = np.zeros(6)  
        self.height_scan = np.zeros(441)
        self.current_base_z = 0.0       

        # ====== 传感器就绪标志位 ======
        self.imu_ready = False
        self.odom_ready = False
        self.map_ready = False
        self.joints_ready = False # 【修复 1】：增加关节就绪标志位，填补熔断盲区
        self.first_joint_init = False # 用于平滑初始化 prev_action

        # 【修复 2】：预定义底层硬件的准确关节名称 (请根据你真机底层的实际命名修改此处！)
        self.wheel_names = ['FL_wheel', 'FR_wheel', 'RL_wheel', 'RR_wheel']
        self.leg_names = ['FL_leg', 'FR_leg', 'RL_leg', 'RR_leg']

        # ====== ROS 2 订阅器 ======
        self.sub_imu = self.create_subscription(Imu, '/livox/imu', self.imu_callback, 1)
        self.sub_joints = self.create_subscription(JointState, '/robot/joint_states', self.joint_callback, 1)
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 1)
        #self.sub_elevation = self.create_subscription(GridMap, '/elevation_mapping_cupy/elevation_map_raw', self.map_callback, 1)
        # 修改后：
        self.sub_elevation = self.create_subscription(
        GridMap, 
        '/elevation_mapping_cupy/elevation_map_raw', 
        self.map_callback, 
        qos_profile_sensor_data  # 告诉大脑用 Best Effort 去接收
            )
        self.sub_odom = self.create_subscription(Odometry, '/Odometry', self.odom_callback, 1)

        # ====== 动作发布器 ======
        self.pub_action = self.create_publisher(Float32MultiArray, '/robot/action_command_raw', 1)
        self.pub_debug_obs = self.create_publisher(Float32MultiArray, '/robot/debug/ai_observation', 1)

        # 50Hz 核心控制循环
        self.timer = self.create_timer(0.02, self.control_loop)

    def imu_callback(self, msg):
        self.base_ang_vel = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        rot = R.from_quat(quat)
        self.proj_grav = rot.inv().apply(np.array([0.0, 0.0, -1.0]))
        self.imu_ready = True

    def cmd_callback(self, msg):
        self.cmd_vw = np.array([msg.linear.x, msg.angular.z])

    def joint_callback(self, msg):
        # 【修复 3】：通过名称安全匹配索引，彻底杜绝数组顺序错乱导致手脚不分
        try:
            for i, name in enumerate(self.wheel_names):
                idx = msg.name.index(name)
                self.wheel_vel[i] = msg.velocity[idx]
                
            for i, name in enumerate(self.leg_names):
                idx = msg.name.index(name)
                self.leg_pos[i] = msg.position[idx]
                self.leg_vel[i] = msg.velocity[idx]
                
            # 【修复 4】：冷启动平滑保护。用真实腿部位姿反推 prev_action，防止上电第一下抽搐
            if not self.first_joint_init:
                # 假设腿部动作缩放系数是 0.3，这里反向除以 0.3 获取归一化动作
                self.prev_action[2:6] = np.clip(self.leg_pos / 0.3, -1.0, 1.0)
                self.first_joint_init = True
                
            self.joints_ready = True
            
        except ValueError as e:
            self.get_logger().warn(f"JointState 缺少指定的关节名称: {e}，请检查底层驱动命名！", throttle_duration_sec=2.0)

    def odom_callback(self, msg):
        self.current_base_z = msg.pose.pose.position.z
        self.odom_ready = True

    def map_callback(self, msg):
        try:
            layer_idx = msg.layers.index('elevation')
            terrain_data = np.array(msg.data[layer_idx].data)
            terrain_data = np.nan_to_num(terrain_data, nan=self.current_base_z)
            relative_height = terrain_data - self.current_base_z
            relative_height = np.clip(relative_height, -2.0, 2.0)

            # 【修复 5】：高程图行列顺序对齐 Isaac Lab
            # 动态计算网格边长 (例如 441 维开方是 21，625 维开方是 25)
            grid_size = int(np.sqrt(relative_height.shape[0]))
            grid_2d = relative_height.reshape(grid_size, grid_size)
            
            # 通常 Isaac Lab 是前到后、左到右扫描，而 GridMap 可能是不同的主序。
            # 这里先做转置对齐，如果实车发现避障左右反了，可以在这里加上 np.fliplr 或 np.flipud
            self.height_scan = grid_2d.T.flatten() 

            self.map_ready = True
            
        except ValueError:
            self.get_logger().warn("高程图中找不到 'elevation' 图层", throttle_duration_sec=2.0)

    def control_loop(self):
        # 【修复 6】：将 joints_ready 加入熔断防线，必须等电机上线才推理
        if not (self.imu_ready and self.odom_ready and self.map_ready and self.joints_ready):
            self.get_logger().info("等待传感器数据流介入... 保持底层急停状态", throttle_duration_sec=2.0)
            safe_action_msg = Float32MultiArray()
            safe_action_msg.data = [0.0] * 6
            self.pub_action.publish(safe_action_msg)
            return  
        
        with torch.no_grad():
            obs_flat_np = np.concatenate([
                self.base_ang_vel,  # 3
                self.proj_grav,     # 3
                self.cmd_vw,        # 2
                self.wheel_vel,     # 4
                self.leg_pos,       # 4
                self.leg_vel,       # 4
                self.prev_action,   # 6 
                self.height_scan    # 根据你 yaml 尺寸，可能是 441 或 625
            ])
            # 【新增】：将组装好的观测向量发布出去供 RQT 或终端观测
            debug_obs_msg = Float32MultiArray()
            debug_obs_msg.data = obs_flat_np.tolist()
            self.pub_debug_obs.publish(debug_obs_msg)
            # 【修复 7】：解决网络输入维度崩溃问题
            # 1. 使用 view(1, 1, -1) 适配 GRU 的 sequence 维度要求
            # 2. 使用 -1 让 PyTorch 自动推断长度，即使你修改了 YAML 的地图分辨率也不会越界报错
            obs_tensor = torch.tensor(obs_flat_np, dtype=torch.float32, device=self.device).view(1,-1)
            
            action_tensor, self.hidden_state = self.model(obs_tensor, self.hidden_state)
            raw_action_np = action_tensor.cpu().numpy().flatten()
            clipped_action = np.clip(raw_action_np, -1.0, 1.0)
            self.prev_action = clipped_action

            scaled_action = np.zeros(6)
            scaled_action[0] = clipped_action[0] * 2.0  
            scaled_action[1] = clipped_action[1] * 1.0  
            scaled_action[2:6] = clipped_action[2:6] * 0.3  

            self.get_logger().info(f"🧠 AI 动作指令: {np.round(scaled_action, 3)}")
            action_msg = Float32MultiArray()
            action_msg.data = scaled_action.tolist()
            self.pub_action.publish(action_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EmbodiedBrainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

'''''
import rclpy
from rclpy.node import Node
import torch
import numpy as np

# ROS 2 消息类型
from grid_map_msgs.msg import GridMap
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R

class EmbodiedBrainNode(Node):
    def __init__(self):
        super().__init__('policy_inference_node')
        self.device = torch.device('cuda:0')
        # 记得把你的模型文件放到同级目录下，或写绝对路径
        self.model = torch.jit.load('/home/agx/ros2_ws/src/rl_embodied_brain/rl_embodied_brain/policy_5.pt').to(self.device)
        self.model.eval()
        # 加入这行，让 GRU 权重在显存里连续排列，压榨极速性能
        try:
            self.model.flatten_parameters()
        except Exception:
            pass # 如果 JIT 导出时抹除了这个方法，忽略即可

        self.hidden_state = torch.zeros((1, 1, 128), device=self.device).contiguous() # 把隐状态也标记为连续
        

        # ====== 内部状态缓存 ======
        self.base_ang_vel = np.zeros(3)
        self.proj_grav = np.array([0.0, 0.0, -1.0])
        self.cmd_vw = np.zeros(2)       
        self.wheel_vel = np.zeros(4)    
        self.leg_pos = np.zeros(4)      
        self.leg_vel = np.zeros(4)      
        self.prev_action = np.zeros(6)  # IsaacLab 通常缓存的是缩放前 (-1到1) 的动作
        self.height_scan = np.zeros(441)
        self.current_base_z = 0.0       # 极其重要：用于计算相对高度

        # ====== ROS 2 订阅器 ======
        self.sub_imu = self.create_subscription(Imu, '/livox/imu', self.imu_callback, 1)
        self.sub_joints = self.create_subscription(JointState, '/robot/joint_states', self.joint_callback, 1)
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 1)
        self.sub_elevation = self.create_subscription(GridMap, '/elevation_mapping/elevation_map_raw', self.map_callback, 1)
        # 新增：订阅 FAST-LIO2 的里程计，实时获取底盘高度
        self.sub_odom = self.create_subscription(Odometry, '/Odometry', self.odom_callback, 1)

        # ====== 动作发布器 ======
        self.pub_action = self.create_publisher(Float32MultiArray, '/robot/action_command_raw', 1)

        # 50Hz 核心控制循环
        self.timer = self.create_timer(0.02, self.control_loop)
        self.imu_ready = False
        self.odom_ready = False
        self.map_ready = False

    def imu_callback(self, msg):
        self.base_ang_vel = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        rot = R.from_quat(quat)
        self.proj_grav = rot.inv().apply(np.array([0.0, 0.0, -1.0]))
        self.imu_ready = True

    def cmd_callback(self, msg):
        self.cmd_vw = np.array([msg.linear.x, msg.angular.z])

    def joint_callback(self, msg):
            # 假设底层的 JointState 消息里按顺序包含了轮速和腿部位姿
        # TODO: 这里需要你根据真机底层发上来的具体格式，做准确的提取
        try:
            # 假设底层发来的数组前 4 个是轮子，后 4 个是 EHA 悬挂腿
            # 提取 4 个轮子的真实转速
            self.wheel_vel = np.array(msg.velocity[0:4])
            # 提取 4 条 EHA 悬挂的当前位姿和伸缩速度
            self.leg_pos = np.array(msg.position[4:8])
            self.leg_vel = np.array(msg.velocity[4:8])
        except IndexError:
            self.get_logger().warn("接收到的 JointState 维度不对，请检查底层驱动！")

    def odom_callback(self, msg):
        # 提取机器人在 odom 坐标系下的绝对 Z 高度
        self.current_base_z = msg.pose.pose.position.z
        self.odom_ready = True

    def map_callback(self, msg):
        # 【你指出的那个缺失的函数，核心补全在这里！】
        try:
            # 1. 从多层 GridMap 中找到名为 'elevation' 的层
            layer_idx = msg.layers.index('elevation')
            
            # 2. 提取地图数据并转换为 numpy 数组
            # 注意：ROS 2 的 Float32MultiArray 存放在 data 字段里
            terrain_data = np.array(msg.data[layer_idx].data)

            # 3. 处理盲区/雷达没扫到的地方 (通常被标记为 NaN)
            # 我们把没有数据的地方，安全地假设为和平地一样高（相对高度为0）
            terrain_data = np.nan_to_num(terrain_data, nan=self.current_base_z)

            # 4. 计算相对高度 (地图绝对高度 - 机器人绝对高度)
            relative_height = terrain_data - self.current_base_z

            # 5. 【高程图限幅】：限制在正负 2.0 米以内
            relative_height = np.clip(relative_height, -2.0, 2.0)

            # 6. 拉平成 625 维的一维向量并存入缓存
            self.height_scan = relative_height.flatten()

            self.map_ready = True
            
        except ValueError:
            self.get_logger().warn("高程图中找不到 'elevation' 图层，请检查 CuPy 配置！")

    def control_loop(self):
        
        # 【安全熔断机制】：必须等核心传感器全部上线，才允许网络前向推理！
        if not (self.imu_ready and self.odom_ready and self.map_ready):
            self.get_logger().info("等待传感器数据流介入... 保持底层急停状态", throttle_duration_sec=2.0)
            
                # 发送全 0 的绝对安全指令，确保 150kg 底盘死死锁住
            safe_action_msg = Float32MultiArray()
            safe_action_msg.data = [0.0] * 6
            self.pub_action.publish(safe_action_msg)
            return  # 直接跳出，不执行后续的神经网络代码
        
        with torch.no_grad():
            # 1. 拼装 651 维的观测空间
            obs_flat_np = np.concatenate([
                self.base_ang_vel,  # 3
                self.proj_grav,     # 3
                self.cmd_vw,        # 2
                self.wheel_vel,     # 4
                self.leg_pos,       # 4
                self.leg_vel,       # 4
                self.prev_action,   # 6 (保存的是缩放前的值)
                self.height_scan    # 625 (已被 clip 在 -2~2 之间)
            ])
            # 动态获取长度，防止 view 报错
            actual_size = obs_flat_np.shape[0]
            obs_tensor = torch.tensor(obs_flat_np, dtype=torch.float32, device=self.device).view(1, 467)
            #obs_tensor = torch.tensor(obs_flat_np, dtype=torch.float32, device=self.device).view(1, 1, actual_size)
            # 2. 转换为 Tensor 并前向推理
            #obs_tensor = torch.tensor(obs_flat_np, dtype=torch.float32, device=self.device).view(1, 1, 467)
            action_tensor, self.hidden_state = self.model(obs_tensor, self.hidden_state)
            
            # 拿到原始的 6 维输出
            raw_action_np = action_tensor.cpu().numpy().flatten()

            # 3. 【动作输出限幅】：截断在 [-1.0, 1.0] 之间
            clipped_action = np.clip(raw_action_np, -1.0, 1.0)

            # 刷新上一帧动作缓存 (在 Isaac Lab 中，网络下一次吃的 prev_action 是缩放前的归一化值)
            self.prev_action = clipped_action

            # 4. 【动作输出缩放】：乘以各自的物理缩放系数
            scaled_action = np.zeros(6)
            scaled_action[0] = clipped_action[0] * 2.0  # 1维车身线速度，乘 2.0
            scaled_action[1] = clipped_action[1] * 1.0  # 1维车身角速度，乘 1.0
            scaled_action[2:6] = clipped_action[2:6] * 0.3  # 4维腿位，乘 0.3

            # 5. 打包下发给底层电机/液压控制器
            self.get_logger().info(f"🧠 AI 动作指令: {np.round(scaled_action, 3)}") # 新增这行打印
            action_msg = Float32MultiArray()
            action_msg.data = scaled_action.tolist()
            self.pub_action.publish(action_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EmbodiedBrainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''