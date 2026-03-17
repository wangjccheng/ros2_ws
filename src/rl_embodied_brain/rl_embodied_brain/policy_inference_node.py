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
        self.model = torch.jit.load('/home/agx/ros2_ws/src/rl_embodied_brain/rl_embodied_brain/policy_0313.pt').to(self.device)
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
        self.wheel_vel = np.zeros(4)   #网络输入顺序【LB、LF、RF、RB】 
        self.leg_pos = np.zeros(4)      #同上
        self.leg_vel = np.zeros(4)      #同上
        self.prev_action = np.zeros(10)  #动作输出顺序【残轮-LF、LB、RF、RB】+【腿-LF、RF、LB、RB】
        self.height_scan = np.zeros(625)
        self.current_base_z = 0.0       

        # ====== 传感器就绪标志位 ======
        self.imu_ready = False
        self.odom_ready = False
        self.map_ready = False
        self.legs_ready = False   # 【修改 1】：原 joints_ready 拆分为腿部就绪
        self.wheels_ready = False # 【修改 1】：新增轮子就绪
        self.first_joint_init = False 

        # 预定义底层硬件的准确关节名称
        self.wheel_names = ['LB_wheel', 'LF_wheel', 'RF_wheel', 'RB_wheel']
        self.leg_names = ['LB_leg', 'LF_leg', 'RF_leg', 'RB_leg']

        # ====== ROS 2 订阅器 ======
        self.sub_imu = self.create_subscription(Imu, '/livox/imu', self.imu_callback, 1)
        
        # 【修改 2】：将关节订阅器拆分为腿部和轮子两个
        self.sub_legs = self.create_subscription(JointState, '/robot/joint_states', self.leg_callback, 1)
        # ⚠️ 注意：这里假设轮子话题叫 /robot/wheel_states，请根据实际情况修改
        self.sub_wheels = self.create_subscription(JointState, '/robot/wheel_states', self.wheel_callback, 1)
        
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 1)
        self.sub_elevation = self.create_subscription(
            GridMap, 
            '/elevation_mapping_node/elevation_map_raw', 
            self.map_callback, 
            qos_profile_sensor_data  
        )
        self.sub_odom = self.create_subscription(Odometry, '/Odometry', self.odom_callback, 1)

        # ====== 动作发布器 ======
        self.pub_action = self.create_publisher(Float32MultiArray, '/robot/action_command_raw', 1)
        self.pub_debug_obs = self.create_publisher(Float32MultiArray, '/robot/debug/ai_observation', 1)

        # 200Hz 核心控制循环
        self.timer = self.create_timer(0.005, self.control_loop)

    def imu_callback(self, msg):
        #！！！注意这个版本是IMU（雷达）倒放，对输入进行了处理
        # 1. 角速度翻转 (绕X轴转180度：X不变，Y取反，Z取反)
        self.base_ang_vel = np.array([
            msg.angular_velocity.x, 
            -msg.angular_velocity.y, 
            -msg.angular_velocity.z
        ])
        
        # 2. 重力投影翻转
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        rot_lidar = R.from_quat(quat)
        proj_grav_lidar = rot_lidar.inv().apply(np.array([0.0, 0.0, -1.0]))
        
        # 将倒放雷达系下的重力向量，转换回 base_link 系
        self.proj_grav = np.array([
            proj_grav_lidar[0], 
            -proj_grav_lidar[1], 
            -proj_grav_lidar[2]
        ])
        self.imu_ready = True

    def cmd_callback(self, msg):
        self.cmd_vw = np.array([msg.linear.x, msg.angular.z])

    # 【修改 3】：专门处理腿部数据的回调函数
    def leg_callback(self, msg):
        try:
            for i, name in enumerate(self.leg_names):
                idx = msg.name.index(name)
                self.leg_pos[i] = msg.position[idx]
                self.leg_vel[i] = msg.velocity[idx]
                
            if not self.first_joint_init:
                self.prev_action[6:10] = np.clip(self.leg_pos / 0.3, -1.0, 1.0)
                self.first_joint_init = True
                
            self.legs_ready = True
            
        except ValueError as e:
            self.get_logger().warn(f"腿部状态缺少指定的关节名称: {e}", throttle_duration_sec=2.0)

    # 【修改 4】：新增专门处理轮子数据的回调函数
    def wheel_callback(self, msg):
        try:
            for i, name in enumerate(self.wheel_names):
                idx = msg.name.index(name)
                self.wheel_vel[i] = msg.velocity[idx]
                
            self.wheels_ready = True
            
        except ValueError as e:
            self.get_logger().warn(f"轮子状态缺少指定的关节名称: {e}", throttle_duration_sec=2.0)

    def odom_callback(self, msg):
        #！！！这里根据雷达相对车辆base垂向偏移更改
        # 将雷达的 Z 高度向下平移 0.5 米，得到真实的底盘中心高度
        self.current_base_z = msg.pose.pose.position.z - 0.5
        self.odom_ready = True

    def map_callback(self, msg):
        try:
            layer_idx = msg.layers.index('elevation')
            terrain_data = np.array(msg.data[layer_idx].data)
            terrain_data = np.nan_to_num(terrain_data, nan=self.current_base_z)
            relative_height = terrain_data - self.current_base_z
            relative_height = np.clip(relative_height, -2.0, 2.0)

            grid_size = int(np.sqrt(relative_height.shape[0]))
            grid_2d = relative_height.reshape(grid_size, grid_size)
            
            EXPECTED_SIZE = 25  
            
            if grid_size < EXPECTED_SIZE:
                self.get_logger().warn(f"高程图过小: 当前 {grid_size}x{grid_size}，AI需要 {EXPECTED_SIZE}x{EXPECTED_SIZE}！", throttle_duration_sec=2.0)
                return

            start_idx = (grid_size - EXPECTED_SIZE) // 2
            end_idx = start_idx + EXPECTED_SIZE
            cropped_grid = grid_2d[start_idx:end_idx, start_idx:end_idx]
            
            self.height_scan = np.flipud(cropped_grid).flatten() 
            self.map_ready = True
            
        except ValueError:
            self.get_logger().warn("高程图中找不到 'elevation' 图层", throttle_duration_sec=2.0)

    def control_loop(self):
        # 【修改 5】：更新就绪状态的检查，要求腿部和轮子同时就绪
        missing_sensors = []
        if not self.imu_ready:
            missing_sensors.append("IMU (/livox/imu)")
        if not self.odom_ready:
            missing_sensors.append("Odometry (/Odometry)")
        if not self.map_ready:
            missing_sensors.append("ElevationMap (高程图)")
        if not self.legs_ready:
            missing_sensors.append("LegStates (腿部)")
        if not self.wheels_ready:
            missing_sensors.append("WheelStates (轮子)")

        if missing_sensors:
            missing_str = ", ".join(missing_sensors)
            self.get_logger().info(f"等待数据流... 当前缺失: [{missing_str}]，保持底层急停状态", throttle_duration_sec=2.0)
            
            safe_action_msg = Float32MultiArray()
            safe_action_msg.data = [0.0] * 10
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
                self.prev_action,   # 10 
                self.height_scan    # 625
            ])
            
            debug_obs_msg = Float32MultiArray()
            debug_obs_msg.data = obs_flat_np.tolist()
            self.pub_debug_obs.publish(debug_obs_msg)
            
            obs_tensor = torch.tensor(obs_flat_np, dtype=torch.float32, device=self.device).view(1,-1)
            
            action_tensor, self.hidden_state = self.model(obs_tensor, self.hidden_state)
            raw_action_np = action_tensor.cpu().numpy().flatten()
            clipped_action = np.clip(raw_action_np, -1.0, 1.0)
            self.prev_action = clipped_action

            scaled_action = np.zeros(8)
            target_vel = clipped_action[0] * 1.0  
            target_ang = clipped_action[1] * 0.5 
            wl = (target_vel - target_ang * (0.68 / 2.0)) / 0.19
            wr = (target_vel + target_ang * (0.68 / 2.0)) / 0.19
            scaled_action[0:2] = wl + clipped_action[2:4] * 0.1
            scaled_action[2:4] = wr + clipped_action[4:6] * 0.1  
            scaled_action[4:8] = clipped_action[6:10] * 0.3 

            self.get_logger().info(f"🧠 AI 动作指令: {scaled_action[0:4]}轮子，{scaled_action[4:8]}EHA")
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