import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # ==========================================
    # 1. 静态 TF 桥梁 (瞬间启动，最先执行)
    # 参数: X Y Z Yaw Pitch Roll 父坐标系 子坐标系
    # ==========================================
    # 雷达在底盘上的位置 (假设 Mid-360 安装在车体正中心偏上 0.5 米)
    # 注意坐标系方向
    tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',

        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.5', '--yaw', '0.0', '--pitch', '0.0', '--roll', '3.1415926', '--frame-id', 'base_link', '--child-frame-id', 'livox_frame']
    )
    # === 新增：Avia 的静态 TF ===
    # 假设 Avia 装在 Mid360 前方 0.3 米处，你需要根据你的 150kg 机器人的实际安装位置测量并修改 X Y Z 
    tf_avia = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0.3', '--y', '0.0', '--z', '0.5', '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0', '--frame-id', 'base_link', '--child-frame-id', 'avia_frame']
    )
    # 相机在底盘上的位置 (假设 D435i 安装在车头前 0.8 米处)
    tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.5', '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0', '--frame-id', 'base_link', '--child-frame-id', 'camera_link']
    )
    # 彻底修正语法错误的 body -> base_link 静态 TF
    tf_body_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '-0.5', '--yaw', '0.0', '--pitch', '0.0', '--roll', '3.1415926', '--frame-id', 'body', '--child-frame-id', 'base_link']
    )
    
    # ==========================================
    # 2. 硬件与底层算法驱动 
    # (目前你手边没硬件，这里先写好模板，等硬件到了取消注释即可)
    # ==========================================
  
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('livox_ros_driver2'), 'launch_ROS2', 'msg_MID360_launch.py')])
    )
    # === 新增：Avia 的驱动启动 ===
    # 确保你已经在这个 launch 文件内部把话题重映射成了 /avia/lidar 和 /avia/imu
    avia_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('livox_ros2_avia'), 'launch', 'livox_lidar_launch.py')])
    )

    # === 新增：双雷达鲁棒融合节点 ===
    # 将包名 'livox_merger' 替换为你实际创建的 ROS 2 package 名字
    livox_merge_node = Node(
        package='livox_merger', 
        executable='robust_livox_merge_node',
        name='robust_livox_merge_node',
        output='screen'
    )
    # 先获取 fast_lio 安装后的共享目录路径
    fast_lio_share_dir = get_package_share_directory('fast_lio')
    # 拼接出 mid360.yaml 的绝对路径
    fast_lio_config_path = os.path.join(fast_lio_share_dir, 'config', 'mid360.yaml')
    
    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py')]),
        launch_arguments={'config_file': fast_lio_config_path}.items()
    )

    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')]),
        launch_arguments={'enable_pointcloud': 'true',      # 推荐使用这种带点的写法
            'align_depth.enable': 'true',
            'pointcloud.stream_filter': '2',  # 0:深度, 1:颜色, 2:两者 (为了融合颜色)
            'enable_sync': 'true',
            'enable_color': 'true',
            'filters': 'spatial,temporal,decimation',
            'depth_qos': 'DEFAULT',
            'color_qos': 'DEFAULT',
            'camera_info_qos': 'DEFAULT',
            'depth_module.visual_preset': '2'
            }.items()
    )

    pointcloud_node = Node(
        package='depth_image_proc',
        executable='point_cloud_xyzrgb_node',
        name='point_cloud_xyzrgb_node',
        remappings=[
            ('rgb/image_rect_color', '/camera/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('depth_registered/image_rect', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('points', '/camera/camera/depth/color/points')
        ]
    )
    # ==========================================
    # 新增: IMU 姿态滤波 (Madgwick)
    # 补充 Livox 缺失的四元数，将 /livox/imu 解算后输出到 /livox/imu_filtered
    # ==========================================
    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_node',
        output='screen',
        parameters=[{
            'use_mag': False,        # 无地磁计
            'publish_tf': False,     # 不发布 TF，避免与 FAST-LIO 冲突
            'world_frame': 'enu'
        }],
        remappings=[
            ('/imu/data_raw', '/livox/imu'),
            ('/imu/data', '/livox/imu_filtered')
        ]
    )

    # 获取官方核心参数路径
    core_param_file = os.path.join(
        get_package_share_directory('elevation_mapping_cupy'), 
        'config', 'core', 'core_param.yaml'
    )
    # ==========================================
    # 3. GPU 高程建图 (延迟 3 秒启动)
    # 必须等 FAST-LIO2 吐出 odom，以及静态 TF 建立完毕后，建图网格才能初始化
    # ==========================================
    elevation_mapping = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='elevation_mapping_cupy',
                executable='elevation_mapping_node.py', # 【修正 1】加上 .py 后缀
                name='elevation_mapping_node',          # 【修正 2】节点名必须和官方一致
                parameters=[
                    core_param_file,                    # 【修正 3】官方数百个参数垫底
                    #'/home/agx/ros2_ws/ronghe.yaml'     # 你的定制 25x25 参数覆盖
                ]
            )
        ]
    )
    # ==========================================
    # 4. 底层通信与安全防线 (与高程图同步或稍后启动)
    # 确保在 AI 发出指令前，底层通道和安全锁死已经准备就绪
    # ==========================================
    udp_bridge = Node(
        package='udp_bridge',              # 【关键修改】：这里换成新包的名字
        executable='udp_bridge_node',
        name='udp_bridge_node',
        output='screen',
        parameters=[{
            'target_ip': '192.168.1.5',
            'target_port': 25000,
            'local_port': 25001
        }]
    )

    safety_cerebellum = Node(
        package='robot_safety_core',  # 【关键修改】：这里换成新包的名字
        executable='safety_cerebellum_node',
        name='safety_cerebellum_node',
        output='screen',
        parameters=[{
            'dry_run_mode': False  # ⚠️ 极其重要：实车调试初期务必保持为 True，确保只打印不输出动力
        }]
    )

    # ==========================================
    # 4. AI 大脑推理节点 (延迟 6 秒启动)
    # 确保 CuPy 已经完全预热，并且有了第一帧的 25x25 地图数据
    # ==========================================
    policy_brain = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rl_embodied_brain',
                executable='policy_node',
                name='policy_node',
                output='screen' # 把 print 信息和报错直接打印在当前终端
            )
        ]
    )

    # 将所有拼图打包返回
    return LaunchDescription([
        tf_lidar,
        tf_avia,
        tf_camera,
        tf_body_to_base,
        livox_driver, 
        avia_driver,
        livox_merge_node,
        pointcloud_node,     # 硬件到位后解除注释
         fast_lio,         # 硬件到位后解除注释
         realsense_camera, # 硬件到位后解除注释
        imu_filter,          # 硬件到位后解除注释
        elevation_mapping,
        udp_bridge,
        safety_cerebellum,
        policy_brain
        
    ])