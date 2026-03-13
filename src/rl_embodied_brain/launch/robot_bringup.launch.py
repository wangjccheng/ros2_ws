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
    tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',

        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.5', '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0', '--frame-id', 'base_link', '--child-frame-id', 'livox_frame']
    )
    
    # 相机在底盘上的位置 (假设 D435i 安装在车头前 0.8 米处)
    tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0.8', '--y', '0.0', '--z', '-0.2', '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0', '--frame-id', 'base_link', '--child-frame-id', 'camera_link']
    )
    # 彻底修正语法错误的 body -> base_link 静态 TF
    tf_body_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0', '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0', '--frame-id', 'body', '--child-frame-id', 'base_link']
    )
    
    # ==========================================
    # 2. 硬件与底层算法驱动 
    # (目前你手边没硬件，这里先写好模板，等硬件到了取消注释即可)
    # ==========================================
  
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('livox_ros_driver2'), 'launch_ROS2', 'msg_MID360_launch.py')])
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
        launch_arguments={'enable_pointcloud': 'true', 'align_depth.enable': 'true'}.items()
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
        period=3.0,
        actions=[
            Node(
                package='elevation_mapping_cupy',
                executable='elevation_mapping_node.py', # 【修正 1】加上 .py 后缀
                name='elevation_mapping_cupy',          # 【修正 2】节点名必须和官方一致
                parameters=[
                    core_param_file,                    # 【修正 3】官方数百个参数垫底
                    '/home/agx/ros2_ws/ronghe.yaml'     # 你的定制 25x25 参数覆盖
                ]
            )
        ]
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
        tf_camera,
        tf_body_to_base,
         livox_driver,     # 硬件到位后解除注释
         fast_lio,         # 硬件到位后解除注释
         realsense_camera, # 硬件到位后解除注释
        elevation_mapping,
        policy_brain
    ])