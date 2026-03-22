import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_self_filter')
    
    # 拿到两个配置文件的绝对路径
    lidar_config = os.path.join(pkg_dir, 'config', 'lidar_filter.yaml')
    camera_config = os.path.join(pkg_dir, 'config', 'camera_filter.yaml')

    # ==========================================================
    # 创建一个多线程的 Component Container (组件容器)
    # 所有的过滤器将作为插件加载到这同一个进程中，共享内存，极大地节省算力！
    # ==========================================================
    container = ComposableNodeContainer(
        name='self_filter_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt', # _mt 表示开启多线程
        composable_node_descriptions=[
            
            # 【一】雷达清洗流水线
            ComposableNode(
                package='pcl_ros',
                plugin='pcl_ros::CropBox',   # ⚠️ ROS 2 专属的插件类名声明方式
                name='lidar_filter_main',    # 这里的名字与 YAML 里的配置名一一对应
                remappings=[('input', '/merged_livox_cloud'), ('output', '/livox/filtered_stage1')],
                parameters=[lidar_config]
            ),
            ComposableNode(
                package='pcl_ros',
                plugin='pcl_ros::CropBox',
                name='lidar_filter_sub1',
                remappings=[('input', '/livox/filtered_stage1'), ('output', '/livox/filtered_stage2')],
                parameters=[lidar_config]
            ),
            ComposableNode(
                package='pcl_ros',
                plugin='pcl_ros::CropBox',
                name='lidar_filter_sub2',
                remappings=[('input', '/livox/filtered_stage2'), ('output', '/livox/merged_cloud_filtered')],
                parameters=[lidar_config]
            ),

            # 【二】相机清洗流水线
            ComposableNode(
                package='pcl_ros',
                plugin='pcl_ros::CropBox',
                name='camera_filter_main',
                remappings=[('input', '/camera/camera/depth/color/points'), ('output', '/camera/filtered_stage1')],
                parameters=[camera_config]
            ),
            ComposableNode(
                package='pcl_ros',
                plugin='pcl_ros::CropBox',
                name='camera_filter_sub1',
                remappings=[('input', '/camera/filtered_stage1'), ('output', '/camera/filtered_stage2')],
                parameters=[camera_config]
            ),
            ComposableNode(
                package='pcl_ros',
                plugin='pcl_ros::CropBox',
                name='camera_filter_sub2',
                remappings=[('input', '/camera/filtered_stage2'), ('output', '/camera/filtered_points')],
                parameters=[camera_config]
            )
        ],
        output='screen',
    )

    return LaunchDescription([container])



'''
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_self_filter')
    
    # 拿到两个配置文件的绝对路径
    lidar_config = os.path.join(pkg_dir, 'config', 'lidar_filter.yaml')
    camera_config = os.path.join(pkg_dir, 'config', 'camera_filter.yaml')

    # ==========================================================
    # 【一】雷达清洗流水线 (喂给 FAST-LIO)
    # ==========================================================
    lidar_main = Node(
        package='pcl_ros', executable='CropBoxNode', name='lidar_filter_main',
        remappings=[('input', '/merged_livox_cloud'), ('output', '/livox/filtered_stage1')],
        parameters=[lidar_config]
    )
    lidar_sub1 = Node(
        package='pcl_ros', executable='CropBoxNode', name='lidar_filter_sub1',
        remappings=[('input', '/livox/filtered_stage1'), ('output', '/livox/filtered_stage2')],
        parameters=[lidar_config]
    )
    lidar_sub2 = Node(
        package='pcl_ros', executable='CropBoxNode', name='lidar_filter_sub2',
        remappings=[('input', '/livox/filtered_stage2'), ('output', '/livox/merged_cloud_filtered')], # 最终输出
        parameters=[lidar_config]
    )

    # ==========================================================
    # 【二】相机清洗流水线 (喂给高程建图 Elevation Mapping)
    # ==========================================================
    camera_main = Node(
        package='pcl_ros', executable='CropBoxNode', name='camera_filter_main',
        remappings=[
            ('input', '/camera/camera/depth/color/points'), # 接收原始深度点云
            ('output', '/camera/filtered_stage1')
        ],
        parameters=[camera_config]
    )
    camera_sub1 = Node(
        package='pcl_ros', executable='CropBoxNode', name='camera_filter_sub1',
        remappings=[
            ('input', '/camera/filtered_stage1'), 
            ('output', '/camera/filtered_stage2')
        ],
        parameters=[camera_config]
    )
    camera_sub2 = Node(
        package='pcl_ros', executable='CropBoxNode', name='camera_filter_sub2',
        remappings=[
            ('input', '/camera/filtered_stage2'), 
            ('output', '/camera/filtered_points') # 最终喂给高程图的纯净数据
        ],
        parameters=[camera_config]
    )

    return LaunchDescription([
        # 启动雷达的3个节点
        lidar_main, lidar_sub1, lidar_sub2,
        # 启动相机的3个节点
        camera_main, camera_sub1, camera_sub2
    ])
'''
