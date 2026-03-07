import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('elevation_mapping_cupy')
    core_param_file = os.path.join(pkg_share_dir, 'config', 'core', 'core_param.yaml')

    home_dir = os.environ['HOME']
    user_param_file = os.path.join(home_dir, 'ros2_ws', 'ronghe.yaml')

    elevation_mapping_node = Node(
        package='elevation_mapping_cupy',
        executable='elevation_mapping_node.py',
        # 【关键修改 1】将节点名改为包原本的名称，防止 ROS2 忽略官方 YAML
        name='elevation_mapping_cupy', 
        output='screen',
        parameters=[
            core_param_file,
            user_param_file,
            # 【关键修改 2】暴力注入缺失的参数，彻底杜绝 Undeclared 报错
            {
                'use_chainer': False,
                'use_inpaint': False
            }
        ]
    )

    return LaunchDescription([
        elevation_mapping_node
    ])
