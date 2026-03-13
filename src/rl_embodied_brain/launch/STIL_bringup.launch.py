import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # ==========================================
    # 1. 启动虚拟台架驱动 (Mock Hardware)
    # 持续高频发送平地、0速度、悬空状态等假数据
    # ==========================================
    mock_hardware_node = Node(
        package='rl_embodied_brain',
        executable='mock_hardware_node', # 注意：请确保这是你在 setup.py 中注册的实际 entry_point 名称
        name='mock_hardware',
        output='screen'
    )

    # ==========================================
    # 2. 启动硬件安全小脑 (Safety Cerebellum)
    # 拦截并过滤 AI 指令，进行限幅和兜底
    # ==========================================
    safety_cerebellum_node = Node(
        package='robot_safety_core',
        executable='safety_cerebellum_node', 
        name='safety_cerebellum',
        output='screen'
    )

    # ==========================================
    # 3. 嵌套调用主算法栈 (TF + 建图 + 大脑)
    # 这里会继承原有 robot_bringup.launch.py 里面的延迟启动逻辑
    # ==========================================
    rl_brain_share_dir = get_package_share_directory('rl_embodied_brain')
    
    robot_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rl_brain_share_dir, 'launch', 'robot_bringup.launch.py')
        )
    )

    # 将所有节点和包含的 launch 文件打包返回
    return LaunchDescription([
        mock_hardware_node,
        safety_cerebellum_node,
        robot_bringup_launch
    ])