import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rl_embodied_brain'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 加入下面这行，告诉 ROS 2 把 launch 文件夹搬运到安装目录
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agx',
    maintainer_email='2081157056@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 格式：'终端启动命令 = 包名.文件名:主函数'
            'policy_node = rl_embodied_brain.policy_inference_node:main'
        ],
    },
)
