from setuptools import setup
from glob import glob
import os

package_name = 'lz_gripper_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 添加launch文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 添加nodes文件
        (os.path.join('share', package_name, 'nodes'), glob('nodes/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='LZ Gripper ROS2 Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_controller_node.py = nodes.hand_controller_node:main',
            'test_hand_client.py = nodes.test_hand_client:main',
        ],
    },
) 