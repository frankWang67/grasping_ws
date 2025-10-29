import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'grasping_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, package_name), [package_name + '/grasp_pose_sampler.py']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wshf',
    maintainer_email='wshf@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection = grasping_perception.detection:main',
            'object_pointcloud_publisher = grasping_perception.object_pointcloud_publisher:main',
            'table_frame_publisher = grasping_perception.table_frame_publisher:main',
            'grasp_pose_sampler_test = grasping_perception.grasp_pose_sampler_test:main',
            'joint_state_publish_test = grasping_perception.joint_state_publish_test:main',
        ],
    },
)
