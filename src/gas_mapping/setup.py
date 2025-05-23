from setuptools import setup
from glob import glob
import os

package_name = 'gas_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs', 'geometry_msgs', 'gas_mapping_msgs'],
    zip_safe=True,
    maintainer='bedathenic',
    maintainer_email='bedathenic@todo.todo',
    description='Gas sensor mapping and visualization',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fusion_node = gas_mapping.gas_slam_fusion_node:main',
            'visualization_node = gas_mapping.gas_visualization_node:main',
            'map_builder_node = gas_mapping.gas_map_builder_node:main',
            'odom_to_pose = gas_mapping.odom_to_pose:main',  # optional
            'fake_gas_publisher = gas_mapping.fake_gas_publisher:main'  # optional
        ],
    },
)
