from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([

        # Your gas fusion node
        Node(
            package='gas_mapping',
            executable='fusion_node',
            name='fusion_node'
        ),

        # Your gas visualization node
        Node(
            package='gas_mapping',
            executable='visualization_node',
            name='visualization_node',
            parameters=[{'visible_gases': ['mq2', 'mq3']}]
        ),

        # Your gas map builder node
 #       Node(
 #           package='gas_mapping',
 #           executable='map_builder_node',
 #           name='map_builder_node'
 #       ),

        # (Optional) Pose bridge if your nodes need it
        Node(
            package='gas_mapping',
            executable='odom_to_pose',
            name='odom_to_pose'
        ),

        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('gas_mapping'),
                'rviz',
                'gas_map_view.rviz'  # Create this file next
            )],
            output='screen'
        ),
   ])

