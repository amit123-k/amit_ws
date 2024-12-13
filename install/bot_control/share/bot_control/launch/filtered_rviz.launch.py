from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
import os

def generate_launch_description():

    config_path = os.path.join(get_package_share_path('bot_control'), 'config', 'new_urdf_config.rviz')


    filtered_scan = Node(
        package='bot_control',
        executable='reading_laser.py',
    )
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', config_path]
    )
    return LaunchDescription([
        filtered_scan,
        rviz2
    ])