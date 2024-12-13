import os
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_path, get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    urdf_path = os.path.join(get_package_share_path('bot_description'), 'urdf', 'bot_description.urdf.xacro')
    robot_description= ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    config_path = os.path.join(get_package_share_path('bot_control'), 'config', 'new_urdf_config.rviz')

    world_file = os.path.join(
        get_package_share_directory('bot_world'), 'worlds', 'model.world'

    )

    gazebo = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    world_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo),
        launch_arguments={'world': world_file}.items()  
    )
   

    gazebo_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments= ['-d', config_path]
    )

    return LaunchDescription([
        # gazebo,
        robot_state_publisher_node,
        world_node,
        # rviz2_node,
        gazebo_node
    ])