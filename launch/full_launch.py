import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments for namespace and image processing tracker
        DeclareLaunchArgument('namespace', default_value='bluerov2', description='Namespace for nodes'),
        DeclareLaunchArgument('run_image_processing_tracker', default_value='off', description='Flag to run image processing tracker'),

        # Include the other launch files
        IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('autonomous_rov'), 'launch', 'run_gamepad.launch.py')
            )
        ),
        IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('autonomous_rov'), 'launch', 'run_listener_MIR_joy.launch.py')
            )
        ),
        IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('autonomous_rov'), 'launch', 'run_mavros.launch.py')
            )
        ),
        
        # Namespace for the nodes
        PushRosNamespace(LaunchConfiguration('namespace')),

        # Launch video node
        Node(
            package='autonomous_rov',
            executable='video',
            name='video',
            output='screen'
        ),
        
        # Conditionally launch image_processing_tracker node
        Node(
            package='autonomous_rov',
            executable='image_processing_tracker',
            name='image_processing_tracker',
            output='screen',
            condition=IfCondition(LaunchConfiguration('run_image_processing_tracker'))
        ),
    ])
