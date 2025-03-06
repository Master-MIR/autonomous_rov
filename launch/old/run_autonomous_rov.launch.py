from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

    
def generate_launch_description():

    return LaunchDescription([
    	Node(package='autonomous_rov',
    	           executable='listenerMIR',
    	           output='screen'),
  	   
    	IncludeLaunchDescription(
    	    PythonLaunchDescriptionSource([
    	            FindPackageShare("teleop_twist_joy"), '/launch', '/teleop-launch.py'])
    	    ,
    	    launch_arguments={'joy_config': 'xbox', 'joy_dev': '/dev/input/js0'}.items())
    	])

