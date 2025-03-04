#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    pinger_node = Node(
        package="autonomous_rov",
        executable="pinger_node",
        output='screen',
	arguments=['--udp', '192.168.2.2:9090']
    )

    ld.add_action(pinger_node)

    return ld
