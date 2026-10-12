# Copyright (c) 2022 PAL Robotics S.L. All rights reserved.
# (Licenza omessa per brevità)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Dichiariamo l'argomento per evitare warning da ROS 2
    base_type_arg = DeclareLaunchArgument(
        'base_type', default_value='pmb2',
        description='Base type of TIAGo (pmb2 or omni_base)'
    )

    spawn_coordinates = [0.0, -1.3, 0.0]

    model_name = DeclareLaunchArgument(
        'model_name', default_value='tiago',
        description='Gazebo model name'
    )

    tiago_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', LaunchConfiguration(
                                       'model_name'),
                                   '-x', str(spawn_coordinates[0]),
                                   '-y', str(spawn_coordinates[1]),
                                   '-z', str(spawn_coordinates[2]),
                                   ],
                        output='screen')

    ld = LaunchDescription()

    ld.add_action(base_type_arg) # Aggiungiamo al launch description
    ld.add_action(model_name)
    ld.add_action(tiago_entity)

    return ld
