# Copyright (c) 2022 PAL Robotics S.L. All rights reserved.
# (Licenza omessa per brevità)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_pal.include_utils import include_launch_py_description


def generate_launch_description():
    # RICEVIAMO il parametro dal launch file principale
    base_type_arg = DeclareLaunchArgument(
        'base_type', default_value='pmb2',
        description='Base type of TIAGo (pmb2 or omni_base)'
    )

    # PASSIAMO il parametro ai controller (così carica quelli per le ruote svedesi)
    default_controllers = include_launch_py_description(
        'tiago_controller_configuration',
        ['launch', 'default_controllers.launch.py'],
        launch_arguments={'base_type': LaunchConfiguration('base_type')}.items())

    play_motion2 = include_launch_py_description(
        'tiago_bringup', ['launch', 'tiago_play_motion2.launch.py'])

    # PASSIAMO il parametro al twist_mux (che gestisce le velocità)
    twist_mux = include_launch_py_description(
        'tiago_bringup', ['launch', 'twist_mux.launch.py'],
        launch_arguments={'base_type': LaunchConfiguration('base_type')}.items())

    # PASSIAMO il parametro al publisher (che costruirà il modello 3D giusto)
    tiago_state_publisher = include_launch_py_description(
        'tiago_description',
        ['launch', 'robot_state_publisher.launch.py'],
        launch_arguments={'base_type': LaunchConfiguration('base_type')}.items())

    # @TODO: robot pose publisher
    # @TODO: tf lookup
    # @TODO: dynamic footprint

    ld = LaunchDescription()
    
    ld.add_action(base_type_arg) # Aggiungiamo l'argomento
    ld.add_action(default_controllers)
    #ld.add_action(play_motion2)
    ld.add_action(twist_mux)
    ld.add_action(tiago_state_publisher)

    return ld
