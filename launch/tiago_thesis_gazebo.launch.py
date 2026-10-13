import os
import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

# --- Funzione Python che teletrasporta il robot IN MODO DINAMICO ---
def teleport_robot(context, *args, **kwargs):
    x_val = float(LaunchConfiguration('x').perform(context))
    y_val = float(LaunchConfiguration('y').perform(context))
    yaw_val = float(LaunchConfiguration('yaw').perform(context))
    
    # Se le coordinate sono 0, non facciamo nulla
    if x_val == 0.0 and y_val == 0.0 and yaw_val == 0.0:
        return []

    # Calcolo del quaternione
    qz = math.sin(yaw_val / 2.0)
    qw = math.cos(yaw_val / 2.0)
    
    # Messaggio YAML per il teletrasporto
    goal_msg = f"{{state: {{name: 'tiago', pose: {{position: {{x: {x_val}, y: {y_val}, z: 0.1}}, orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}}}}}}}"
    
    # Loop che aspetta esattamente il millisecondo in cui Gazebo è pronto, e poi esegue
    smart_wait_cmd = (
        'until ros2 service list | grep -q "/gazebo/set_entity_state"; do '
        'sleep 0.5; '
        'done; '
        f'ros2 service call /gazebo/set_entity_state gazebo_msgs/srv/SetEntityState "{goal_msg}"'
    )
    
    teleport_cmd = ExecuteProcess(
        cmd=[smart_wait_cmd],
        shell=True,
        output='screen'
    )
    
    # Restituisce l'azione direttamente
    return [teleport_cmd]


def generate_launch_description():
    declare_base_type_cmd = DeclareLaunchArgument('base_type', default_value='pmb2', description='Base type')
    declare_world_name_cmd = DeclareLaunchArgument('world_name', default_value='small_office', description='Gazebo world')
    declare_is_public_sim_cmd = DeclareLaunchArgument('is_public_sim', default_value='false', description='Use public sim')
    
    declare_x_cmd = DeclareLaunchArgument('x', default_value='0.0', description='X spawn coordinate')
    declare_y_cmd = DeclareLaunchArgument('y', default_value='0.0', description='Y spawn coordinate')
    declare_yaw_cmd = DeclareLaunchArgument('yaw', default_value='0.0', description='Yaw orientation in rads')

    base_type = LaunchConfiguration('base_type')
    world_name = LaunchConfiguration('world_name')
    is_public_sim = LaunchConfiguration('is_public_sim')

    tiago_gazebo_launch = os.path.join(
        get_package_share_directory('tiago_gazebo'),
        'launch',
        'tiago_gazebo.launch.py'
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tiago_gazebo_launch),
        launch_arguments={
            'is_public_sim': is_public_sim,
            'base_type': base_type,
            'world_name': world_name
        }.items()
    )

    # Il comando del braccio ha già un'attesa intelligente interna (Waiting for action server...)
    tuck_goal_msg = '"{trajectory: {joint_names: [\'arm_1_joint\', \'arm_2_joint\', \'arm_3_joint\', \'arm_4_joint\', \'arm_5_joint\', \'arm_6_joint\', \'arm_7_joint\'], points: [{positions: [0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0], time_from_start: {sec: 4, nanosec: 0}}]}}"'
    tuck_arm_cmd = ExecuteProcess(
        cmd=['ros2', 'action', 'send_goal', '/arm_controller/follow_joint_trajectory', 'control_msgs/action/FollowJointTrajectory', tuck_goal_msg],
        shell=True,
        output='screen'
    )

    odom_tf_script_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'scripts', 'tools', 'odom_tf_publisher.py')
    odom_tf_bridge_cmd = ExecuteProcess(
        cmd=['python3', odom_tf_script_path, '--ros-args', '-p', 'use_sim_time:=true'],
        condition=LaunchConfigurationEquals('base_type', 'omni_base'),
        output='screen'
    )

    return LaunchDescription([
        declare_base_type_cmd,
        declare_world_name_cmd,
        declare_is_public_sim_cmd,
        declare_x_cmd,
        declare_y_cmd,
        declare_yaw_cmd,
        sim_launch,
        tuck_arm_cmd,
        odom_tf_bridge_cmd,
        OpaqueFunction(function=teleport_robot)
    ])
