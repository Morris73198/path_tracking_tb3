import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    robot_description_path = os.path.join(
        get_package_share_directory('robot_description'))

    fws_robot_sim_path = os.path.join(
        get_package_share_directory('robot_gazebo'))

    # Set gazebo classic resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            os.path.join(fws_robot_sim_path, 'worlds'), ':' +
            os.path.join(fws_robot_sim_path, 'models'), ':' +
            str(Path(robot_description_path).parent.resolve())
        ]
    )

    arguments = LaunchDescription([
        DeclareLaunchArgument('world', default_value='path_tracking',
                              description='Gazebo World'),
    ])

    # 使用經典 Gazebo (需要 .world 文件而不是 .sdf)
    world_file_path = os.path.join(fws_robot_sim_path, 'worlds', 'path_tracking.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'),
                        'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_file_path,
            'verbose': 'true',
            'pause': 'false'
        }.items()
    )

    xacro_file = os.path.join(robot_description_path,
                              'robot',
                              'robot.xacro')

    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})

    robot_desc = doc.toprettyxml(indent='  ')

    params = {'robot_description': robot_desc, 'use_sim_time': use_sim_time}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # 使用 Gazebo classic 的 spawn_entity.py
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'fws_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '3.1415'
        ],
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_forward_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'forward_velocity_controller'],
        output='screen'
    )

    load_forward_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'forward_position_controller'],
        output='screen'
    )

    # 經典 Gazebo 不需要 gz bridge，使用 gazebo_ros 插件
    # 如果需要 TF 和 IMU，在 URDF/xacro 中添加相應的 gazebo_ros 插件

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_forward_velocity_controller,
                         load_forward_position_controller],
            )
        ),
        gazebo_resource_path,
        arguments,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])