import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # 使用專案中的 TurtleBot3 描述
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

    # 使用 TurtleBot3 Burger 模型
    xacro_file = os.path.join(robot_description_path,
                              'turtlebot3',
                              'urdf',
                              'turtlebot3_burger.urdf.xacro')

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
            '-entity', 'turtlebot3',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '3.1415'
        ],
    )

    # TurtleBot3 使用 differential drive 插件，不需要額外的控制器
    # 差速驅動插件已經在 TurtleBot3 的 URDF 中配置

    return LaunchDescription([
        gazebo_resource_path,
        arguments,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])