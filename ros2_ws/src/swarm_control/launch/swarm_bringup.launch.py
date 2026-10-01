"""One-command bringup: Crazyswarm2 (sim) + rosbridge for Unity + swarm_commander.

    ros2 launch swarm_control swarm_bringup.launch.py
    ros2 launch swarm_control swarm_bringup.launch.py rviz:=True port:=9090
    ros2 launch swarm_control swarm_bringup.launch.py crazyflies_yaml_file:=/path/to/other.yaml
    ros2 launch swarm_control swarm_bringup.launch.py commander:=False
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    swarm_share = get_package_share_directory('swarm_control')
    crazyflie_share = get_package_share_directory('crazyflie')
    rosbridge_share = get_package_share_directory('rosbridge_server')

    args = [
        DeclareLaunchArgument(
            'crazyflies_yaml_file',
            default_value=os.path.join(swarm_share, 'config', 'crazyflies_6.yaml'),
            description='Drone list / initial positions'),
        DeclareLaunchArgument(
            'backend', default_value='sim',
            description='sim | cflib | cpp (sim = simulated Crazyflies)'),
        DeclareLaunchArgument(
            'rviz', default_value='False', description='Also open RViz'),
        DeclareLaunchArgument(
            'teleop', default_value='False',
            description='Joystick teleop (off: no gamepad in sim)'),
        DeclareLaunchArgument(
            'port', default_value='9090', description='rosbridge websocket port'),
        DeclareLaunchArgument(
            'commander', default_value='True', description='Start swarm_commander'),
        DeclareLaunchArgument(
            'commander_params',
            default_value=os.path.join(swarm_share, 'config', 'swarm_commander.yaml'),
            description='swarm_commander parameter file'),
    ]

    crazyswarm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(crazyflie_share, 'launch', 'launch.py')),
        launch_arguments={
            'crazyflies_yaml_file': LaunchConfiguration('crazyflies_yaml_file'),
            'backend': LaunchConfiguration('backend'),
            'rviz': LaunchConfiguration('rviz'),
            'teleop': LaunchConfiguration('teleop'),
            'mocap': 'False',
        }.items(),
    )

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(rosbridge_share, 'launch', 'rosbridge_websocket_launch.xml')),
        launch_arguments={'port': LaunchConfiguration('port')}.items(),
    )

    return LaunchDescription(args + [
        LogInfo(msg=['[swarm_control] backend=', LaunchConfiguration('backend'),
                     ', rosbridge on ws://<host IP>:', LaunchConfiguration('port')]),
        crazyswarm,
        rosbridge,
        Node(
            package='swarm_control',
            executable='swarm_commander',
            name='swarm_commander',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('commander')),
            parameters=[
                LaunchConfiguration('commander_params'),
                {'crazyflies_yaml_file': LaunchConfiguration('crazyflies_yaml_file')},
            ],
        ),
    ])
