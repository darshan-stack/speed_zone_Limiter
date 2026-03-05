import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('speed_zone_limiter')
    zones_yaml = os.path.join(pkg_share, 'config', 'zones.yaml')

    # ------------------------------------------------------------------ #
    # 1. Speed-zone limiter node
    #    - Reads /cmd_vel   (move_base / nav2 output remapped here)
    #    - Publishes /cmd_vel_safe
    # ------------------------------------------------------------------ #
    speed_limiter_node = Node(
        package='speed_zone_limiter',
        executable='speed_zone_limiter_node',
        name='speed_zone_limiter',
        parameters=[zones_yaml],
        remappings=[
            # move_base/Nav2 publishes to /cmd_vel; remap that topic into our node
            ('/cmd_vel', '/cmd_vel_raw'),
        ],
        output='screen',
    )

    # ------------------------------------------------------------------ #
    # 2. Relay: publish /cmd_vel_safe as the real /cmd_vel that the
    #    robot driver actually consumes.
    #    Uses the built-in topic_tools relay node.
    # ------------------------------------------------------------------ #
    relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        arguments=['/cmd_vel_safe', '/cmd_vel'],
        output='screen',
    )

    # ------------------------------------------------------------------ #
    # 3. Nav2 / TurtleBot3 navigation (adjust path as needed)
    #    We remap nav2's cmd_vel output to /cmd_vel_raw so it flows
    #    through our limiter before reaching the robot.
    # ------------------------------------------------------------------ #
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_navigation2'),
                'launch',
                'navigation2.launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
        }.items(),
    )

    return LaunchDescription([
        nav2_launch,
        speed_limiter_node,
        relay_node,
    ])
