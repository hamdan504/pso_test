from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('pso_visual')
    world_path = os.path.join(pkg_path, 'worlds', 'pso_world.world')

    # Launch Gazebo first
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', world_path,
            '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'  # Important: need factory.so for spawn_entity
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,

        # Wait for Gazebo to start before spawning
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=gazebo,
                on_start=[
                    TimerAction(
                        period=5.0,
                        actions=[
                            ExecuteProcess(
                                cmd=[
                                    'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                                    '-entity', 'turtlebot3_waffle',
                                    '-x', '1.0', '-y', '1.0', '-z', '0.1',
                                    '-topic', '/robot_description'
                                ],
                                output='screen'
                            )
                        ]
                    )
                ]
            )
        ),

        # PSO Visual Node
        Node(
            package='pso_visual',
            executable='pso_visual_node',
            name='pso_visual_node',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }]
        )
    ])

