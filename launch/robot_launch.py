import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('p3dxros2')
    robot_description_path = os.path.join(package_dir, 'resource', 'p3at.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'p3at.wbt'),
    )

    my_robot_driver = WebotsController(
        robot_name='P3AT',
        parameters=[{'robot_description': robot_description_path}]
    )

    return LaunchDescription([
        # TFs estáticos
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=['0', '0', '0.15', '0', '0', '0', 'base_footprint', 'base_link'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0.1', '0', '0.20', '0', '0', '0', 'base_link', 'laser_link'],
        ),

        # Webots
        webots,
        my_robot_driver,

        # SLAM Toolbox — construye el mapa
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_footprint',
                'scan_topic': '/scan',
                'mode': 'mapping',
                'max_laser_range': 15.0,
                'resolution': 0.05,
                'minimum_travel_distance': 0.1,
                'minimum_travel_heading': 0.1,
                'transform_timeout': 0.5,
                'tf_buffer_duration': 30.0,
                'transform_publish_period': 0.02,
                'map_start_at_dock': True,
            }]
        ),

        # Navegador autónomo — usa /odom + /map → publica /cmd_vel
        Node(
            package='p3dxros2',
            executable='navegador',
            name='navegador_autonomo',
            output='screen',
        ),

        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
