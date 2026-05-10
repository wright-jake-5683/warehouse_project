import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_description = "map_server"

    map_file_arg = DeclareLaunchArgument('map_file', default_value="warehouse_map_sim.yaml")
    map_file_f = LaunchConfiguration('map_file')

    map_file = PathJoinSubstitution([
        FindPackageShare(package_description),
        'config',
        map_file_f
    ])

    rviz_config = os.path.join(get_package_share_directory('map_server'), 'rviz', 'map_display.rviz')

    return LaunchDescription([
        map_file_arg,

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename': map_file} 
            ]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}
            ]),

         Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments = ['-d', rviz_config]
        ),            
    ])