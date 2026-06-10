import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_description = "map_server"

    map_file_arg = DeclareLaunchArgument('map_file')
    map_file_f = LaunchConfiguration('map_file')

    map_file = PathJoinSubstitution([
        FindPackageShare(package_description),
        'config',
        map_file_f
    ])

    rviz_config = os.path.join(get_package_share_directory('localization_server'), 'rviz', 'map_display.rviz')
    amcl_config_sim = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config_sim.yaml')
    amcl_config_real = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config_real.yaml')

    filters_sim_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'filters.yaml')
    filters_real_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'filters_real.yaml')


    amcl_config = PythonExpression([
        f"'{amcl_config_sim}' if '", map_file_f, f"' == 'warehouse_map_keepout_sim.yaml' else '{amcl_config_real}'"
    ])

    odom_frame = PythonExpression([
        "'odom' if '", map_file_f, "' == 'warehouse_map_keepout_sim.yaml' else 'robot_odom'"
    ])
    
    sim_time = PythonExpression([
        "'", map_file_f, "'== 'warehouse_map_sim.yaml'"
    ])

    filter_used = PythonExpression([
        f"'{filters_sim_yaml}' if '", map_file_f, f"' == 'warehouse_map_keepout_sim.yaml' else '{filters_real_yaml}'"
    ])

    return LaunchDescription([
        map_file_arg,

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            emulate_tty=True,
            arguments=['0', '0', '0', '0', '0', '0', 'map', odom_frame]
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': sim_time}, 
                        {'yaml_filename': map_file}]
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            emulate_tty=True,
            parameters=[filter_used]
        ),

        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            emulate_tty=True,
            parameters=[filter_used]
        ),


         Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{'use_sim_time': True},
                       amcl_config]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server',
                                        'amcl',
                                        'filter_mask_server',
                                        'costmap_filter_info_server'
                        ]}
            ]),

         Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': sim_time}],
            arguments = ['-d', rviz_config]
        ),            
    ])