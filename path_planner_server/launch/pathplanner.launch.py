import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import LogInfo

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    use_sim_time_str = LaunchConfiguration('use_sim_time')

    # PythonExpression is an evaulation function. So use_sim_time equals true or false based on the expression.
    # if use_sim_time_str is set to either true, 1, or yes then use_sim-time equals true. otherwise its false
    use_sim_time = PythonExpression(["'", use_sim_time_str, "'.lower() in ['true', '1', 'yes']"])

    use_sim_time_message = LogInfo(
        msg=["use_sim_time set to: ", PythonExpression(["str('", use_sim_time_str, "'.lower() in ['true', '1', 'yes'])"])]
    )


    rviz_config = os.path.join(get_package_share_directory('path_planner_server'), 'rviz', 'map_display.rviz')
    
    controller_yaml_sim = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller_sim.yaml')
    bt_navigator_yaml_sim = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_navigator_sim.yaml')
    planner_yaml_sim = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server_sim.yaml')
    recovery_yaml_sim = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery_sim.yaml')
    
    
    controller_yaml_real = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller_real.yaml')
    bt_navigator_yaml_real = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_navigator_real.yaml')
    planner_yaml_real = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server_real.yaml')
    recovery_yaml_real = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery_real.yaml')
    

    controller_config = PythonExpression([
        f"'{controller_yaml_sim}' if ", use_sim_time, f" == True else '{controller_yaml_real}'"
    ])
    bt_navigator_config = PythonExpression([
        f"'{bt_navigator_yaml_sim}' if ", use_sim_time, f" == True else '{bt_navigator_yaml_real}'"
    ])
    planner_config = PythonExpression([
        f"'{planner_yaml_sim}' if ", use_sim_time, f" == True else '{planner_yaml_real}'"
    ])
    recovery_config = PythonExpression([
        f"'{recovery_yaml_sim}' if ", use_sim_time, f" == True else '{recovery_yaml_real}'"
    ])

    cmd_topic = PythonExpression([
        "'/diffbot_base_controller/cmd_vel_unstamped' if ", use_sim_time, " == True else '/cmd_vel'"
    ])

    return LaunchDescription([
        use_sim_time_arg,
        use_sim_time_message,

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, controller_config],
            remappings=[('cmd_vel', cmd_topic)],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, planner_config]
        ),
            
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            parameters=[{'use_sim_time': use_sim_time}, recovery_config],
            output='screen'),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, bt_navigator_config]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': [
                                        'planner_server',
                                        'controller_server',
                                        'recoveries_server',
                                        'bt_navigator'
                        ]}
            ]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments = ['-d', rviz_config]
        ),       
    ])