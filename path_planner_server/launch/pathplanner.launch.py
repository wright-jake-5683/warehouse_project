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

    controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller_sim.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_navigator_sim.yaml')
    planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server_sim.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery_sim.yaml')

    """
    controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller_real.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_navigator_real.yaml')
    planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server_real.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery_real.yaml')
    """

    return LaunchDescription([
        use_sim_time_arg,
        use_sim_time_message,

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml],
            remappings=[('cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]),
            
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            parameters=[recovery_yaml],
            output='screen'),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': [
                                        'planner_server',
                                        'controller_server',
                                        'recoveries_server',
                                        'bt_navigator'
                        ]}
            ]),      
    ])