import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import LogInfo

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time')
    use_sim_time_str = LaunchConfiguration('use_sim_time')

    # PythonExpression is an evaulation function. So use_sim_time equals true or false based on the expression.
    # if use_sim_time_str is set to either true, 1, or yes then use_sim-time equals true. otherwise its false
    use_sim_time = PythonExpression(["'", use_sim_time_str, "'.lower() in ['true', '1', 'yes']"])


    cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
    #configuration_basename = 'cartographer_sim.lua'
    configuration_basename = 'cartographer_real.lua'

    use_sim_time_message = LogInfo(
        msg=["use_sim_time set to: ", PythonExpression(["str('", use_sim_time_str, "'.lower() in ['true', '1', 'yes'])"])]
    )

    rviz_config = os.path.join(get_package_share_directory('cartographer_slam'), 'rviz', 'mapping.rviz')

    return LaunchDescription([
        use_sim_time_arg,
        use_sim_time_message,
        
        Node(
            package='cartographer_ros', 
            executable='cartographer_node', 
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            name='occupancy_grid_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments = ['-d', rviz_config]
        ), 
    ]) 