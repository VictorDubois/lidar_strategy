from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

def generate_launch_description():
    use_sim_time_value = LaunchConfiguration('use_sim_time')

    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True'
    )
    return LaunchDescription([
        use_sim_time_launch_arg,
        Node(
            package='lidar_strategy',
            namespace='krabi_ns',
            executable='lidar_strategy_node',
            name='lidar_strat',
            parameters=[{"use_sim_time": use_sim_time_value}],
            respawn=True,
            respawn_delay=5.0
            #,prefix=['gdbserver localhost:3000']
        )
    ])