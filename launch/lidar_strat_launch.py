from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_strategy',
            namespace='krabi_ns',
            executable='lidar_strategy_node',
            name='lidar_strat'
            #,remappings=[
            #    ('/input/pose', '/turtlesim1/turtle1/pose'),
            #    ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            #]
        )
    ])