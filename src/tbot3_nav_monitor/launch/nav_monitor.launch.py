from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    metrics_collector_node = Node(
        package='tbot3_nav_monitor',
        executable='metrics_collector',
        name='metrics_collector',
        output='screen'
    )

    csv_logger_node = Node(
        package='tbot3_nav_monitor',
        executable='csv_logger',
        name='csv_logger',
        output='screen'
    )
    
    adaptive_controller_node = Node(
        package='tbot3_nav_monitor',
        executable='adaptive_controller',
        name='adaptive_controller',
        output='screen'
    )
    
    metrics_visualizer_node = Node(
        package='tbot3_nav_monitor',
        executable='metrics_visualizer',
        name='metrics_visualizer',
        output='screen'
    )

    return LaunchDescription([
        metrics_collector_node,
        csv_logger_node,
        adaptive_controller_node,
        metrics_visualizer_node
    ])