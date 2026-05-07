from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


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

    adaptive_controller_config = PathJoinSubstitution([
        FindPackageShare('tbot3_nav_monitor'),
        'config',
        'adaptive_controller.yaml'
    ])

    adaptive_controller_node = Node(
        package='tbot3_nav_monitor',
        executable='adaptive_controller',
        name='adaptive_controller',
        output='screen',
        parameters=[adaptive_controller_config]
    )

    ml_predictor_node = Node(
        package='tbot3_nav_monitor',
        executable='ml_predictor',
        name='ml_predictor',
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
        ml_predictor_node,
        metrics_visualizer_node
    ])