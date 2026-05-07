import csv
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node

from tbot3_nav_monitor_msgs.msg import NavigationMetrics


class CsvLogger(Node):

    def __init__(self):
        super().__init__('csv_logger')

        log_dir = Path('/root/tbot3_ws/logs')
        log_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.file_path = log_dir / f'navigation_metrics_{timestamp}.csv'

        self.csv_file = open(self.file_path, mode='w', newline='')
        self.writer = csv.writer(self.csv_file)

        self.writer.writerow([
            'timestamp',
            'total_distance',
            'current_x',
            'current_y',
            'goal_id',
            'goal_x',
            'goal_y',
            'commanded_speed',
            'actual_speed',
            'speed_error',
            'closest_obstacle_distance',
            'mean_obstacle_distance',
            'obstacle_density',
            'goal_progress_rate',
            'environment_complexity',
            'navigation_status',
            'battery_consumption',
            'stuck_count',
            'navigation_accuracy(distance2goal)',
            'obstacle_avoidance_efficiency',
            'left_clearance',
            'right_clearance',
            'front_clearance',
            'corridor_score',
            'optimal_path_length',
            'goal_reached',
            'path_execution_time'
        ])

        self.create_subscription(
            NavigationMetrics,
            '/navigation_metrics',
            self.metrics_callback,
            10
        )

        self.get_logger().info(f'CSV logger started: {self.file_path}')


    def metrics_callback(self, msg):

        current_time = datetime.now().isoformat(timespec='seconds')

        self.writer.writerow([
            current_time,
            msg.total_distance,
            msg.current_x,
            msg.current_y,
            msg.goal_id,
            msg.goal_x,
            msg.goal_y,
            msg.commanded_speed,
            msg.actual_speed,
            msg.speed_error,
            msg.closest_obstacle_distance,
            msg.mean_obstacle_distance,
            msg.obstacle_density,
            msg.goal_progress_rate,
            msg.environment_complexity,
            msg.navigation_status,
            msg.battery_consumption,
            msg.stuck_count,
            msg.navigation_accuracy,
            msg.obstacle_avoidance_efficiency,
            msg.left_clearance,
            msg.right_clearance,
            msg.front_clearance,
            msg.corridor_score,
            msg.optimal_path_length,
            msg.goal_reached,
            msg.path_execution_time
        ])

        self.csv_file.flush()


    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CsvLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()