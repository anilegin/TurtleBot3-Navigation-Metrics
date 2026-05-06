import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker

from tbot3_nav_monitor_msgs.msg import NavigationMetrics


class MetricsVisualizer(Node):

    def __init__(self):
        super().__init__('metrics_visualizer')
        
        self.get_logger().info('Metrics Visualizer Started')

        self.create_subscription(
            NavigationMetrics,
            '/navigation_metrics',
            self.metrics_callback,
            10
        )

        self.marker_publisher = self.create_publisher(
            Marker,
            '/navigation_metrics_marker',
            10
        )

    def metrics_callback(self, msg):

        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'navigation_metrics'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = msg.current_x
        marker.pose.position.y = msg.current_y
        marker.pose.position.z = 1.0

        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.25

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.text = (
            f"Goal ID: {msg.goal_id}\n"
            f"Robot: ({msg.current_x:.2f}, {msg.current_y:.2f})\n"
            f"Goal: ({msg.goal_x:.2f}, {msg.goal_y:.2f})\n"
            f"Distance to goal: {msg.navigation_accuracy:.2f} m\n"
            f"Path length: {msg.total_distance:.2f} m\n"
            f"Optimal path: {msg.optimal_path_length:.2f} m\n"
            f"Efficiency: {msg.obstacle_avoidance_efficiency:.2f}\n"
            f"Obstacle min: {msg.closest_obstacle_distance:.2f} m\n"
            f"Obstacle density: {msg.obstacle_density:.2f}\n"
            f"Battery: {msg.battery_consumption:.3f}\n"
            f"Stuck count: {msg.stuck_count}\n"
            f"Execution time: {msg.path_execution_time:.1f} s\n"
            f"Goal reached: {msg.goal_reached}"
        )

        self.marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = MetricsVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()