import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from tbot3_nav_monitor_msgs.msg import NavigationMetrics


class MetricsVisualizer(Node):

    def __init__(self):
        super().__init__('metrics_visualizer')

        self.get_logger().info('Metrics Visualizer Started')

        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/navigation_metrics_markers',
            10
        )

        self.create_subscription(
            NavigationMetrics,
            '/navigation_metrics',
            self.metrics_callback,
            10
        )

    def metrics_callback(self, msg):

        markers = MarkerArray()

        mode = self.get_navigation_mode(msg)

        text_marker = self.create_text_marker(msg, mode)
        status_marker = self.create_status_marker(msg, mode)
        goal_marker = self.create_goal_marker(msg)
        line_marker = self.create_goal_line_marker(msg)

        markers.markers.append(text_marker)
        markers.markers.append(status_marker)
        markers.markers.append(goal_marker)
        markers.markers.append(line_marker)

        self.marker_publisher.publish(markers)

    def get_navigation_mode(self, msg):

        if msg.goal_reached:
            return 'GOAL REACHED'

        if msg.navigation_status == 2:
            return 'STUCK'

        if msg.corridor_score > 0.5:
            return 'NARROW PASSAGE'

        if msg.environment_complexity > 0.8:
            return 'COMPLEX ENVIRONMENT'

        return 'NORMAL'

    def create_text_marker(self, msg, mode):

        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'navigation_metrics_text'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # marker.pose.position.x = msg.current_x
        # marker.pose.position.y = msg.current_y
        marker.pose.position.x = -2.5
        marker.pose.position.y = 2.5
        marker.pose.position.z = 1.0

        marker.scale.z = 0.18

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.text = (
            f'NAVIGATION STATUS\n'
            f'Mode: {mode}\n'
            f'Goal ID: {msg.goal_id}\n'
            f'Goal reached: {msg.goal_reached}\n'
            f'Execution time: {msg.path_execution_time:.1f} s\n\n'

            f'ROBOT MOTION\n'
            f'Position: x={msg.current_x:.2f}, y={msg.current_y:.2f}\n'
            f'Commanded speed: {msg.commanded_speed:.2f} m/s\n'
            f'Actual speed: {msg.actual_speed:.2f} m/s\n'
            f'Speed error: {msg.speed_error:.2f} m/s\n'
            f'Progress rate: {msg.goal_progress_rate:.4f} m/update\n\n'

            f'GOAL TRACKING\n'
            f'Goal: x={msg.goal_x:.2f}, y={msg.goal_y:.2f}\n'
            f'Distance to goal: {msg.navigation_accuracy:.2f} m\n'
            f'Total path length: {msg.total_distance:.2f} m\n'
            f'Optimal path length: {msg.optimal_path_length:.2f} m\n'
            f'Path efficiency ratio: {msg.obstacle_avoidance_efficiency:.2f}\n\n'

            f'OBSTACLE / ENVIRONMENT\n'
            f'Closest obstacle: {msg.closest_obstacle_distance:.2f} m\n'
            f'Mean obstacle distance: {msg.mean_obstacle_distance:.2f} m\n'
            f'Obstacle density: {msg.obstacle_density:.2f}\n'
            f'Environment complexity: {msg.environment_complexity:.2f}\n\n'

            f'NARROW PASSAGE\n'
            f'Left clearance: {msg.left_clearance:.2f} m\n'
            f'Right clearance: {msg.right_clearance:.2f} m\n'
            f'Front clearance: {msg.front_clearance:.2f} m\n'
            f'Corridor score: {msg.corridor_score:.1f}\n\n'

            f'SIMULATED BATTERY\n'
            f'Battery used: {msg.battery_consumption:.2f}%\n'
            f'Stuck events: {msg.stuck_count}'
        )

        return marker

    def create_status_marker(self, msg, mode):

        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'navigation_status_marker'
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = msg.current_x
        marker.pose.position.y = msg.current_y
        marker.pose.position.z = 0.25

        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25

        if mode == 'STUCK':
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

        elif mode == 'NARROW PASSAGE':
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0

        elif mode == 'COMPLEX ENVIRONMENT':
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0

        elif mode == 'GOAL REACHED':
            marker.color.r = 0.0
            marker.color.g = 0.4
            marker.color.b = 1.0

        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

        marker.color.a = 0.9

        return marker

    def create_goal_marker(self, msg):

        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'navigation_goal_marker'
        marker.id = 2
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = msg.goal_x
        marker.pose.position.y = msg.goal_y
        marker.pose.position.z = 0.05

        marker.scale.x = 0.30
        marker.scale.y = 0.30
        marker.scale.z = 0.10

        marker.color.r = 0.0
        marker.color.g = 0.4
        marker.color.b = 1.0
        marker.color.a = 0.8

        return marker

    def create_goal_line_marker(self, msg):

        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'navigation_goal_line'
        marker.id = 3
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.03

        start = Point()
        start.x = msg.current_x
        start.y = msg.current_y
        start.z = 0.05

        end = Point()
        end.x = msg.goal_x
        end.y = msg.goal_y
        end.z = 0.05

        marker.points.append(start)
        marker.points.append(end)

        marker.color.r = 0.0
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 0.8

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = MetricsVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()