import random

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid


class BatchGoalSender(Node):

    def __init__(self):
        super().__init__('batch_goal_sender')

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.num_goals = 50
        self.goal_index = 0

        self.wait_time_between_goals = 2.0
        self.goal_timeout = 40

        self.map_msg = None
        self.current_goal_handle = None
        self.goal_start_time = None
        self.goal_active = False

        self.delay_timer = None

        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )

        self.create_timer(1.0, self.check_goal_timeout)

        self.get_logger().info('Waiting for Nav2 action server...')
        self.client.wait_for_server()
        self.get_logger().info('Nav2 action server ready.')

        self.get_logger().info('Waiting for /map...')
        self.map_wait_timer = self.create_timer(1.0, self.try_start)

    def map_callback(self, msg):
        self.map_msg = msg

    def try_start(self):
        if self.map_msg is None:
            self.get_logger().info('Still waiting for /map...')
            return

        self.map_wait_timer.cancel()
        self.get_logger().info('Map received. Starting random goal batch.')
        self.send_next_goal()

    def sample_free_goal_from_map(self):
        width = self.map_msg.info.width
        height = self.map_msg.info.height
        resolution = self.map_msg.info.resolution
        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y

        for _ in range(2000):
            cell_x = random.randint(0, width - 1)
            cell_y = random.randint(0, height - 1)

            index = cell_y * width + cell_x
            value = self.map_msg.data[index]

            if value != 0:
                continue

            map_x = origin_x + (cell_x + 0.5) * resolution
            map_y = origin_y + (cell_y + 0.5) * resolution

            return map_x, map_y

        return None, None

    def send_next_goal(self):
        if self.goal_active:
            return

        if self.goal_index >= self.num_goals:
            self.get_logger().info(f'All {self.num_goals} random goals completed.')
            rclpy.shutdown()
            return

        x, y = self.sample_free_goal_from_map()

        if x is None or y is None:
            self.get_logger().warn('Could not sample free goal from map.')
            self.goal_index += 1
            self.start_delay_timer()
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.goal_active = True
        self.current_goal_handle = None
        self.goal_start_time = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info(
            f'Sending goal {self.goal_index + 1}/{self.num_goals}: '
            f'x={x:.2f}, y={y:.2f}'
        )

        future = self.client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected.')
            self.finish_current_goal()
            return

        self.current_goal_handle = goal_handle
        self.get_logger().info('Goal accepted.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        if not self.goal_active:
            return

        result = future.result()

        self.get_logger().info(
            f'Goal {self.goal_index + 1} finished with status: {result.status}'
        )

        self.finish_current_goal()

    def check_goal_timeout(self):
        if not self.goal_active:
            return

        if self.goal_start_time is None:
            return

        elapsed = (
            self.get_clock().now().nanoseconds / 1e9
            - self.goal_start_time
        )

        if elapsed < self.goal_timeout:
            return

        self.get_logger().warn(
            f'Goal {self.goal_index + 1} timed out after {elapsed:.1f}s. Cancelling.'
        )

        if self.current_goal_handle is not None:
            future = self.current_goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_done_callback)
        else:
            self.finish_current_goal()

    def cancel_done_callback(self, future):
        if not self.goal_active:
            return

        self.get_logger().warn(
            f'Goal {self.goal_index + 1} cancelled due to timeout.'
        )

        self.finish_current_goal()

    def finish_current_goal(self):
        if not self.goal_active:
            return

        self.goal_active = False
        self.current_goal_handle = None
        self.goal_start_time = None

        self.goal_index += 1
        self.start_delay_timer()

    def start_delay_timer(self):
        if self.delay_timer is not None:
            self.delay_timer.cancel()
            self.delay_timer = None

        self.delay_timer = self.create_timer(
            self.wait_time_between_goals,
            self.delayed_next_goal
        )

    def delayed_next_goal(self):
        if self.delay_timer is not None:
            self.delay_timer.cancel()
            self.delay_timer = None

        self.send_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = BatchGoalSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()