import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

from tbot3_nav_monitor_msgs.msg import NavigationMetrics

import math
import time


class MetricsCollector(Node):

    def __init__(self):
        super().__init__('metrics_collector')
        
        self.get_logger().info("Metrics are being monitored.")
        
        ## tf for converting odom into map frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        ## odom topic subscription
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.odom_subscription  # prevent unused variable warning
        
        self.start_x = None
        self.start_y = None
        
        self.prev_x = None
        self.prev_y = None

        self.current_x = 0.0
        self.current_y = 0.0
        
        self.odom_linear_velocity = 0.0
        self.velocity_error = 0.0

        self.total_distance = 0.0
        
        ## cmd vel topic subscription
        self.cmd_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.cmd_subscription
        
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        ## scan topic subscription
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.scan_subscription
        
        self.closest_obstacle_distance = float('inf')
        # more data for path planning and to use for ai model
        self.mean_obstacle_distance = 0.0
        self.obstacle_density = 0.0
        self.environment_complexity = 0.0
        
        ## battery metrics
        self.battery_consumption = 0.0
        self.battery_rate_per_meter = 0.05  # 5% battery per meter as an example
        
        ## stuck behavior
        self.stuck_count = 0
        self.stuck_counter = 0
        self.stuck_counter_threshold = 3 # if robot is not moving 3 cons times then defined stuck
        
        ## nav accuracy 
        # for now i assumed goal is origin but can change later on.
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_id = -1
        
        ## nav2 goal subscription
        self.goal_subscription = self.create_subscription(
            Path,
            '/plan',
            self.goal_callback,
            10
        )
        self.goal_subscription

        self.navigation_accuracy = 0.0
        #path planning metric
        self.previous_navigation_accuracy = None
        self.goal_progress_rate = 0.0
        
        ## obstacle avoidance 
        self.optimal_path_length = 0.0
        self.obstacle_avoidance_efficiency = 0.0
        
        ## path exec time
        self.goal_tolerance = 0.15 #within 15 cm of goal is considered success

        self.navigation_started = False
        self.navigation_finished = False
        
        # 0: navigating 1: goal reached 2: stuck
        self.navigation_status = 0

        self.start_time = None
        self.path_execution_time = 0.0
        
        ## narrow passage detection
        self.left_clearance = float("inf")
        self.right_clearance = float("inf")
        self.front_clearance = float("inf")
        self.corridor_score = 0.0
        
        ## publisher for navigation metrics
        self.metrics_publisher = self.create_publisher(
            NavigationMetrics,
            '/navigation_metrics',
            10
        )

        self.create_timer(1.0, self.publish_metrics)



    def odom_callback(self, msg):

        self.update_robot_pose()
        
        
        self.odom_linear_velocity = msg.twist.twist.linear.x
        # to avoid tiny movements being counted as distance due to noise
        if abs(self.odom_linear_velocity) < 0.001:
            self.odom_linear_velocity = 0.0
            
        self.velocity_error = abs(
            self.linear_velocity - self.odom_linear_velocity
        )  

        # first message
        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            
        if self.prev_x is None:
            self.prev_x = self.current_x
            self.prev_y = self.current_y
            return

        # euclidean distance
        distance = math.sqrt(
            (self.current_x - self.prev_x) ** 2 +
            (self.current_y - self.prev_y) ** 2
        )
        # euclidean distance from start to goal 
        self.optimal_path_length = math.sqrt(
            (self.goal_x - self.start_x) ** 2 +
            (self.goal_y - self.start_y) ** 2
        )
        
        # to avoid tiny movements being counted as distance due to noise
        if distance < 0.001:
            distance = 0.0

        self.total_distance += distance
        
        if self.optimal_path_length > 0.001:
            self.obstacle_avoidance_efficiency = self.total_distance / self.optimal_path_length
        else:
            self.obstacle_avoidance_efficiency = 0.0
        
        # sample battery consumption based on distance traveled
        self.battery_consumption = (
            self.total_distance * self.battery_rate_per_meter
        )
        
        #nav accuracy
        self.navigation_accuracy = math.sqrt(
            (self.goal_x - self.current_x) ** 2 +
            (self.goal_y - self.current_y) ** 2
        )
        
        if self.previous_navigation_accuracy is not None:
    
            self.goal_progress_rate = (
                self.previous_navigation_accuracy
                - self.navigation_accuracy
            )

        self.previous_navigation_accuracy = (
            self.navigation_accuracy
        )
        
        self.update_path_execution_time() # checking if goal reached

        self.prev_x = self.current_x
        self.prev_y = self.current_y     

        self.get_logger().info(
            f"Current Position x: {self.current_x:.2f}, y: {self.current_y:.2f}"
        )

        self.get_logger().info(
            f"Total Distance Travelled: {self.total_distance:.2f} meters"
        )
        self.get_logger().info(
            f"Battery Consumption: {self.battery_consumption:.2f}%"
        )
        
    def update_robot_pose(self):
    
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            self.current_x = transform.transform.translation.x
            self.current_y = transform.transform.translation.y

        except TransformException:
            return
        
    def cmd_vel_callback(self, msg):
    
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

        self.get_logger().info(
            f"Velocity linear: {self.linear_velocity:.2f}, "
            f"angular: {self.angular_velocity:.2f}"
        )
        
    def scan_callback(self, msg):
    
        valid_ranges = [
            r for r in msg.ranges
            if not math.isinf(r) and not math.isnan(r)
        ]

        if len(valid_ranges) == 0:
            return

        self.closest_obstacle_distance = min(valid_ranges)
        
        self.mean_obstacle_distance = (
            sum(valid_ranges) / len(valid_ranges)
        )
        
        close_obstacles = [
            r for r in valid_ranges
            if r < 1.0
        ]

        self.obstacle_density = (
            len(close_obstacles) / len(valid_ranges)
        )
        
        self.environment_complexity = (
            self.obstacle_density *
            (1.0 / max(self.mean_obstacle_distance, 0.001))
        )
        
        ## narrow passages
        n = len(msg.ranges)

        def clean(vals):
            return [r for r in vals if not math.isinf(r) and not math.isnan(r)]

        front = clean(msg.ranges[0:20] + msg.ranges[-20:])
        left = clean(msg.ranges[60:120])
        right = clean(msg.ranges[-120:-60])

        self.front_clearance = min(front) if front else float("inf")
        self.left_clearance = min(left) if left else float("inf")
        self.right_clearance = min(right) if right else float("inf")

        # side_close = self.left_clearance < 0.75 and self.right_clearance < 0.75
        # front_open = self.front_clearance > 0.35

        # self.corridor_score = 1.0 if side_close and front_open else 0.0
        
        one_side_very_close = (
            self.left_clearance < 0.45 or
            self.right_clearance < 0.45
        )

        both_sides_close = (
            self.left_clearance < 0.90 and
            self.right_clearance < 0.90
        )

        self.corridor_score = 1.0 if one_side_very_close or both_sides_close else 0.0
        

        self.get_logger().info(
            f"Closest Obstacle Distance: "
            f"{self.closest_obstacle_distance:.2f} meters"
        )
        
    def update_stuck_count(self):
    
        robot_cmd_to_move = abs(self.linear_velocity) > 0.05
        robot_not_moving = abs(self.odom_linear_velocity) < 0.01

        if robot_cmd_to_move and robot_not_moving:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0

        if self.stuck_counter == self.stuck_counter_threshold:
            self.stuck_count += 1
            self.get_logger().warn(
                f"Possible stuck event detected. Count: {self.stuck_count}"
            )
            
    def update_path_execution_time(self):
    
        if self.navigation_finished:
            return

        if not self.navigation_started:
            self.navigation_started = True
            self.start_time = time.time()
            return
        
        self.path_execution_time = time.time() - self.start_time

        if self.navigation_accuracy < self.goal_tolerance:
            self.navigation_finished = True
            self.get_logger().info(
                f"Goal reached. Execution time: {self.path_execution_time:.2f} seconds"
            )
        if self.navigation_finished:
            self.navigation_status = 1
        elif self.stuck_counter >= self.stuck_counter_threshold:
            self.navigation_status = 2
        else:
            self.navigation_status = 0
            
    def goal_callback(self, msg):
    
        if len(msg.poses) == 0:
            return

        final_pose = msg.poses[-1]

        new_goal_x = final_pose.pose.position.x
        new_goal_y = final_pose.pose.position.y

        # ignoring small planner updates to same goal
        goal_changed = (
            abs(new_goal_x - self.goal_x) > 0.05 or
            abs(new_goal_y - self.goal_y) > 0.05
        )

        if not goal_changed:
            return

        self.goal_x = new_goal_x
        self.goal_y = new_goal_y
        self.goal_id += 1

        # reset navigation session
        self.navigation_finished = False
        self.navigation_started = False

        self.total_distance = 0.0
        self.path_execution_time = 0.0

        self.start_x = self.current_x
        self.start_y = self.current_y

        self.prev_x = self.current_x
        self.prev_y = self.current_y

        self.get_logger().info(
            f"New navigation goal received: goal_id={self.goal_id}, x={self.goal_x:.2f}, y={self.goal_y:.2f}"
        )

    def publish_metrics(self):
    
        msg = NavigationMetrics()
        self.update_stuck_count()

        msg.total_distance = float(self.total_distance)
        
        msg.current_x = float(self.current_x)
        msg.current_y = float(self.current_y)
        
        msg.goal_id = int(self.goal_id)
        msg.goal_x = float(self.goal_x)
        msg.goal_y = float(self.goal_y)
        
        msg.commanded_speed = float(self.linear_velocity)
        msg.actual_speed = float(self.odom_linear_velocity)
        msg.speed_error = float(self.velocity_error)
        
        msg.battery_consumption = float(self.battery_consumption)
        
        msg.navigation_accuracy = float(self.navigation_accuracy)
        msg.goal_progress_rate = float(self.goal_progress_rate)
        
        msg.closest_obstacle_distance = float(self.closest_obstacle_distance)
        msg.mean_obstacle_distance = float(self.mean_obstacle_distance)
        msg.obstacle_density = float(self.obstacle_density)
        msg.environment_complexity = float(self.environment_complexity)
        msg.obstacle_avoidance_efficiency = float(self.obstacle_avoidance_efficiency)
        msg.stuck_count = int(self.stuck_count)
        
        msg.optimal_path_length = float(self.optimal_path_length)
        msg.path_execution_time = float(self.path_execution_time) if self.path_execution_time is not None else 0.0  
        msg.goal_reached = bool(self.navigation_finished)
        
        msg.left_clearance = float(self.left_clearance)
        msg.right_clearance = float(self.right_clearance)
        msg.front_clearance = float(self.front_clearance)
        msg.corridor_score = float(self.corridor_score)

        self.metrics_publisher.publish(msg)

        self.get_logger().info(
            f"robot=({self.current_x:.2f}, {self.current_y:.2f}) | "
            f"goal=({self.goal_id}: {self.goal_x:.2f}, {self.goal_y:.2f}) | "
            f"distance={self.total_distance:.2f} m | "
            f"vel_cmd={self.linear_velocity:.2f} | "
            f"vel_actual={self.odom_linear_velocity:.2f} | "
            f"vel_error={self.velocity_error:.2f} | "
            f"obstacle_distance={self.closest_obstacle_distance:.2f} m | "
            f"obstacle_avoidance_efficiency={self.obstacle_avoidance_efficiency:.2f} | "
            f"straight_path_length={self.optimal_path_length:.2f} m | "
            f"navigation_accuracy(distance2goal)={self.navigation_accuracy:.2f} m | "
            f"battery={self.battery_consumption:.3f} | "
            f"stuck={self.stuck_count} | "
            f"execution_time={self.path_execution_time:.2f} s | "
            f"goal_reached={self.navigation_finished}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = MetricsCollector()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()