import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
import math

from nav2_msgs.srv import ClearEntireCostmap

from tbot3_nav_monitor_msgs.msg import NavigationMetrics


class AdaptiveController(Node):

    def __init__(self):
        super().__init__('adaptive_controller')
        
        self.get_logger().info('Adaptive Controller Started')

        self.controller_node = self.declare_parameter(
            'controller_node',
            '/controller_server'
        ).value
        
        ## adapting vel if robot often stucks
        self.normal_max_velocity = self.declare_parameter(
            'normal_max_velocity',
            0.26
        ).value
        self.reduced_max_velocity = self.declare_parameter(
            'reduced_max_velocity',
            0.10
        ).value

        self.stuck_threshold = self.declare_parameter(
            'stuck_threshold',
            3
        ).value
        self.velocity_reduced = False
        
        # goal tolerance when nav accuracy is poor
        self.normal_goal_tolerance = self.declare_parameter(
            'normal_goal_tolerance',
            0.15
        ).value
        self.relaxed_goal_tolerance = self.declare_parameter(
            'relaxed_goal_tolerance',
            0.30
        ).value

        self.goal_struggle_threshold = self.declare_parameter(
            'goal_struggle_threshold',
            8
        ).value
        self.goal_struggle_counter = 0
        self.goal_tolerance_relaxed = False
        
        ## path planner adjustment
        self.normal_inflation_radius = self.declare_parameter(
            'normal_inflation_radius',
            0.30
        ).value
        self.conservative_inflation_radius = self.declare_parameter(
            'conservative_inflation_radius',
            0.60
        ).value

        self.bad_efficiency_threshold = self.declare_parameter(
            'bad_efficiency_threshold',
            5
        ).value
        self.bad_efficiency_counter = 0

        self.conservative_mode_enabled = False
        
        ## local costmap update based on env compelxity
        self.normal_cost_scaling_factor = self.declare_parameter(
            'normal_cost_scaling_factor',
            3.0
        ).value
        self.complex_cost_scaling_factor = self.declare_parameter(
            'complex_cost_scaling_factor',
            10.0
        ).value

        self.complex_environment_threshold = self.declare_parameter(
            'complex_environment_threshold',
            5
        ).value
        self.complex_environment_counter = 0

        self.complex_environment_enabled = False
        
        ## i add this since current one lacks moving through narrow passages
        self.narrow_passage_inflation_radius = self.declare_parameter(
            'narrow_passage_inflation_radius',
            0.35
        ).value

        self.narrow_passage_cost_scaling_factor = self.declare_parameter(
            'narrow_passage_cost_scaling_factor',
            0.5
        ).value

        self.narrow_passage_velocity = self.declare_parameter(
            'narrow_passage_velocity',
            0.05
        ).value

        self.narrow_passage_threshold = self.declare_parameter(
            'narrow_passage_threshold',
            2
        ).value

        self.narrow_passage_counter = 0
        self.narrow_passage_enabled = False
        
        self.last_distance_to_goal = None
        self.narrow_stuck_counter = 0
        self.narrow_stuck_threshold = self.declare_parameter(
            'narrow_stuck_threshold',
            8
        ).value
        
        self.recovery_max_velocity = self.declare_parameter(
            'recovery_max_velocity',
            0.035
        ).value

        self.recovery_min_velocity = self.declare_parameter(
            'recovery_min_velocity',
            0.01
        ).value

        self.narrow_max_theta_velocity = self.declare_parameter(
            'narrow_max_theta_velocity',
            0.10
        ).value

        self.recovery_max_theta_velocity = self.declare_parameter(
            'recovery_max_theta_velocity',
            0.08
        ).value

        self.recovery_inflation_radius = self.declare_parameter(
            'recovery_inflation_radius',
            0.25
        ).value

        self.recovery_cost_scaling_factor = self.declare_parameter(
            'recovery_cost_scaling_factor',
            0.25
        ).value
        
        ## wall head turning problem
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        self.use_wall_escape = self.declare_parameter(
            'use_wall_escape',
            True
        ).value

        self.original_goal = None
        self.active_goal_id = -1
        self.escape_goal_active = False

        self.wall_escape_forward_distance = self.declare_parameter(
            'wall_escape_forward_distance',
            0.30
        ).value

        self.wall_escape_lateral_offset = self.declare_parameter(
            'wall_escape_lateral_offset',
            0.25
        ).value

        self.wall_escape_close_threshold = self.declare_parameter(
            'wall_escape_close_threshold',
            0.45
        ).value

        self.wall_escape_progress_threshold = self.declare_parameter(
            'wall_escape_progress_threshold',
            0.002
        ).value
        
        self.wall_escape_clear_threshold = self.declare_parameter(
            'wall_escape_clear_threshold',
            0.70
        ).value

        self.wall_escape_min_interval = self.declare_parameter(
            'wall_escape_min_interval',
            6.0
        ).value

        self.last_wall_escape_time = 0.0

        self.escape_mode_enabled = False

        self.local_clear_client = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap'
        )

        self.global_clear_client = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap'
        )

        # param update 
        self.param_client = self.create_client(
            SetParameters,
            f'{self.controller_node}/set_parameters'
        )
        
        # cost map param update
        self.local_costmap_client = self.create_client(
            SetParameters,
            '/local_costmap/local_costmap/set_parameters'
        )

        self.global_costmap_client = self.create_client(
            SetParameters,
            '/global_costmap/global_costmap/set_parameters'
        )

        
        ## our custom navigation metrics
        self.create_subscription(
            NavigationMetrics,
            '/navigation_metrics',
            self.metrics_callback,
            10
        )


    def metrics_callback(self, msg):

        if not self.param_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn('Controller parameter service not available yet.')
            return
        
        self.update_goal_tolerance(msg)
        self.update_narrow_passage_mode(msg)

        # if self.narrow_passage_enabled or self.escape_mode_enabled:
        #     return

        if msg.stuck_count >= self.stuck_threshold and not self.velocity_reduced:
            self.set_max_velocity(self.reduced_max_velocity)
            self.velocity_reduced = True
            self.get_logger().warn(
                f'Stuck count high. Reducing max velocity to {self.reduced_max_velocity}'
            )

        elif msg.stuck_count < self.stuck_threshold and self.velocity_reduced:
            self.set_max_velocity(self.normal_max_velocity)
            self.velocity_reduced = False
            self.get_logger().info(
                f'Stuck count normal. Restoring max velocity to {self.normal_max_velocity}'
            )
        
        self.update_conservative_planning(msg)
        self.update_environment_complexity(msg)
        
    def update_goal_tolerance(self, msg):
        """
        If robot is near 50cm radius of the goal but not reaching it while moving slowly we increase the goal tolerance
        old (default): 0.15m
        new : 0.30m
        """
        
        near_goal = msg.navigation_accuracy < 0.50
        not_reached = not msg.goal_reached
        moving_slowly = abs(msg.actual_speed) < 0.10
        trying_to_finish = abs(msg.commanded_speed) < 0.15

        struggling_near_goal = (
            near_goal and
            not_reached and
            moving_slowly and
            trying_to_finish
        )

        if struggling_near_goal:
            self.goal_struggle_counter += 1
        else:
            self.goal_struggle_counter = 0

        if (
            self.goal_struggle_counter >= self.goal_struggle_threshold
            and not self.goal_tolerance_relaxed
        ):
            self.set_goal_tolerance(self.relaxed_goal_tolerance)
            self.goal_tolerance_relaxed = True

            self.get_logger().warn(
                f'Robot is struggling near goal. Increasing goal tolerance to {self.relaxed_goal_tolerance}'
            )

        if msg.goal_reached and self.goal_tolerance_relaxed:
            self.set_goal_tolerance(self.normal_goal_tolerance)
            self.goal_tolerance_relaxed = False
            self.goal_struggle_counter = 0

            self.get_logger().info(
                f'Goal reached. Restoring goal tolerance to {self.normal_goal_tolerance}'
            )
        
    def update_conservative_planning(self, msg):
        """
            Nav2 uses NavFn which is basically Dijkstra + A* planner.
            the parameters we adjusted are:
            inflation radius: this adds artificial block around the obstacle so that robot tries to stay away more
            
            if robot hits obstacles often we increase the radius to 60cm from 30cm
            to avoid getting too close to obstacles.
        """
    
        poor_efficiency = (
            msg.obstacle_avoidance_efficiency > 1.8
        )

        if poor_efficiency:
            self.bad_efficiency_counter += 1
        else:
            self.bad_efficiency_counter = 0

        if (
            self.bad_efficiency_counter >= self.bad_efficiency_threshold
            and not self.conservative_mode_enabled
        ):

            self.set_inflation_radius(
                self.conservative_inflation_radius
            )

            self.conservative_mode_enabled = True

            self.get_logger().warn(
                f'bad obstacle avoidance efficiency detected. Enabling conservative planning mode.'
            )

        elif (
            self.bad_efficiency_counter == 0
            and self.conservative_mode_enabled
        ):

            self.set_inflation_radius(
                self.normal_inflation_radius
            )

            self.conservative_mode_enabled = False

            self.get_logger().info(
                f'Obstacle avoidance efficiency normalized. '
                f'Restoring normal planning mode.'
            )
            
    def update_environment_complexity(self, msg):
        """
        based on our monitored metrics we 

        obstacle_density: is the fraction of laser readings that are hitting something nearby
        environment_complexity: is tends to increase when there are many nearby obstacles around the robot. 

        we adjust cost_scaling_factor of the local costmap, this is quite similar to inflation radius 
        but it affects the cost values in a more continuous way. 
        higher cost scaling factor means that the cost of cells near obstacles will increase greater,
        which also makes the planner more conservative in its path selection.
        
        """
        
        complex_environment = (
            msg.obstacle_density > 0.35 or
            msg.environment_complexity > 0.8 or
            msg.mean_obstacle_distance < 1.0
        )

        if complex_environment:
            self.complex_environment_counter += 1
        else:
            self.complex_environment_counter = 0

        if (
            self.complex_environment_counter >= self.complex_environment_threshold
            and not self.complex_environment_enabled
        ):
            self.set_cost_scaling_factor(self.complex_cost_scaling_factor)
            self.complex_environment_enabled = True

            self.get_logger().warn(
                f'Complex environment detected. Increasing local costmap cost scaling factor to {self.complex_cost_scaling_factor}'
            )

        elif (
            self.complex_environment_counter == 0
            and self.complex_environment_enabled
        ):
            self.set_cost_scaling_factor(self.normal_cost_scaling_factor)
            self.complex_environment_enabled = False

            self.get_logger().info(
                f'Environment complexity normalized to: {self.normal_cost_scaling_factor}'
            )
            
    def update_narrow_passage_mode(self, msg):
        
        if self.last_distance_to_goal is None:
            progress = 999.0
        else:
            progress = self.last_distance_to_goal - msg.navigation_accuracy

        self.last_distance_to_goal = msg.navigation_accuracy

        self.get_logger().info(
            f"narrow debug | left={msg.left_clearance:.2f}, "
            f"right={msg.right_clearance:.2f}, "
            f"front={msg.front_clearance:.2f}, "
            f"corridor={msg.corridor_score:.1f}, "
            f"progress={progress:.4f}, "
            f"stuck_counter={self.narrow_stuck_counter}"
        )
        
        narrow_stuck = (
            msg.corridor_score > 0.5 and
            progress < 0.002 and
            not msg.goal_reached
        )

        if narrow_stuck:
            self.narrow_stuck_counter += 1
        else:
            self.narrow_stuck_counter = 0
            
        # for simulation i will disable it since it can mess up goal ids
        if self.use_wall_escape:
            self.update_wall_escape_goal(msg, progress)

        if self.escape_goal_active:
            return

        if self.narrow_stuck_counter >= self.narrow_stuck_threshold:
            self.apply_narrow_recovery()
        
    def set_max_velocity(self, value):
        
        # these are responsible for planner's commanded velocities. 
        params = [
            self.make_double_param('FollowPath.max_vel_x', value),
            self.make_double_param('FollowPath.max_speed_xy', value),
        ]

        request = SetParameters.Request()
        request.parameters = params

        self.param_client.call_async(request)


    def make_double_param(self, name, value):

        param = Parameter()
        param.name = name
        param.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE,
            double_value=float(value)
        )

        return param
    
            
    def set_goal_tolerance(self, value):
    
        params = [
            self.make_double_param(
                'general_goal_checker.xy_goal_tolerance',
                value
            ),
            self.make_double_param(
                'FollowPath.xy_goal_tolerance',
                value
            )
        ]

        request = SetParameters.Request()
        request.parameters = params

        self.param_client.call_async(request)
        
    def set_inflation_radius(self, value):
    
        params = [
            self.make_double_param(
                'inflation_layer.inflation_radius',
                value
            )
        ]

        request = SetParameters.Request()
        request.parameters = params

        self.local_costmap_client.call_async(request)
        self.global_costmap_client.call_async(request)
        
    def set_cost_scaling_factor(self, value):
    
        params = [
            self.make_double_param(
                'inflation_layer.cost_scaling_factor',
                value
            )
        ]

        request = SetParameters.Request()
        request.parameters = params

        self.local_costmap_client.call_async(request)
        
    def set_controller_parameter(self, name, value):
        
        params = [
            self.make_double_param(name, value)
        ]

        request = SetParameters.Request()
        request.parameters = params

        self.param_client.call_async(request)
        
    def apply_narrow_recovery(self):
        
        self.get_logger().warn("Narrow passage stuck. Applying slow recovery mode.")

        self.set_controller_parameter('FollowPath.max_vel_theta', self.recovery_max_theta_velocity)
        self.set_controller_parameter('FollowPath.min_vel_x', self.recovery_min_velocity)
        self.set_controller_parameter('FollowPath.max_vel_x', self.recovery_max_velocity)
        self.set_controller_parameter('FollowPath.max_speed_xy', self.recovery_max_velocity)

        self.set_inflation_radius(self.recovery_inflation_radius)
        self.set_cost_scaling_factor(self.recovery_cost_scaling_factor)

        if self.local_clear_client.wait_for_service(timeout_sec=0.1):
            self.local_clear_client.call_async(ClearEntireCostmap.Request())

        if self.global_clear_client.wait_for_service(timeout_sec=0.1):
            self.global_clear_client.call_async(ClearEntireCostmap.Request())

        self.escape_mode_enabled = False
        self.narrow_stuck_counter = 0
        self.narrow_passage_counter = 0
        
    def update_wall_escape_goal(self, msg, progress):
    
        if msg.goal_id < 0:
            return

        # handling escape goal first
        if self.escape_goal_active:
            both_sides_clear = (
                msg.left_clearance > self.wall_escape_clear_threshold and
                msg.right_clearance > self.wall_escape_clear_threshold
            )

            escape_goal_finished = msg.goal_reached

            if both_sides_clear or escape_goal_finished:
                self.get_logger().info(
                    'Wall escape finished. Restoring original goal.'
                )
                self.send_original_goal()

            return

        # normal goal checks only after escape handling
        if msg.goal_reached:
            return

        if msg.navigation_accuracy < 0.20:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9

        if current_time - self.last_wall_escape_time < self.wall_escape_min_interval:
            return

        no_progress = progress < self.wall_escape_progress_threshold

        right_close = msg.right_clearance < self.wall_escape_close_threshold
        left_close = msg.left_clearance < self.wall_escape_close_threshold

        if not no_progress:
            return

        if right_close:
            self.original_goal = (msg.goal_x, msg.goal_y)
            self.send_wall_escape_goal('right')
            self.last_wall_escape_time = current_time

        elif left_close:
            self.original_goal = (msg.goal_x, msg.goal_y)
            self.send_wall_escape_goal('left')
            self.last_wall_escape_time = current_time


    def send_wall_escape_goal(self, side):
        
        """
        Wall escape logic, we inspect global pose of the robot and find a 
        smaller goal point pointing opposite to the close wall
        
        it seemed to work well in narrow passages as well
        """

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

        except TransformException:
            self.get_logger().warn('Could not get robot transform for wall escape.')
            return

        x = transform.transform.translation.x
        y = transform.transform.translation.y

        q = transform.transform.rotation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        forward_x = math.cos(yaw)
        forward_y = math.sin(yaw)

        left_x = -math.sin(yaw)
        left_y = math.cos(yaw)

        if side == 'right':
            lateral = self.wall_escape_lateral_offset
        else:
            lateral = -self.wall_escape_lateral_offset

        target_x = (
            x +
            self.wall_escape_forward_distance * forward_x +
            lateral * left_x
        )

        target_y = (
            y +
            self.wall_escape_forward_distance * forward_y +
            lateral * left_y
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = target_x
        goal_msg.pose.pose.position.y = target_y
        goal_msg.pose.pose.orientation.w = 1.0

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('NavigateToPose action server not available.')
            return

        self.nav_to_pose_client.send_goal_async(goal_msg)

        self.escape_goal_active = True

        self.get_logger().warn(
            f'Wall escape goal sent. side={side}, x={target_x:.2f}, y={target_y:.2f}'
        )


    def send_original_goal(self):

        if self.original_goal is None:
            self.escape_goal_active = False
            return

        goal_x, goal_y = self.original_goal

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.orientation.w = 1.0

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('NavigateToPose action server not available.')
            return

        self.nav_to_pose_client.send_goal_async(goal_msg)

        self.escape_goal_active = False
        self.original_goal = None
        self.narrow_stuck_counter = 0
        self.last_distance_to_goal = None

        self.get_logger().info('Original goal restored after wall escape.')

def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()