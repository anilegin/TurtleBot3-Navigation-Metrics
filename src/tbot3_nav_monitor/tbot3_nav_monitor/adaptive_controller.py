import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters

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

        if self.narrow_passage_enabled or self.escape_mode_enabled:
            return

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
        # this helps us to avoid getting stucked in narrow passages 
        if self.narrow_passage_enabled:
            return
    
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
        if self.narrow_passage_enabled:
            return

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
            f"counter={self.narrow_passage_counter}, "
            f"stuck_counter={self.narrow_stuck_counter}"
        )
        
        narrow_passage = (
            msg.corridor_score > 0.5 and
            progress < 0.002 and
            not msg.goal_reached
        )

        narrow_stuck = (
            narrow_passage and
            progress < 0.002
        )

        if narrow_stuck:
            self.narrow_stuck_counter += 1
        else:
            self.narrow_stuck_counter = 0
            
        if self.narrow_stuck_counter >= self.narrow_stuck_threshold:
            self.apply_narrow_recovery()
            return

        if narrow_passage:
            self.narrow_passage_counter += 1
        else:
            self.narrow_passage_counter = 0

        
        if self.narrow_passage_counter >= self.narrow_passage_threshold and not self.narrow_passage_enabled:
            self.set_max_velocity(self.narrow_passage_velocity)
            self.set_inflation_radius(self.narrow_passage_inflation_radius)
            self.set_cost_scaling_factor(self.narrow_passage_cost_scaling_factor)
            
            self.set_controller_parameter('FollowPath.max_vel_theta', self.narrow_max_theta_velocity)

            self.narrow_passage_enabled = True
            self.escape_mode_enabled = False
            self.velocity_reduced = False
            self.conservative_mode_enabled = False
            self.complex_environment_enabled = False

            self.get_logger().warn("Narrow Passage Mode Enbaled")

        elif self.narrow_passage_counter == 0 and (self.narrow_passage_enabled or self.escape_mode_enabled):
            self.set_max_velocity(self.normal_max_velocity)
            self.set_inflation_radius(self.normal_inflation_radius)
            self.set_cost_scaling_factor(self.normal_cost_scaling_factor)
            
            self.set_controller_parameter('FollowPath.max_vel_x', self.normal_max_velocity)
            self.set_controller_parameter('FollowPath.min_vel_x', 0.0)
            self.set_controller_parameter('FollowPath.max_speed_xy', self.normal_max_velocity)
            self.set_controller_parameter('FollowPath.max_vel_theta', 1.0)

            self.narrow_passage_enabled = False
            self.escape_mode_enabled = False
            self.narrow_stuck_counter = 0
            self.get_logger().info("Narrow passage cleared")

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

        self.escape_mode_enabled = True
        self.narrow_stuck_counter = 0
        self.narrow_passage_counter = 0

def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()