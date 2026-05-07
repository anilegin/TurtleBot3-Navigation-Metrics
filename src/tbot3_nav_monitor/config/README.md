## Adaptive Controller Configuration

The adaptive controller uses `config/adaptive_controller.yaml` to tune how the robot reacts to difficult navigation conditions such as getting stuck, poor path efficiency, narrow passages, and wall escape.

### General

| Parameter | Description |
|---|---|
| `controller_node` | Name of the Nav2 controller server node whose parameters are updated dynamically. Default is `/controller_server`. |

### Velocity Adaptation

| Parameter | Description |
|---|---|
| `normal_max_velocity` | Default maximum linear velocity used during normal navigation. |
| `reduced_max_velocity` | Lower maximum velocity used when the robot repeatedly appears stuck. |
| `stuck_threshold` | Number of detected stuck events required before reducing velocity. |

### Goal Tolerance Adaptation

| Parameter | Description |
|---|---|
| `normal_goal_tolerance` | Default distance tolerance for considering a goal reached. |
| `relaxed_goal_tolerance` | Larger tolerance used when the robot struggles near the goal. |
| `goal_struggle_threshold` | Number of consecutive struggling detections near the goal before relaxing tolerance. |

### Path Planning Adaptation

| Parameter | Description |
|---|---|
| `normal_inflation_radius` | Default costmap inflation radius around obstacles. |
| `conservative_inflation_radius` | Increased inflation radius used when path efficiency is poor. |
| `bad_efficiency_threshold` | Number of consecutive poor-efficiency detections before enabling conservative planning. |

### Environment Complexity Adaptation

| Parameter | Description |
|---|---|
| `normal_cost_scaling_factor` | Default cost scaling factor for the local costmap inflation layer. |
| `complex_cost_scaling_factor` | Higher cost scaling factor used in complex environments. |
| `complex_environment_threshold` | Number of consecutive complex-environment detections before updating costmap behavior. |

### Narrow Passage Detection

| Parameter | Description |
|---|---|
| `narrow_passage_threshold` | Number of narrow-passage detections required before considering the robot to be in a narrow passage. |
| `narrow_stuck_threshold` | Number of consecutive low-progress detections inside a narrow passage before recovery behavior is applied. |

### Narrow Recovery Behavior

| Parameter | Description |
|---|---|
| `recovery_max_velocity` | Maximum linear velocity used during recovery mode. |
| `recovery_min_velocity` | Minimum linear velocity allowed during recovery mode. |
| `recovery_max_theta_velocity` | Maximum angular velocity used during recovery mode. |
| `recovery_inflation_radius` | Inflation radius used during recovery mode. |
| `recovery_cost_scaling_factor` | Cost scaling factor used during recovery mode. |

### Wall Escape Behavior

| Parameter | Description |
|---|---|
| `wall_escape_forward_distance` | Forward distance of the temporary escape goal from the robot position. |
| `wall_escape_lateral_offset` | Sideways offset added to the temporary escape goal to move away from the nearby wall. |
| `wall_escape_close_threshold` | Distance threshold for deciding that a wall is too close on the left or right side. |
| `wall_escape_clear_threshold` | Clearance threshold used to decide that the robot has enough space to return to the original goal. |
| `wall_escape_progress_threshold` | Minimum progress required to avoid triggering wall escape. If progress is lower than this, the robot may be considered stuck near a wall. |
| `wall_escape_min_interval` | Minimum time in seconds between two wall escape attempts. Prevents repeated escape goals from being sent too frequently. |
| `use_wall_escape` | Enables or disables the wall escape behavior. Set to `False` during clean ML data collection if temporary goals should be avoided. |