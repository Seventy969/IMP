#!/bin/bash
# ============================================================
# robot_hotfix.sh
# Fixes ALL bugs found in robot_setup_full.sh + robot_fix.sh:
#
#  Bug 1: nav2_params.yaml — Windows-1252 em-dash (0x97) at byte 82
#  Bug 2: slam_params.yaml — UTF-8 BOM causing "line 0" YAML parse error
#  Bug 3: package.xml      — <n> instead of <name>
#  Bug 4: launch files     — package='robot_controllers' (re-applies robot_fix patches)
#  Bug 5: xterm title      — em-dash in prefix string (cosmetic)
#  Bug 6: cutter_on.sh     — add timeout so it doesn't hang forever
#
# Run with: bash robot_hotfix.sh
# ============================================================

set -e
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo ""
echo "============================================================"
echo "   Robot Hotfix — fixing all known bugs"
echo "============================================================"
echo ""

HOME_DIR="$HOME"
ROBOT="$HOME_DIR/robot"
SCRIPTS="$ROBOT/scripts"
CONFIG="$ROBOT/config"
LAUNCH="$ROBOT/launch"
PKG="$HOME_DIR/ws_lidar/src/robot_controllers"


# ════════════════════════════════════════════════════════════
# BUG 1 + 2 FIX: Rewrite both YAML config files cleanly
#   (no heredoc from Windows-encoded file — write byte-safe)
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[1/6] Rewriting slam_params.yaml (fix BOM / line-0 error)...${NC}"

# Use python3 to write the file — avoids any shell encoding issues entirely
python3 - << 'PYEOF'
import os
home = os.path.expanduser('~')
path = os.path.join(home, 'robot', 'config', 'slam_params.yaml')
os.makedirs(os.path.dirname(path), exist_ok=True)

content = """\
# slam_params.yaml - SLAM Toolbox (online async)
# Tuned for no-encoder robot: minimum_travel_* = 0
slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    use_map_saver: true
    mode: mapping

    # No encoder - allow map updates without odometry movement
    minimum_travel_distance: 0.0
    minimum_travel_heading: 0.0
    map_update_interval: 3.0

    resolution: 0.05
    max_laser_range: 12.0
    minimum_time_interval: 0.3
    transform_timeout: 0.5
    tf_buffer_duration: 30.0

    stack_size_to_use: 40000000

    # Loop closure
    do_loop_closing: true
    loop_search_maximum_distance: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45
    loop_match_maximum_variance_coarse: 3.0
    loop_match_force_accept_ratio: 0.9

    # Scan matching
    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0
    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
"""

with open(path, 'w', encoding='utf-8', newline='\n') as f:
    f.write(content)

# Verify: read back and check no BOM and no 0x97
with open(path, 'rb') as f:
    raw = f.read()
assert raw[:3] != b'\xef\xbb\xbf', "BOM found!"
assert b'\x97' not in raw, "0x97 found!"
print(f"  OK: {path}")
print(f"  First 4 bytes: {raw[:4].hex()}")
PYEOF
echo -e "${GREEN}  slam_params.yaml rewritten cleanly${NC}"


echo -e "${YELLOW}[2/6] Rewriting nav2_params.yaml (fix 0x97 em-dash at byte 82)...${NC}"

python3 - << 'PYEOF'
import os
home = os.path.expanduser('~')
path = os.path.join(home, 'robot', 'config', 'nav2_params.yaml')

content = """\
# nav2_params.yaml - Lawn Mower Robot
# ROS2 Kilted / No wheel encoder / No IMU

amcl:
  ros__parameters:
    use_sim_time: false
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    odom_frame_id: "odom"
    global_frame_id: "map"
    laser_max_range: 12.0
    laser_min_range: 0.12
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    laser_model_type: "likelihood_field"
    max_particles: 2000
    min_particles: 500
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    scan_topic: scan
    set_initial_pose: true
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    wait_for_service_timeout: 1000
    action_server_result_timeout: 900.0
    navigators: ["navigate_to_pose", "navigate_through_poses"]
    navigate_to_pose:
      plugin: "nav2_bt_navigator::NavigateToPoseNavigator"
    navigate_through_poses:
      plugin: "nav2_bt_navigator::NavigateThroughPosesNavigator"
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_smooth_path_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_assisted_teleop_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_drive_on_heading_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_globally_updated_goal_condition_bt_node
      - nav2_is_path_valid_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_truncate_path_local_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_path_expiring_timer_condition
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
      - nav2_goal_updated_controller_bt_node
      - nav2_is_battery_low_condition_bt_node
      - nav2_navigate_through_poses_action_bt_node
      - nav2_navigate_to_pose_action_bt_node
      - nav2_remove_passed_goals_action_bt_node
      - nav2_planner_selector_bt_node
      - nav2_controller_selector_bt_node
      - nav2_goal_checker_selector_bt_node
      - nav2_smoother_selector_bt_node
      - nav2_cancel_control_bt_node
      - nav2_cancel_recovery_bt_node

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    general_goal_checker:
      stateful: true
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.20
      yaw_goal_tolerance: 0.35
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: false
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.35
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.35
      min_speed_theta: 0.0
      acc_lim_x: 0.3
      acc_lim_y: 0.0
      acc_lim_theta: 1.5
      decel_lim_x: -0.3
      decel_lim_y: 0.0
      decel_lim_theta: -1.5
      vx_samples: 20
      vy_samples: 5
      vtheta_samples: 20
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.5
      xy_goal_tolerance: 0.20
      trans_stopped_velocity: 0.25
      short_circuit_trajectory_evaluation: true
      stateful: true
      critics:
        - "RotateToGoal"
        - "Oscillation"
        - "BaseObstacle"
        - "GoalAlign"
        - "PathAlign"
        - "PathDist"
        - "GoalDist"
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: false
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.25
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        publish_voxel_map: true
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: true

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: false
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.25
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
        enabled: true
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: true

map_server:
  ros__parameters:
    use_sim_time: false
    yaml_filename: ""

map_saver:
  ros__parameters:
    use_sim_time: false
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65
    map_subscribe_transient_local: true

planner_server:
  ros__parameters:
    use_sim_time: false
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

smoother_server:
  ros__parameters:
    use_sim_time: false
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: true

behavior_server:
  ros__parameters:
    use_sim_time: false
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait", "assisted_teleop"]
    spin:
      plugin: "nav2_behaviors::Spin"
    backup:
      plugin: "nav2_behaviors::BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors::DriveOnHeading"
    wait:
      plugin: "nav2_behaviors::Wait"
    assisted_teleop:
      plugin: "nav2_behaviors::AssistedTeleop"
    global_frame: odom
    robot_base_frame: base_link
    transform_tolerance: 0.1
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2

waypoint_follower:
  ros__parameters:
    use_sim_time: false
    loop_rate: 20
    stop_on_failure: false
    action_server_result_timeout: 900.0
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 0

velocity_smoother:
  ros__parameters:
    use_sim_time: false
    smoothing_frequency: 20.0
    scale_velocities: false
    feedback: "OPEN_LOOP"
    max_velocity: [0.35, 0.0, 1.0]
    min_velocity: [-0.35, 0.0, -1.0]
    max_accel: [2.5, 0.0, 3.2]
    max_decel: [-2.5, 0.0, -3.2]
    odom_topic: "odom"
    odom_duration: 0.1
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 1.0

collision_monitor:
  ros__parameters:
    use_sim_time: false
    base_frame_id: "base_link"
    odom_frame_id: "odom"
    cmd_vel_in_topic: "cmd_vel_smoothed"
    cmd_vel_out_topic: "cmd_vel"
    state_topic: "collision_monitor_state"
    transform_tolerance: 0.5
    source_timeout: 5.0
    base_shift_correction: true
    stop_pub_timeout: 2.0
    polygons: ["FootprintApproach"]
    FootprintApproach:
      type: "polygon"
      action_type: "approach"
      footprint_topic: "/local_costmap/published_footprint"
      time_before_collision: 2.0
      simulation_time_step: 0.02
      min_points: 6
      visualize: false
      enabled: true
    observation_sources: ["scan"]
    scan:
      type: "scan"
      topic: "/scan"
"""

with open(path, 'w', encoding='utf-8', newline='\n') as f:
    f.write(content)

with open(path, 'rb') as f:
    raw = f.read()
assert raw[:3] != b'\xef\xbb\xbf', "BOM found!"
assert b'\x97' not in raw, "0x97 still present!"
print(f"  OK: {path}")
print(f"  Byte 82: 0x{raw[82]:02X}  (should be 0x2D '-' or 0x4C 'L')")
PYEOF
echo -e "${GREEN}  nav2_params.yaml rewritten cleanly (no 0x97)${NC}"


# ════════════════════════════════════════════════════════════
# BUG 3 FIX: package.xml <n> -> <name>
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[3/6] Fixing package.xml (<n> -> <name>)...${NC}"
PKG_XML="$PKG/package.xml"
if [ -f "$PKG_XML" ]; then
    sed -i 's|<n>robot_controllers</n>|<name>robot_controllers</name>|g' "$PKG_XML"
    # Verify
    if grep -q "<name>robot_controllers</name>" "$PKG_XML"; then
        echo -e "${GREEN}  package.xml fixed${NC}"
    else
        echo -e "${RED}  WARNING: package.xml fix may have failed — check manually${NC}"
    fi
else
    echo "  package.xml not found at $PKG_XML — skipping (robot_controllers package not present)"
fi


# ════════════════════════════════════════════════════════════
# BUG 4 + 5 FIX: Rewrite all 3 launch files
#   - ExecuteProcess instead of Node(package='robot_controllers')
#   - Plain hyphens instead of em-dashes
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[4/6] Rewriting mapping_launch.py...${NC}"

python3 - << 'PYEOF'
import os
home = os.path.expanduser('~')
path = os.path.join(home, 'robot', 'launch', 'mapping_launch.py')

content = '''\
#!/usr/bin/env python3
"""
mapping_launch.py
Launches: RPLidar, SLAM Toolbox, Motor Controller, Teleop, RViz
Drive around to build a map, then: robot_save_map my_lawn
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    home        = os.path.expanduser('~')
    slam_dir    = get_package_share_directory('slam_toolbox')
    slam_params = os.path.join(home, 'robot', 'config', 'slam_params.yaml')
    rviz_config = os.path.join(home, 'robot', 'rviz', 'mapping_config.rviz')
    scripts_dir = os.path.join(home, 'robot', 'scripts')

    return LaunchDescription([

        # static TF: base_link -> laser (LiDAR 10 cm above base)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
            output='screen',
        ),
        # static TF: odom -> base_link (no encoder fallback)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen',
        ),

        # RPLidar A1M8
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port':      '/dev/ttyUSB0',
                'serial_baudrate':   115200,
                'frame_id':          'laser',
                'angle_compensate':  True,
                'scan_mode':         'Sensitivity',
            }],
            output='screen',
        ),

        # Motor controller - direct Python call (no package discovery needed)
        ExecuteProcess(
            cmd=['python3', os.path.join(scripts_dir, 'motor_controller.py')],
            name='motor_controller',
            output='screen',
        ),

        # SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'slam_params_file': slam_params,
                'use_sim_time':     'false',
            }.items(),
        ),

        # Teleop keyboard in dedicated xterm window
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            output='screen',
            prefix='xterm -title "TELEOP - click here first!" -fa Monospace -fs 12 -e',
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
'''

with open(path, 'w', encoding='utf-8', newline='\n') as f:
    f.write(content)
os.chmod(path, 0o755)
print(f"  Written: {path}")
PYEOF
echo -e "${GREEN}  mapping_launch.py rewritten${NC}"


echo -e "${YELLOW}[4b/6] Rewriting navigation_launch.py...${NC}"

python3 - << 'PYEOF'
import os
home = os.path.expanduser('~')
path = os.path.join(home, 'robot', 'launch', 'navigation_launch.py')

content = '''\
#!/usr/bin/env python3
"""
navigation_launch.py - WITH saved map
Launches: RPLidar, map_server, AMCL, Nav2, Motor, Cutter, Coverage, RViz
Usage: robot_navigate my_lawn
"""

import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             ExecuteProcess, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    home        = os.path.expanduser('~')
    nav2_dir    = get_package_share_directory('nav2_bringup')
    params_file = os.path.join(home, 'robot', 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(home, 'robot', 'rviz', 'navigation_config.rviz')
    scripts_dir = os.path.join(home, 'robot', 'scripts')

    map_name_arg = DeclareLaunchArgument(
        'map_name', default_value='my_lawn',
        description='Map stem name in ~/robot/maps/'
    )
    map_name = LaunchConfiguration('map_name')

    return LaunchDescription([
        map_name_arg,

        # static TFs
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen',
        ),

        # RPLidar A1M8
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port':      '/dev/ttyUSB0',
                'serial_baudrate':   115200,
                'frame_id':          'laser',
                'angle_compensate':  True,
                'scan_mode':         'Sensitivity',
            }],
            output='screen',
        ),

        # Motor controller
        ExecuteProcess(
            cmd=['python3', os.path.join(scripts_dir, 'motor_controller.py')],
            name='motor_controller',
            output='screen',
        ),

        # Cutter controller
        ExecuteProcess(
            cmd=['python3', os.path.join(scripts_dir, 'cutter_controller.py')],
            name='cutter_controller',
            output='screen',
        ),

        # Map server - map_name is a LaunchConfiguration substitution
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': [os.path.join(home, 'robot', 'maps/'),
                                  map_name, '.yaml'],
                'use_sim_time': False,
            }],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart':    True,
                'node_names':   ['map_server'],
                'bond_timeout': 0.0,
            }],
        ),

        # AMCL localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_amcl',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart':    True,
                'node_names':   ['amcl'],
                'bond_timeout': 0.0,
            }],
        ),

        # Nav2 core stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file':  params_file,
            }.items(),
        ),

        # Coverage path planner - delayed 20s for Nav2 action server
        TimerAction(
            period=20.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3',
                         os.path.join(scripts_dir, 'coverage_path_planner.py')],
                    name='coverage_path_planner',
                    output='screen',
                )
            ],
        ),

        # RViz - delayed 18s so map is latched before subscribing
        TimerAction(
            period=18.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config],
                    output='screen',
                )
            ],
        ),
    ])
'''

with open(path, 'w', encoding='utf-8', newline='\n') as f:
    f.write(content)
os.chmod(path, 0o755)
print(f"  Written: {path}")
PYEOF
echo -e "${GREEN}  navigation_launch.py rewritten${NC}"


echo -e "${YELLOW}[4c/6] Rewriting coverage_nomap_launch.py...${NC}"

python3 - << 'PYEOF'
import os
home = os.path.expanduser('~')
path = os.path.join(home, 'robot', 'launch', 'coverage_nomap_launch.py')

content = '''\
#!/usr/bin/env python3
"""
coverage_nomap_launch.py - autonomous WITHOUT a saved map
SLAM builds map live while Nav2 drives the coverage pattern.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    home        = os.path.expanduser('~')
    nav2_dir    = get_package_share_directory('nav2_bringup')
    slam_dir    = get_package_share_directory('slam_toolbox')
    params_file = os.path.join(home, 'robot', 'config', 'nav2_params.yaml')
    slam_params = os.path.join(home, 'robot', 'config', 'slam_params.yaml')
    rviz_config = os.path.join(home, 'robot', 'rviz', 'navigation_config.rviz')
    scripts_dir = os.path.join(home, 'robot', 'scripts')

    return LaunchDescription([

        # static TFs
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen',
        ),

        # RPLidar A1M8
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port':      '/dev/ttyUSB0',
                'serial_baudrate':   115200,
                'frame_id':          'laser',
                'angle_compensate':  True,
                'scan_mode':         'Sensitivity',
            }],
            output='screen',
        ),

        # Motor controller
        ExecuteProcess(
            cmd=['python3', os.path.join(scripts_dir, 'motor_controller.py')],
            name='motor_controller',
            output='screen',
        ),

        # Cutter controller
        ExecuteProcess(
            cmd=['python3', os.path.join(scripts_dir, 'cutter_controller.py')],
            name='cutter_controller',
            output='screen',
        ),

        # SLAM Toolbox - starts immediately, provides map->odom TF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'slam_params_file': slam_params,
                'use_sim_time':     'false',
            }.items(),
        ),

        # Nav2 - delayed 8s so SLAM publishes /map first
        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': 'false',
                        'params_file':  params_file,
                    }.items(),
                )
            ],
        ),

        # Coverage planner - delayed 25s for full startup
        TimerAction(
            period=25.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3',
                         os.path.join(scripts_dir, 'coverage_path_planner.py')],
                    name='coverage_path_planner',
                    output='screen',
                )
            ],
        ),

        # RViz - delayed 10s
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config],
                    output='screen',
                )
            ],
        ),
    ])
'''

with open(path, 'w', encoding='utf-8', newline='\n') as f:
    f.write(content)
os.chmod(path, 0o755)
print(f"  Written: {path}")
PYEOF
echo -e "${GREEN}  coverage_nomap_launch.py rewritten${NC}"


# ════════════════════════════════════════════════════════════
# BUG 6 FIX: cutter_on.sh / cutter_off.sh - add timeout
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[5/6] Fixing cutter_on.sh and cutter_off.sh (add timeout)...${NC}"

cat > "$SCRIPTS/cutter_on.sh" << 'SHEOF'
#!/bin/bash
source /opt/ros/kilted/setup.bash
echo "Turning cutter ON..."
echo "(Waiting up to 10 s for cutter service — only available during robot_navigate or robot_coverage)"

if ros2 service call /cutter/set std_srvs/srv/SetBool '{data: true}' \
        --timeout 10 2>/dev/null; then
    echo "Cutter is ON"
else
    echo "ERROR: /cutter/set service not found."
    echo "Make sure robot_navigate or robot_coverage is running first."
    exit 1
fi
SHEOF

cat > "$SCRIPTS/cutter_off.sh" << 'SHEOF'
#!/bin/bash
source /opt/ros/kilted/setup.bash
echo "Turning cutter OFF..."

if ros2 service call /cutter/set std_srvs/srv/SetBool '{data: false}' \
        --timeout 10 2>/dev/null; then
    echo "Cutter is OFF"
else
    echo "ERROR: /cutter/set service not found."
    echo "Make sure robot_navigate or robot_coverage is running first."
    exit 1
fi
SHEOF

chmod +x "$SCRIPTS/cutter_on.sh" "$SCRIPTS/cutter_off.sh"
echo -e "${GREEN}  cutter_on.sh and cutter_off.sh fixed (10s timeout, clear error msg)${NC}"


# ════════════════════════════════════════════════════════════
# VERIFY: syntax check all Python files
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[6/6] Verifying syntax and encoding of all files...${NC}"
echo ""

PASS=0; FAIL=0

check_py() {
    local f="$1"
    if python3 -c "import ast; ast.parse(open('$f').read())" 2>/dev/null; then
        echo -e "  ${GREEN}OK Python ${NC} $(basename $f)"
        PASS=$((PASS+1))
    else
        echo -e "  ${RED}FAIL     ${NC} $(basename $f)"
        FAIL=$((FAIL+1))
    fi
}

check_yaml() {
    local f="$1"
    # Check no BOM, no 0x97
    if python3 - << PYCHECK
import sys
data = open('$f', 'rb').read()
if data[:3] == b'\\xef\\xbb\\xbf':
    print("  BOM found!"); sys.exit(1)
if b'\\x97' in data:
    pos = data.index(b'\\x97')
    print(f"  0x97 at byte {pos}!"); sys.exit(1)
try:
    import yaml
    yaml.safe_load(data.decode('utf-8'))
    print("  YAML valid")
except Exception as e:
    print(f"  YAML error: {e}"); sys.exit(1)
PYCHECK
    then
        echo -e "  ${GREEN}OK YAML   ${NC} $(basename $f)"
        PASS=$((PASS+1))
    else
        echo -e "  ${RED}FAIL      ${NC} $(basename $f)"
        FAIL=$((FAIL+1))
    fi
}

check_py  "$LAUNCH/mapping_launch.py"
check_py  "$LAUNCH/navigation_launch.py"
check_py  "$LAUNCH/coverage_nomap_launch.py"
check_py  "$SCRIPTS/motor_controller.py"
check_py  "$SCRIPTS/cutter_controller.py"
check_py  "$SCRIPTS/coverage_path_planner.py"
check_yaml "$CONFIG/slam_params.yaml"
check_yaml "$CONFIG/nav2_params.yaml"

# Check package.xml
if [ -f "$PKG/package.xml" ]; then
    if grep -q "<name>robot_controllers</name>" "$PKG/package.xml"; then
        echo -e "  ${GREEN}OK XML    ${NC} package.xml (<name> tag correct)"
        PASS=$((PASS+1))
    else
        echo -e "  ${RED}FAIL      ${NC} package.xml (still has <n> or wrong tag)"
        FAIL=$((FAIL+1))
    fi
fi

echo ""
echo "────────────────────────────────────────────────────────────"
echo -e "  Passed: ${GREEN}$PASS${NC}   Failed: ${RED}$FAIL${NC}"
echo "────────────────────────────────────────────────────────────"
echo ""

if [ "$FAIL" -eq 0 ]; then
    echo -e "${GREEN}ALL CHECKS PASSED — robot is ready!${NC}"
    echo ""
    echo "WORKFLOW:"
    echo "  1.  robot_map                 <- build map (drive with teleop)"
    echo "  2.  robot_save_map my_lawn    <- save when done"
    echo "  3.  robot_navigate my_lawn    <- autonomous mowing"
    echo "  4.  robot_cutter_on           <- start blade"
    echo ""
    echo "OR skip mapping entirely:"
    echo "  1.  robot_coverage            <- mows + maps at the same time"
    echo "  2.  robot_cutter_on"
    echo ""
    echo "FIRST POSE (run once after robot_navigate starts, in a 2nd terminal):"
    echo "  ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \\"
    echo "    '{header:{frame_id:\"map\"},pose:{pose:{position:{x:0.0,y:0.0},orientation:{w:1.0}},covariance:[0.25,0,0,0,0,0,0,0.25,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.07]}}'"
    echo ""
    echo "Then click '2D Pose Estimate' in RViz to refine."
else
    echo -e "${RED}Some checks failed. Review the output above.${NC}"
fi
