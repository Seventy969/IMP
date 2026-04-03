#!/bin/bash
# fix_robot_navigation.sh
# Fixes all navigation files for RPLidar A1M8 + ROS2 Kilted on Raspberry Pi 4B
# Run with: bash fix_robot_navigation.sh

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo ""
echo "================================================="
echo "   Robot Navigation - Complete Fix"
echo "================================================="
echo ""

mkdir -p ~/robot/{maps,launch,scripts,rviz,config}

# ══════════════════════════════════════════════════════
# FILE 1: nav2_params.yaml
# ══════════════════════════════════════════════════════
echo -e "${YELLOW}[1/4] Writing nav2_params.yaml...${NC}"

cat > ~/robot/config/nav2_params.yaml << 'EOF'
# Nav2 Parameters - Lawn Mower Robot (ROS2 Kilted)

amcl:
  ros__parameters:
    use_sim_time: False
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    global_frame_id: "map"
    laser_max_range: 12.0
    laser_min_range: 0.12
    laser_model_type: "likelihood_field"
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    scan_topic: scan

controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.15
      yaw_goal_tolerance: 0.20

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      max_vel_x: 0.4
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.4
      acc_lim_x: 0.3
      acc_lim_theta: 1.5
      decel_lim_x: -0.3
      decel_lim_theta: -1.5
      vx_samples: 20
      vtheta_samples: 20
      sim_time: 1.7
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
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
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: False
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.25
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: False
      robot_radius: 0.25
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: False
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
    backup:
      plugin: "nav2_behaviors::BackUp"
    wait:
      plugin: "nav2_behaviors::Wait"
    global_frame: odom
    robot_base_frame: base_link
    transform_tolerance: 0.1
    use_sim_time: False

bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
EOF

echo -e "${GREEN}  ✓ nav2_params.yaml written${NC}"

# ══════════════════════════════════════════════════════
# FILE 2: navigation_launch.py
# ══════════════════════════════════════════════════════
echo -e "${YELLOW}[2/4] Writing navigation_launch.py...${NC}"

cat > ~/robot/launch/navigation_launch.py << 'EOF'
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    home = os.path.expanduser('~')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file  = LaunchConfiguration('params_file',
                       default=os.path.join(home, 'robot', 'config', 'nav2_params.yaml'))
    rviz_config  = os.path.join(home, 'robot', 'rviz', 'navigation_config.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map',
            default_value=os.path.join(home, 'robot', 'maps', 'my_lawn.yaml')),
        DeclareLaunchArgument('params_file',
            default_value=os.path.join(home, 'robot', 'config', 'nav2_params.yaml')),

        # TF: base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_broadcaster',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
        ),

        # TF: odom -> base_link (static, no wheel encoders)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),

        # RPLidar A1M8
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'angle_compensate': True,
                'scan_mode': 'Sensitivity'
            }],
            output='screen'
        ),

        # Motor Controller
        ExecuteProcess(
            cmd=['python3', os.path.join(home, 'robot', 'scripts', 'motor_controller.py')],
            output='screen'
        ),

        # Map Server - loads the saved map
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': os.path.join(home, 'robot', 'maps', 'my_lawn.yaml'),
                'use_sim_time': False
            }]
        ),

        # Lifecycle Manager for map_server
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server'],
                'bond_timeout': 0.0
            }]
        ),

        # AMCL - localizes robot on the map
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[os.path.join(home, 'robot', 'config', 'nav2_params.yaml')]
        ),

        # Lifecycle Manager for amcl
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_amcl',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['amcl'],
                'bond_timeout': 0.0
            }]
        ),

        # Nav2 stack: controller, planner, bt_navigator, etc.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file':  params_file
            }.items()
        ),

        # RViz - delayed 15s so map is latched before RViz subscribes
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config],
                    output='screen'
                )
            ]
        ),
    ])
EOF

chmod +x ~/robot/launch/navigation_launch.py
echo -e "${GREEN}  ✓ navigation_launch.py written${NC}"

# ══════════════════════════════════════════════════════
# FILE 3: navigation_config.rviz
# ══════════════════════════════════════════════════════
echo -e "${YELLOW}[3/4] Writing navigation_config.rviz...${NC}"

cat > ~/robot/rviz/navigation_config.rviz << 'EOF'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Selection
    Name: Selection
  - Class: nav2_rviz_plugins/Navigation2
    Name: Navigation 2
Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/Map
      Name: Map
      Topic:
        Value: /map
        Depth: 5
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
      Alpha: 0.7
      Enabled: true
    - Class: rviz_default_plugins/Map
      Name: Costmap
      Topic:
        Value: /global_costmap/costmap
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
      Alpha: 0.45
      Enabled: true
    - Class: rviz_default_plugins/LaserScan
      Name: LaserScan
      Topic:
        Value: /scan
        Depth: 5
        Durability Policy: Volatile
        Reliability Policy: Best Effort
      Size (m): 0.04
      Color: 255; 0; 0
      Enabled: true
    - Class: rviz_default_plugins/PoseArray
      Name: Particles
      Topic:
        Value: /particle_cloud
        Depth: 5
        Durability Policy: Volatile
        Reliability Policy: Best Effort
      Color: 0; 255; 0
      Enabled: true
    - Class: rviz_default_plugins/Path
      Name: NavPath
      Topic:
        Value: /plan
        Depth: 5
        Durability Policy: Volatile
        Reliability Policy: Reliable
      Color: 0; 0; 255
      Enabled: true
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: false
  Global Options:
    Fixed Frame: map
    Background Color: 48; 48; 48
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
    - Class: rviz_default_plugins/SetInitialPose
      Topic:
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Value: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
EOF

echo -e "${GREEN}  ✓ navigation_config.rviz written${NC}"

# ══════════════════════════════════════════════════════
# FILE 4: start_navigation.sh (fix ~ expansion bug)
# ══════════════════════════════════════════════════════
echo -e "${YELLOW}[4/4] Writing start_navigation.sh...${NC}"

cat > ~/robot/scripts/start_navigation.sh << 'EOF'
#!/bin/bash
source /opt/ros/kilted/setup.bash
source $HOME/ws_lidar/install/setup.bash

if [ -z "$1" ]; then
    echo "Usage: robot_navigate <map_name>"
    echo "Available maps:"
    ls $HOME/robot/maps/*.yaml 2>/dev/null | xargs -I{} basename {} .yaml | sed 's/^/  - /'
    exit 1
fi

if [ ! -f "$HOME/robot/maps/$1.yaml" ]; then
    echo "Map not found: $1"
    echo "Available maps:"
    ls $HOME/robot/maps/*.yaml 2>/dev/null | xargs -I{} basename {} .yaml | sed 's/^/  - /'
    exit 1
fi

# Fix map yaml to use absolute path (in case it has relative path)
sed -i "s|image: $1.pgm|image: $HOME/robot/maps/$1.pgm|g" $HOME/robot/maps/$1.yaml 2>/dev/null
sed -i "s|image: ~/robot/maps/|image: $HOME/robot/maps/|g" $HOME/robot/maps/$1.yaml 2>/dev/null

echo "Starting NAVIGATION with map: $1"
ros2 launch $HOME/robot/launch/navigation_launch.py
EOF

chmod +x ~/robot/scripts/start_navigation.sh
echo -e "${GREEN}  ✓ start_navigation.sh written${NC}"

# ══════════════════════════════════════════════════════
# VERIFY
# ══════════════════════════════════════════════════════
echo ""
echo "Verifying navigation_launch.py syntax..."
python3 -c "import ast; ast.parse(open('$HOME/robot/launch/navigation_launch.py').read()); print('  ✓ No syntax errors')"

echo ""
echo "Checking map file..."
if [ -f ~/robot/maps/my_lawn.yaml ]; then
    IMAGE_PATH=$(grep "^image:" ~/robot/maps/my_lawn.yaml | awk '{print $2}')
    echo "  map yaml image path: $IMAGE_PATH"
    if [ -f "$IMAGE_PATH" ]; then
        echo -e "  ${GREEN}✓ Map image found${NC}"
    else
        echo "  Fixing map yaml image path..."
        sed -i "s|image:.*|image: $HOME/robot/maps/my_lawn.pgm|g" ~/robot/maps/my_lawn.yaml
        echo -e "  ${GREEN}✓ Map yaml fixed${NC}"
    fi
else
    echo "  WARNING: ~/robot/maps/my_lawn.yaml not found"
    echo "  Run robot_map first to create a map!"
fi

echo ""
echo "================================================================"
echo -e "${GREEN}   ALL FILES FIXED!${NC}"
echo "================================================================"
echo ""
echo "Now run:"
echo -e "  ${BLUE}robot_navigate my_lawn${NC}"
echo ""
echo "Wait 15 seconds for RViz to open, then in a NEW terminal:"
echo -e "  ${BLUE}ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: \"map\"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0,0,0.25,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.07]}}'${NC}"
echo ""
echo "Then in RViz click '2D Pose Estimate' to refine position."
echo ""
