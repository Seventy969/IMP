#!/bin/bash
#
# setup_robot_complete.sh
# Creates ALL robot files from scratch and organizes into ~/robot/
# Run this ONCE after completing Part 11 of the fresh setup guide
#
# Usage: bash setup_robot_complete.sh
#

set -e  # Exit on any error

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

echo ""
echo "================================================================"
echo "   Lawn Mower Robot - Complete File Setup"
echo "================================================================"
echo ""
echo "This script will:"
echo "  1. Create motor_controller.py"
echo "  2. Create nav2_params.yaml"
echo "  3. Create mapping_launch.py"
echo "  4. Create navigation_launch.py"
echo "  5. Create helper scripts"
echo "  6. Organize everything into ~/robot/"
echo ""
read -p "Press Enter to continue or Ctrl+C to cancel..."
echo ""

# ══════════════════════════════════════════════════════════════════
# STEP 1: Create motor_controller.py
# ══════════════════════════════════════════════════════════════════
echo -e "${YELLOW}[1/6] Creating motor_controller.py...${NC}"

cat > ~/motor_controller.py << 'MOTOR_EOF'
#!/usr/bin/env python3
"""
Motor Controller Node for Lawn Mower Robot
Converts /cmd_vel to L298N GPIO signals
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )
        
        # GPIO pins for L298N #1 (Drive Motors)
        self.LEFT_IN1 = 13
        self.LEFT_IN2 = 19
        self.LEFT_ENA = 26
        
        self.RIGHT_IN3 = 5
        self.RIGHT_IN4 = 6
        self.RIGHT_ENB = 0
        
        # !! MEASURE YOUR ROBOT'S WHEEL SEPARATION !!
        # Distance between wheel centers in METERS
        self.WHEEL_SEPARATION = 0.50  # CHANGE THIS!
        
        self.MAX_SPEED = 0.5
        self.MAX_ANGULAR = 2.0
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        GPIO.setup(self.LEFT_IN1, GPIO.OUT)
        GPIO.setup(self.LEFT_IN2, GPIO.OUT)
        GPIO.setup(self.LEFT_ENA, GPIO.OUT)
        GPIO.setup(self.RIGHT_IN3, GPIO.OUT)
        GPIO.setup(self.RIGHT_IN4, GPIO.OUT)
        GPIO.setup(self.RIGHT_ENB, GPIO.OUT)
        
        self.left_pwm = GPIO.PWM(self.LEFT_ENA, 1000)
        self.right_pwm = GPIO.PWM(self.RIGHT_ENB, 1000)
        
        self.left_pwm.start(0)
        self.right_pwm.start(0)
        
        self.last_cmd_time = self.get_clock().now()
        self.timeout = 1.0
        self.timer = self.create_timer(0.1, self.safety_check)
        
        self.get_logger().info('Motor Controller Started')
        self.get_logger().info(f'Wheel Separation: {self.WHEEL_SEPARATION}m')
        
    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()
        
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        left_speed = linear_vel - (angular_vel * self.WHEEL_SEPARATION / 2.0)
        right_speed = linear_vel + (angular_vel * self.WHEEL_SEPARATION / 2.0)
        
        self.set_motor_speed(left_speed, right_speed)
        
    def set_motor_speed(self, left_speed, right_speed):
        left_speed = max(-1.0, min(1.0, left_speed / self.MAX_SPEED))
        right_speed = max(-1.0, min(1.0, right_speed / self.MAX_SPEED))
        
        # LEFT MOTOR
        if left_speed > 0:
            GPIO.output(self.LEFT_IN1, GPIO.HIGH)
            GPIO.output(self.LEFT_IN2, GPIO.LOW)
            pwm_value = abs(left_speed) * 100
        elif left_speed < 0:
            GPIO.output(self.LEFT_IN1, GPIO.LOW)
            GPIO.output(self.LEFT_IN2, GPIO.HIGH)
            pwm_value = abs(left_speed) * 100
        else:
            GPIO.output(self.LEFT_IN1, GPIO.LOW)
            GPIO.output(self.LEFT_IN2, GPIO.LOW)
            pwm_value = 0
        self.left_pwm.ChangeDutyCycle(pwm_value)
        
        # RIGHT MOTOR
        if right_speed > 0:
            GPIO.output(self.RIGHT_IN3, GPIO.HIGH)
            GPIO.output(self.RIGHT_IN4, GPIO.LOW)
            pwm_value = abs(right_speed) * 100
        elif right_speed < 0:
            GPIO.output(self.RIGHT_IN3, GPIO.LOW)
            GPIO.output(self.RIGHT_IN4, GPIO.HIGH)
            pwm_value = abs(right_speed) * 100
        else:
            GPIO.output(self.RIGHT_IN3, GPIO.LOW)
            GPIO.output(self.RIGHT_IN4, GPIO.LOW)
            pwm_value = 0
        self.right_pwm.ChangeDutyCycle(pwm_value)
    
    def safety_check(self):
        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if time_since_cmd > self.timeout:
            self.set_motor_speed(0, 0)
            
    def shutdown(self):
        self.get_logger().info('Shutting down...')
        self.set_motor_speed(0, 0)
        time.sleep(0.1)
        self.left_pwm.stop()
        self.right_pwm.stop()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
MOTOR_EOF

chmod +x ~/motor_controller.py
echo -e "${GREEN}  ✓ motor_controller.py created${NC}"

# ══════════════════════════════════════════════════════════════════
# STEP 2: Create nav2_params.yaml
# ══════════════════════════════════════════════════════════════════
echo ""
echo -e "${YELLOW}[2/6] Creating nav2_params.yaml...${NC}"

cat > ~/nav2_params.yaml << 'NAV2_EOF'
# Nav2 Parameters - Lawn Mower Robot

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
    robot_model_type: "differential"
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
      plugin: "nav2_navfn_planner/NavfnPlanner"
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
      plugin: "nav2_behaviors/Spin"
    backup:
      plugin: "nav2_behaviors/BackUp"
    wait:
      plugin: "nav2_behaviors/Wait"
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
NAV2_EOF

echo -e "${GREEN}  ✓ nav2_params.yaml created${NC}"

# ══════════════════════════════════════════════════════════════════
# STEP 3: Create mapping_launch.py
# ══════════════════════════════════════════════════════════════════
echo ""
echo -e "${YELLOW}[3/6] Creating mapping_launch.py...${NC}"

cat > ~/ws_lidar/src/sllidar_ros2/launch/mapping_launch.py << 'MAPPING_EOF'
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    slam_params_file = os.path.join(
        slam_toolbox_dir, 'config', 'mapper_params_online_async.yaml'
    )

    # RViz config
    rviz_config = os.path.join(os.path.expanduser('~'), 'robot', 'rviz', 'mapping_config.rviz')
    if not os.path.exists(rviz_config):
        try:
            sllidar_dir = get_package_share_directory('sllidar_ros2')
            rviz_config = os.path.join(sllidar_dir, 'rviz', 'sllidar_ros2.rviz')
        except:
            rviz_config = ''

    nodes = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # Transform: base_link -> laser (10cm up)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_broadcaster',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
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

        # SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
            ]),
            launch_arguments={'slam_params_file': slam_params_file, 'use_sim_time': use_sim_time}.items()
        ),

        # Motor Controller
        ExecuteProcess(
            cmd=['python3', os.path.join(os.path.expanduser('~'), 'robot', 'scripts', 'motor_controller.py')],
            output='screen',
            name='motor_controller'
        ),

        # Teleop
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            output='screen',
            prefix='xterm -title "TELEOP - Click here!" -e',
        ),
    ]

    # RViz
    if rviz_config:
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ))
    else:
        nodes.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='screen'))

    return LaunchDescription(nodes)
MAPPING_EOF

chmod +x ~/ws_lidar/src/sllidar_ros2/launch/mapping_launch.py
echo -e "${GREEN}  ✓ mapping_launch.py created${NC}"

# ══════════════════════════════════════════════════════════════════
# STEP 4: Create navigation_launch.py
# ══════════════════════════════════════════════════════════════════
echo ""
echo -e "${YELLOW}[4/6] Creating navigation_launch.py...${NC}"

cat > ~/ws_lidar/src/sllidar_ros2/launch/navigation_launch.py << 'NAV_EOF'
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file = LaunchConfiguration('map', default=os.path.join(os.path.expanduser('~'), 'robot', 'maps', 'my_map.yaml'))
    params_file = LaunchConfiguration('params_file', default=os.path.join(os.path.expanduser('~'), 'robot', 'config', 'nav2_params.yaml'))

    # RViz config
    rviz_config = os.path.join(os.path.expanduser('~'), 'robot', 'rviz', 'navigation_config.rviz')
    if not os.path.exists(rviz_config):
        try:
            sllidar_dir = get_package_share_directory('sllidar_ros2')
            rviz_config = os.path.join(sllidar_dir, 'rviz', 'sllidar_ros2.rviz')
        except:
            rviz_config = ''

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=map_file),
        DeclareLaunchArgument('params_file', default_value=params_file),

        # Transform: MUST match mapping!
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_broadcaster',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
        ),

        # RPLidar
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'frame_id': 'laser',
                'angle_compensate': True,
                'scan_mode': 'Sensitivity'
            }],
            output='screen'
        ),

        # Motor Controller
        ExecuteProcess(
            cmd=['python3', os.path.join(os.path.expanduser('~'), 'robot', 'scripts', 'motor_controller.py')],
            output='screen'
        ),

        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file, 'use_sim_time': use_sim_time}]
        ),

        # Lifecycle Manager for Map Server
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'node_names': ['map_server']}]
        ),

        # AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file]
        ),

        # Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')]),
            launch_arguments={'use_sim_time': use_sim_time, 'params_file': params_file}.items()
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config] if rviz_config else [],
            output='screen'
        )
    ])
NAV_EOF

chmod +x ~/ws_lidar/src/sllidar_ros2/launch/navigation_launch.py
echo -e "${GREEN}  ✓ navigation_launch.py created${NC}"

# ══════════════════════════════════════════════════════════════════
# STEP 5: Create organization script
# ══════════════════════════════════════════════════════════════════
echo ""
echo -e "${YELLOW}[5/6] Creating organization script...${NC}"

cat > ~/organize_robot.sh << 'ORG_EOF'
#!/bin/bash

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo ""
echo "================================================="
echo "   Organizing Robot Files into ~/robot/"
echo "================================================="
echo ""

# Create structure
mkdir -p ~/robot/{maps,launch,scripts,rviz,config}

# Copy files
cp ~/motor_controller.py ~/robot/scripts/ 2>/dev/null
cp ~/nav2_params.yaml ~/robot/config/ 2>/dev/null
cp ~/ws_lidar/src/sllidar_ros2/launch/mapping_launch.py ~/robot/launch/ 2>/dev/null
cp ~/ws_lidar/src/sllidar_ros2/launch/navigation_launch.py ~/robot/launch/ 2>/dev/null

# Copy any existing maps
if ls ~/maps/*.yaml 2>/dev/null | grep -q .; then
    cp ~/maps/*.{yaml,pgm} ~/robot/maps/ 2>/dev/null
fi

# Create helper scripts
cat > ~/robot/scripts/start_mapping.sh << 'MAP_SCRIPT'
#!/bin/bash
source /opt/ros/kilted/setup.bash
source ~/ws_lidar/install/setup.bash
echo "Starting MAPPING mode..."
echo "Open another terminal and run: robot_teleop"
ros2 launch ~/robot/launch/mapping_launch.py
MAP_SCRIPT

cat > ~/robot/scripts/start_navigation.sh << 'NAV_SCRIPT'
#!/bin/bash
source /opt/ros/kilted/setup.bash
source ~/ws_lidar/install/setup.bash

if [ -z "$1" ]; then
    echo "Usage: robot_navigate <map_name>"
    echo "Available maps:"
    ls ~/robot/maps/*.yaml 2>/dev/null | xargs -I{} basename {} .yaml | sed 's/^/  - /'
    exit 1
fi

if [ ! -f ~/robot/maps/$1.yaml ]; then
    echo "Map not found: $1"
    exit 1
fi

echo "Starting NAVIGATION with map: $1"
ros2 launch ~/robot/launch/navigation_launch.py map:=~/robot/maps/$1.yaml
NAV_SCRIPT

cat > ~/robot/scripts/save_map.sh << 'SAVE_SCRIPT'
#!/bin/bash
source /opt/ros/kilted/setup.bash

if [ -z "$1" ]; then
    echo "Usage: robot_save_map <map_name>"
    exit 1
fi

ros2 run nav2_map_server map_saver_cli -f ~/robot/maps/$1
echo "Map saved: ~/robot/maps/$1.yaml"
SAVE_SCRIPT

cat > ~/robot/scripts/teleop.sh << 'TELEOP_SCRIPT'
#!/bin/bash
source /opt/ros/kilted/setup.bash
echo "TELEOP - Click this window before driving!"
ros2 run teleop_twist_keyboard teleop_twist_keyboard
TELEOP_SCRIPT

chmod +x ~/robot/scripts/*.sh

# Add aliases to bashrc
if ! grep -q "# === ROBOT SHORTCUTS ===" ~/.bashrc; then
    cat >> ~/.bashrc << 'ALIASES'

# === ROBOT SHORTCUTS ===
source /opt/ros/kilted/setup.bash 2>/dev/null
source ~/ws_lidar/install/setup.bash 2>/dev/null

alias robot_map='~/robot/scripts/start_mapping.sh'
alias robot_navigate='~/robot/scripts/start_navigation.sh'
alias robot_save_map='~/robot/scripts/save_map.sh'
alias robot_teleop='~/robot/scripts/teleop.sh'
alias robot_scan='ros2 topic hz /scan'
alias robot_status='ros2 node list'
# === END ROBOT ===
ALIASES
fi

echo -e "${GREEN}Done!${NC}"
echo ""
echo "Folder structure:"
echo "  ~/robot/"
echo "  ├── maps/       (saved maps)"
echo "  ├── launch/     (mapping & navigation)"
echo "  ├── scripts/    (helper scripts)"
echo "  ├── config/     (nav2_params.yaml)"
echo "  └── rviz/       (rviz configs)"
echo ""
echo -e "${BLUE}Run: source ~/.bashrc${NC}"
echo ""
echo "Then use:"
echo "  robot_map              - start mapping"
echo "  robot_teleop           - manual drive"
echo "  robot_save_map <name>  - save map"
echo "  robot_navigate <name>  - navigation"
echo ""
ORG_EOF

chmod +x ~/organize_robot.sh
echo -e "${GREEN}  ✓ organize_robot.sh created${NC}"

# ══════════════════════════════════════════════════════════════════
# STEP 6: Run organization
# ══════════════════════════════════════════════════════════════════
echo ""
echo -e "${YELLOW}[6/6] Organizing files into ~/robot/...${NC}"

bash ~/organize_robot.sh

# ══════════════════════════════════════════════════════════════════
# DONE
# ══════════════════════════════════════════════════════════════════
echo ""
echo "================================================================"
echo -e "${GREEN}   ALL FILES CREATED AND ORGANIZED!${NC}"
echo "================================================================"
echo ""
echo "Final step:"
echo -e "${BLUE}  source ~/.bashrc${NC}"
echo ""
echo "Then test:"
echo "  robot_scan    (check LiDAR is working)"
echo "  robot_map     (start mapping)"
echo ""
echo "Your robot is ready! 🤖"
echo ""
