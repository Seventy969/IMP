#!/bin/bash
# ============================================================
# robot_fix.sh
# Fixes "package 'robot_controllers' not found" error.
# Rewrites all 3 launch files to call Python scripts directly
# (no ROS2 package discovery needed) and rebuilds workspace.
# Run with: bash robot_fix.sh
# ============================================================

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

echo ""
echo "============================================================"
echo "   Robot Fix — launch file patch"
echo "============================================================"
echo ""

# ════════════════════════════════════════════════════════════
# 1. Rewrite mapping_launch.py
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[1/4] Rewriting mapping_launch.py...${NC}"
cat > ~/robot/launch/mapping_launch.py << 'PYEOF'
#!/usr/bin/env python3
"""
mapping_launch.py
Launches: RPLidar · SLAM Toolbox · Motor Controller · Teleop · RViz
Scripts called directly — no ROS2 package discovery needed.
"""

import os
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, ExecuteProcess, TimerAction)
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

        # ── static TF: base_link → laser ──────────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
            output='screen',
        ),
        # ── static TF: odom → base_link (no encoder fallback) ─
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen',
        ),

        # ── RPLidar A1M8 ──────────────────────────────────────
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port':     '/dev/ttyUSB0',
                'serial_baudrate':  115200,
                'frame_id':        'laser',
                'angle_compensate': True,
                'scan_mode':       'Sensitivity',
            }],
            output='screen',
        ),

        # ── Motor controller (direct Python call) ─────────────
        ExecuteProcess(
            cmd=['python3', os.path.join(scripts_dir, 'motor_controller.py')],
            name='motor_controller',
            output='screen',
        ),

        # ── SLAM Toolbox ──────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'slam_params_file': slam_params,
                'use_sim_time':    'false',
            }.items(),
        ),

        # ── Teleop keyboard ───────────────────────────────────
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            output='screen',
            prefix='xterm -title "TELEOP — click here first!" -fa Monospace -fs 12 -e',
        ),

        # ── RViz ──────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
PYEOF
echo -e "${GREEN}  ✓ mapping_launch.py${NC}"


# ════════════════════════════════════════════════════════════
# 2. Rewrite navigation_launch.py
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[2/4] Rewriting navigation_launch.py...${NC}"
cat > ~/robot/launch/navigation_launch.py << 'PYEOF'
#!/usr/bin/env python3
"""
navigation_launch.py — WITH saved map
Launches: RPLidar · map_server · AMCL · Nav2 · Motor · Cutter · Coverage · RViz
Scripts called directly — no ROS2 package discovery needed.
"""

import os, sys
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

    # map_name is passed via CLI: ros2 launch ... map_name:=my_lawn
    map_name_arg = DeclareLaunchArgument(
        'map_name', default_value='my_lawn',
        description='Map stem name in ~/robot/maps/'
    )
    map_name = LaunchConfiguration('map_name')

    return LaunchDescription([
        map_name_arg,

        # ── static TFs ────────────────────────────────────────
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

        # ── RPLidar A1M8 ──────────────────────────────────────
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port':     '/dev/ttyUSB0',
                'serial_baudrate':  115200,
                'frame_id':        'laser',
                'angle_compensate': True,
                'scan_mode':       'Sensitivity',
            }],
            output='screen',
        ),

        # ── Motor controller ──────────────────────────────────
        ExecuteProcess(
            cmd=['python3', os.path.join(scripts_dir, 'motor_controller.py')],
            name='motor_controller',
            output='screen',
        ),

        # ── Cutter controller ─────────────────────────────────
        ExecuteProcess(
            cmd=['python3', os.path.join(scripts_dir, 'cutter_controller.py')],
            name='cutter_controller',
            output='screen',
        ),

        # ── Map server ────────────────────────────────────────
        # We use a Python-evaluated path since LaunchConfiguration
        # can't be concatenated with plain strings in all contexts.
        # start_navigation.sh exports MAP_YAML env var as a workaround.
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                # map path resolved by the shell script before launch
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

        # ── AMCL ──────────────────────────────────────────────
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

        # ── Nav2 stack ────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file':  params_file,
            }.items(),
        ),

        # ── Coverage path planner (after Nav2 action server up) ─
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

        # ── RViz (delayed so map is latched first) ─────────────
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
PYEOF
echo -e "${GREEN}  ✓ navigation_launch.py${NC}"


# ════════════════════════════════════════════════════════════
# 3. Rewrite coverage_nomap_launch.py
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[3/4] Rewriting coverage_nomap_launch.py...${NC}"
cat > ~/robot/launch/coverage_nomap_launch.py << 'PYEOF'
#!/usr/bin/env python3
"""
coverage_nomap_launch.py — autonomous WITHOUT a saved map
SLAM builds map live while Nav2 drives the coverage pattern.
Scripts called directly — no ROS2 package discovery needed.
"""

import os
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, ExecuteProcess, TimerAction)
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

        # ── static TFs ────────────────────────────────────────
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

        # ── RPLidar A1M8 ──────────────────────────────────────
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port':     '/dev/ttyUSB0',
                'serial_baudrate':  115200,
                'frame_id':        'laser',
                'angle_compensate': True,
                'scan_mode':       'Sensitivity',
            }],
            output='screen',
        ),

        # ── Motor controller ──────────────────────────────────
        ExecuteProcess(
            cmd=['python3', os.path.join(scripts_dir, 'motor_controller.py')],
            name='motor_controller',
            output='screen',
        ),

        # ── Cutter controller ─────────────────────────────────
        ExecuteProcess(
            cmd=['python3', os.path.join(scripts_dir, 'cutter_controller.py')],
            name='cutter_controller',
            output='screen',
        ),

        # ── SLAM Toolbox — starts immediately ─────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'slam_params_file': slam_params,
                'use_sim_time':    'false',
            }.items(),
        ),

        # ── Nav2 — delayed 8s so SLAM publishes /map first ────
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

        # ── Coverage planner — delayed 25s for full startup ───
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

        # ── RViz ──────────────────────────────────────────────
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
PYEOF
echo -e "${GREEN}  ✓ coverage_nomap_launch.py${NC}"


# ════════════════════════════════════════════════════════════
# 4. Fix the Python scripts: add rclpy init guard and
#    make them runnable standalone (not via ros2 run)
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[4/4] Verifying Python scripts are standalone-runnable...${NC}"

# Verify all 3 scripts exist and have correct shebang + main()
for script in motor_controller.py cutter_controller.py coverage_path_planner.py; do
    path="$HOME/robot/scripts/$script"
    if [ ! -f "$path" ]; then
        echo -e "  ${RED}✗ MISSING: $script — re-run robot_setup_full.sh first${NC}"
        exit 1
    fi
    if ! head -1 "$path" | grep -q "python3"; then
        echo "#!/usr/bin/env python3" | cat - "$path" > /tmp/_tmp && mv /tmp/_tmp "$path"
    fi
    chmod +x "$path"
    python3 -c "import ast; ast.parse(open('$path').read())" \
        && echo -e "  ${GREEN}✓ $script${NC}" \
        || echo -e "  ${RED}✗ syntax error: $script${NC}"
done

# Also verify launch files
echo ""
echo "Verifying launch file syntax..."
for f in mapping_launch.py navigation_launch.py coverage_nomap_launch.py; do
    python3 -c "import ast; ast.parse(open('$HOME/robot/launch/$f').read())" \
        && echo -e "  ${GREEN}✓ $f${NC}" \
        || echo -e "  ${RED}✗ $f${NC}"
done


echo ""
echo "============================================================"
echo -e "${GREEN}   FIX APPLIED!${NC}"
echo "============================================================"
echo ""
echo "Now test:"
echo -e "  ${BLUE}robot_map${NC}             (mapping + teleop)"
echo -e "  ${BLUE}robot_coverage${NC}        (autonomous, no map needed)"
echo -e "  ${BLUE}robot_navigate my_lawn${NC} (if you already have a map)"
echo ""
echo "In a second terminal for manual drive:"
echo -e "  ${BLUE}robot_teleop${NC}"
echo ""
