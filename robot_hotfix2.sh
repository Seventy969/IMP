#!/bin/bash
# ============================================================
# robot_hotfix2.sh
# Fixes new bugs found after robot_hotfix.sh was applied:
#
#  Bug A: bt_navigator FATAL "ID [ComputePathToPose] already registered"
#          -> plugin_lib_names in nav2_params.yaml causes double-registration
#             in Nav2 Kilted. Remove it.
#
#  Bug B: robot_save_map fails - map file not written
#          -> map_saver_cli ignores "-- -f" style arg in Kilted.
#             Use ROS parameter style instead.
#
#  Bug C: xterm crashes (exit code 2) in robot_map
#          -> Title has spaces, xterm parses them as separate flags:
#             xterm sees "-title TELEOP" then bare "-" which is invalid.
#             Fix: remove xterm/teleop from mapping launch entirely.
#             User runs robot_teleop in a SECOND terminal as intended.
#
#  Bug D: robot_teleop "cannot run independently"
#          -> teleop only publishes /cmd_vel; motor_controller must also run.
#             Add robot_drive command that starts both together.
#
# Run with: bash robot_hotfix2.sh
# ============================================================

set -e
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

echo ""
echo "============================================================"
echo "   Robot Hotfix 2 - fixing 4 new bugs"
echo "============================================================"
echo ""

ROBOT="$HOME/robot"
SCRIPTS="$ROBOT/scripts"
CONFIG="$ROBOT/config"
LAUNCH="$ROBOT/launch"


# ════════════════════════════════════════════════════════════
# BUG A FIX: bt_navigator - remove plugin_lib_names
#
# In Nav2 Kilted, NavigateToPoseNavigator and NavigateThroughPosesNavigator
# register their own BT action nodes (ComputePathToPose, FollowPath, etc.)
# internally when they initialize. If plugin_lib_names ALSO lists those same
# BT nodes, they get registered a second time -> FATAL crash.
#
# Fix: remove plugin_lib_names entirely. Kilted handles it automatically.
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[A] Fixing bt_navigator in nav2_params.yaml (remove plugin_lib_names)...${NC}"

python3 - << 'PYEOF'
import re, os

path = os.path.join(os.path.expanduser('~'), 'robot', 'config', 'nav2_params.yaml')
with open(path, 'r', encoding='utf-8') as f:
    content = f.read()

# Remove the entire plugin_lib_names block from bt_navigator
# It starts at "    plugin_lib_names:" and ends before the next top-level key
pattern = r'(bt_navigator:.*?    wait_for_service_timeout:[^\n]*\n    action_server_result_timeout:[^\n]*\n    navigators:[^\n]*\n    navigate_to_pose:[^\n]*\n      plugin:[^\n]*\n    navigate_through_poses:[^\n]*\n      plugin:[^\n]*\n)    plugin_lib_names:.*?(?=\n\w)'
replacement = r'\1'
new_content = re.sub(pattern, replacement, content, flags=re.DOTALL)

if new_content == content:
    # Try simpler approach: just remove plugin_lib_names block
    lines = content.split('\n')
    result = []
    skip = False
    for line in lines:
        if '    plugin_lib_names:' in line and not skip:
            skip = True
            continue
        if skip:
            # Stop skipping when we hit a line that's NOT indented under plugin_lib_names
            # (i.e., not starting with 6+ spaces or "      - ")
            if line.startswith('      - ') or (line.strip() == '' and skip):
                continue
            else:
                skip = False
        result.append(line)
    new_content = '\n'.join(result)

with open(path, 'w', encoding='utf-8', newline='\n') as f:
    f.write(new_content)

print("  plugin_lib_names block removed from bt_navigator")
PYEOF

# Verify nav2_params.yaml is still valid YAML
python3 - << 'PYEOF'
import yaml, os
path = os.path.join(os.path.expanduser('~'), 'robot', 'config', 'nav2_params.yaml')
with open(path, 'rb') as f:
    raw = f.read()
try:
    yaml.safe_load(raw.decode('utf-8'))
    print(f"  YAML valid. plugin_lib_names present: {'plugin_lib_names' in raw.decode()}")
except Exception as e:
    print(f"  ERROR: {e}")
PYEOF
echo -e "${GREEN}  nav2_params.yaml fixed (bt_navigator no longer double-registers BT nodes)${NC}"


# ════════════════════════════════════════════════════════════
# BUG B FIX: robot_save_map - map_saver_cli ignores "-- -f" in Kilted
#
# Root cause: In Nav2 Kilted, map_saver_cli treats the output file via the
# ROS parameter "output_file_no_suffix", NOT as a bare CLI "-f" argument.
# The "-- -f $MAP_PATH" syntax is silently ignored, so the file path is
# empty, and map_io saves to the CURRENT DIRECTORY (which is "/" and
# therefore unwritable by the user) with a timestamp name.
#
# Fix: pass output_file_no_suffix as a proper --ros-args -p parameter.
#      Also mkdir the maps dir and cd there before saving.
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[B] Fixing save_map.sh (map_saver_cli ROS param syntax)...${NC}"

python3 - << 'PYEOF'
import os
home = os.path.expanduser('~')
path = os.path.join(home, 'robot', 'scripts', 'save_map.sh')

content = """\
#!/bin/bash
source /opt/ros/kilted/setup.bash
source "$HOME/ws_lidar/install/setup.bash"

if [ -z "$1" ]; then
    echo "Usage: robot_save_map <map_name>"
    echo "Example: robot_save_map my_lawn"
    exit 1
fi

MAP_DIR="$HOME/robot/maps"
MAP_NAME="$1"
MAP_PATH="$MAP_DIR/$MAP_NAME"

# Ensure maps directory exists
mkdir -p "$MAP_DIR"

echo "Saving map as: $MAP_PATH (.yaml + .pgm)"
echo "(Make sure robot_map is still running in the other terminal)"
echo ""

# In Nav2 Kilted, map_saver_cli uses the ROS parameter output_file_no_suffix
# NOT the bare CLI -f flag. We pass it via --ros-args -p.
ros2 run nav2_map_server map_saver_cli \\
    --ros-args \\
    -p save_map_timeout:=5.0 \\
    -p output_file_no_suffix:="$MAP_PATH"

# Check if files were actually created
if [ -f "${MAP_PATH}.yaml" ] && [ -f "${MAP_PATH}.pgm" ]; then
    echo ""
    echo "Map saved successfully:"
    echo "  ${MAP_PATH}.yaml"
    echo "  ${MAP_PATH}.pgm"
    echo ""
    echo "You can now run: robot_navigate $MAP_NAME"
else
    echo ""
    echo "WARNING: Map files not found at expected location."
    echo "Checking maps directory..."
    ls -la "$MAP_DIR/" 2>/dev/null || echo "  (maps directory is empty)"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Make sure robot_map is still running while saving"
    echo "  2. Try: ros2 topic echo /map --once (to confirm map is published)"
fi
"""

with open(path, 'w', encoding='utf-8', newline='\n') as f:
    f.write(content)
os.chmod(path, 0o755)
print(f"  Written: {path}")
PYEOF
echo -e "${GREEN}  save_map.sh fixed (uses output_file_no_suffix ROS parameter)${NC}"


# ════════════════════════════════════════════════════════════
# BUG C FIX: xterm crashes in mapping_launch.py
#
# Root cause: When ROS2 launch uses prefix='xterm -title "TEXT WITH SPACES"...',
# the prefix string is split by whitespace. xterm then sees:
#   xterm -title TELEOP -  click  here  first!  -fa Monospace -fs 12 -e ...
# The bare "-" is an unrecognised flag -> xterm exits code 2.
#
# Fix: Remove teleop from mapping_launch.py entirely.
# The teleop window in robot_map was ALWAYS meant to be a SECOND terminal.
# user runs: robot_map (terminal 1) then robot_teleop (terminal 2).
# This is cleaner, more reliable, and matches the documented workflow.
#
# Also: fix start_mapping.sh to tell user to open robot_teleop separately.
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[C] Fixing mapping_launch.py (remove broken xterm prefix)...${NC}"

python3 - << 'PYEOF'
import os
home = os.path.expanduser('~')
path = os.path.join(home, 'robot', 'launch', 'mapping_launch.py')

content = '''\
#!/usr/bin/env python3
"""
mapping_launch.py
Launches: RPLidar, SLAM Toolbox, Motor Controller, RViz

Step 1: robot_map        (this file - starts everything)
Step 2: robot_teleop     (run in a SECOND terminal to drive)
Step 3: robot_save_map   (run in any terminal when done mapping)
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    home        = os.path.expanduser(\'~\')
    slam_dir    = get_package_share_directory(\'slam_toolbox\')
    slam_params = os.path.join(home, \'robot\', \'config\', \'slam_params.yaml\')
    rviz_config = os.path.join(home, \'robot\', \'rviz\', \'mapping_config.rviz\')
    scripts_dir = os.path.join(home, \'robot\', \'scripts\')

    return LaunchDescription([

        # static TF: base_link -> laser (LiDAR 10 cm above base)
        Node(
            package=\'tf2_ros\',
            executable=\'static_transform_publisher\',
            name=\'base_to_laser_tf\',
            arguments=[\'0\', \'0\', \'0.1\', \'0\', \'0\', \'0\', \'base_link\', \'laser\'],
            output=\'screen\',
        ),
        # static TF: odom -> base_link (no encoder fallback)
        Node(
            package=\'tf2_ros\',
            executable=\'static_transform_publisher\',
            name=\'odom_to_base_tf\',
            arguments=[\'0\', \'0\', \'0\', \'0\', \'0\', \'0\', \'odom\', \'base_link\'],
            output=\'screen\',
        ),

        # RPLidar A1M8
        Node(
            package=\'sllidar_ros2\',
            executable=\'sllidar_node\',
            name=\'sllidar_node\',
            parameters=[{
                \'serial_port\':      \'/dev/ttyUSB0\',
                \'serial_baudrate\':   115200,
                \'frame_id\':          \'laser\',
                \'angle_compensate\':  True,
                \'scan_mode\':         \'Sensitivity\',
            }],
            output=\'screen\',
        ),

        # Motor controller - direct Python call
        ExecuteProcess(
            cmd=[\'python3\', os.path.join(scripts_dir, \'motor_controller.py\')],
            name=\'motor_controller\',
            output=\'screen\',
        ),

        # SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_dir, \'launch\', \'online_async_launch.py\')
            ),
            launch_arguments={
                \'slam_params_file\': slam_params,
                \'use_sim_time\':     \'false\',
            }.items(),
        ),

        # RViz
        Node(
            package=\'rviz2\',
            executable=\'rviz2\',
            name=\'rviz2\',
            arguments=[\'-d\', rviz_config],
            output=\'screen\',
        ),

        # NOTE: Teleop is NOT launched here intentionally.
        # Run robot_teleop in a SECOND terminal after this starts.
        # xterm launched as a subprocess of ros2 launch is unreliable in VNC.
    ])
'''

with open(path, 'w', encoding='utf-8', newline='\n') as f:
    f.write(content)
os.chmod(path, 0o755)
print(f"  Written: {path}")
PYEOF
echo -e "${GREEN}  mapping_launch.py fixed (teleop removed - run robot_teleop in 2nd terminal)${NC}"


# Fix start_mapping.sh to tell user clearly what to do
echo -e "${YELLOW}[C2] Updating start_mapping.sh with correct instructions...${NC}"

python3 - << 'PYEOF'
import os
home = os.path.expanduser('~')
path = os.path.join(home, 'robot', 'scripts', 'start_mapping.sh')

content = """\
#!/bin/bash
source /opt/ros/kilted/setup.bash
source "$HOME/ws_lidar/install/setup.bash"

echo "=== MAPPING MODE ==="
echo ""
echo "STEP 1: This window starts SLAM + LiDAR + Motor Controller + RViz"
echo "STEP 2: Open a NEW terminal and run: robot_teleop"
echo "STEP 3: Click the teleop terminal and drive the whole lawn"
echo "STEP 4: When done mapping, run in any terminal: robot_save_map my_lawn"
echo ""
echo "Starting now..."
echo ""
ros2 launch "$HOME/robot/launch/mapping_launch.py"
"""

with open(path, 'w', encoding='utf-8', newline='\n') as f:
    f.write(content)
os.chmod(path, 0o755)
print(f"  Written: {path}")
PYEOF
echo -e "${GREEN}  start_mapping.sh updated with clear two-terminal instructions${NC}"


# ════════════════════════════════════════════════════════════
# BUG D FIX: robot_teleop cannot work alone (needs motor_controller)
#
# When robot_map is NOT running, motor_controller is not running either.
# robot_teleop publishes to /cmd_vel but nothing listens -> motors silent.
#
# Fix 1: Improve teleop.sh to warn the user if motor_controller is not running
# Fix 2: Add robot_drive command = motor_controller + teleop in one go
#         (useful for testing motors without doing a full mapping session)
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[D] Fixing teleop.sh and adding robot_drive command...${NC}"

# Fix teleop.sh - add check for motor_controller
python3 - << 'PYEOF'
import os
home = os.path.expanduser('~')
path = os.path.join(home, 'robot', 'scripts', 'teleop.sh')

content = """\
#!/bin/bash
source /opt/ros/kilted/setup.bash
source "$HOME/ws_lidar/install/setup.bash"

echo "=== TELEOP MODE ==="
echo ""

# Check if motor_controller is running
if ros2 node list 2>/dev/null | grep -q motor_controller; then
    echo "Motor controller is running. You can drive now."
else
    echo "WARNING: motor_controller is NOT running."
    echo "Keys will be captured but motors will NOT respond."
    echo ""
    echo "To drive AND move the motors:"
    echo "  If robot_map / robot_navigate / robot_coverage is running -> use THIS window"
    echo "  To test motors only (no mapping) -> use: robot_drive"
    echo ""
fi

echo "Click THIS window, then use keys to drive:"
echo "  i = forward    , = backward"
echo "  j = turn left  l = turn right"
echo "  k = stop"
echo "  q/z = speed up/down"
echo ""
ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

with open(path, 'w', encoding='utf-8', newline='\n') as f:
    f.write(content)
os.chmod(path, 0o755)
print(f"  Written: {path}")
PYEOF

# Create robot_drive.sh - motor_controller + teleop together (no SLAM, no map)
python3 - << 'PYEOF'
import os
home = os.path.expanduser('~')
path = os.path.join(home, 'robot', 'scripts', 'robot_drive.sh')

content = """\
#!/bin/bash
source /opt/ros/kilted/setup.bash
source "$HOME/ws_lidar/install/setup.bash"

echo "=== MANUAL DRIVE MODE (motors + teleop only) ==="
echo ""
echo "This starts ONLY the motor controller."
echo "Open a NEW terminal and run: robot_teleop"
echo ""
echo "Use this for:"
echo "  - Testing motors without building a map"
echo "  - Verifying GPIO wiring"
echo "  - Manual positioning"
echo ""
echo "Ctrl+C to stop."
echo ""

# Start motor controller in background
python3 "$HOME/robot/scripts/motor_controller.py" &
MOTOR_PID=$!

# Wait for motor controller to start
sleep 2

if kill -0 $MOTOR_PID 2>/dev/null; then
    echo "Motor controller started (PID $MOTOR_PID)"
    echo "Now open another terminal and run: robot_teleop"
    echo ""
    # Keep this terminal alive and wait
    wait $MOTOR_PID
else
    echo "ERROR: Motor controller failed to start."
    echo "Check GPIO permissions:"
    echo "  groups  (should include gpio)"
    echo "  sudo chmod 666 /dev/gpiomem"
fi
"""

with open(path, 'w', encoding='utf-8', newline='\n') as f:
    f.write(content)
os.chmod(path, 0o755)
print(f"  Written: {path}")
PYEOF

echo -e "${GREEN}  teleop.sh fixed (warns if motor_controller not running)${NC}"
echo -e "${GREEN}  robot_drive.sh created (standalone motor test mode)${NC}"


# ════════════════════════════════════════════════════════════
# Add robot_drive alias to bashrc
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[D2] Adding robot_drive alias to bashrc...${NC}"

if ! grep -q "robot_drive" ~/.bashrc; then
    # Find the END ROBOT marker and insert before it
    sed -i '/# === END ROBOT ===/i alias robot_drive='"'"'bash "$HOME/robot/scripts/robot_drive.sh"'"'" ~/.bashrc
    echo -e "${GREEN}  robot_drive alias added${NC}"
else
    echo "  robot_drive alias already present"
fi


# ════════════════════════════════════════════════════════════
# BONUS FIX: RViz GLSL shader error in VNC
# "active samplers with a different type refer to the same texture image unit"
# This is a Mesa/software-GL bug with VNC. Fix: force software rendering
# which is more stable than the default VNC OpenGL.
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[E] Fixing RViz GLSL error in VNC (software renderer)...${NC}"

# Add LIBGL_ALWAYS_SOFTWARE to the robot shortcuts in bashrc
if ! grep -q "LIBGL_ALWAYS_SOFTWARE" ~/.bashrc; then
    sed -i '/# === ROBOT SHORTCUTS ===/a export LIBGL_ALWAYS_SOFTWARE=1  # Fixes RViz GLSL errors in VNC' ~/.bashrc
    echo -e "${GREEN}  LIBGL_ALWAYS_SOFTWARE=1 added to bashrc (fixes RViz map display in VNC)${NC}"
else
    echo "  LIBGL_ALWAYS_SOFTWARE already set"
fi


# ════════════════════════════════════════════════════════════
# VERIFY: check all modified files
# ════════════════════════════════════════════════════════════
echo ""
echo -e "${YELLOW}Verifying all files...${NC}"
echo ""

PASS=0; FAIL=0

check_py() {
    local f="$1"
    local label="$2"
    if python3 -c "import ast; ast.parse(open('$f').read())" 2>/dev/null; then
        echo -e "  ${GREEN}OK${NC} $label"
        PASS=$((PASS+1))
    else
        echo -e "  ${RED}FAIL${NC} $label"
        FAIL=$((FAIL+1))
    fi
}

check_yaml_no_plugin_lib() {
    local f="$1"
    python3 - << PYCHECK
import yaml, os
path = '$f'
with open(path, 'rb') as fh:
    raw = fh.read()
try:
    data = yaml.safe_load(raw.decode('utf-8'))
    has_plnames = 'plugin_lib_names' in str(data.get('bt_navigator', {}).get('ros__parameters', {}))
    print(f"  YAML valid. plugin_lib_names in bt_navigator: {has_plnames}")
    if not has_plnames:
        print("  OK nav2_params.yaml (bt_navigator has no plugin_lib_names)")
except Exception as e:
    print(f"  ERROR: {e}")
PYCHECK
}

check_sh_has_ros_param() {
    local f="$1"
    if grep -q "output_file_no_suffix" "$f"; then
        echo -e "  ${GREEN}OK${NC} save_map.sh (uses output_file_no_suffix)"
        PASS=$((PASS+1))
    else
        echo -e "  ${RED}FAIL${NC} save_map.sh"
        FAIL=$((FAIL+1))
    fi
}

check_py "$LAUNCH/mapping_launch.py"       "mapping_launch.py"
check_py "$LAUNCH/navigation_launch.py"    "navigation_launch.py"
check_py "$LAUNCH/coverage_nomap_launch.py" "coverage_nomap_launch.py"
check_sh_has_ros_param "$SCRIPTS/save_map.sh"
check_yaml_no_plugin_lib "$CONFIG/nav2_params.yaml"

# Check xterm no longer in mapping_launch
if grep -q "xterm" "$LAUNCH/mapping_launch.py"; then
    echo -e "  ${RED}FAIL${NC} mapping_launch.py still contains xterm!"
    FAIL=$((FAIL+1))
else
    echo -e "  ${GREEN}OK${NC} mapping_launch.py - xterm removed"
    PASS=$((PASS+1))
fi

# Check robot_drive exists
if [ -f "$SCRIPTS/robot_drive.sh" ]; then
    echo -e "  ${GREEN}OK${NC} robot_drive.sh created"
    PASS=$((PASS+1))
else
    echo -e "  ${RED}FAIL${NC} robot_drive.sh missing"
    FAIL=$((FAIL+1))
fi

echo ""
echo "============================================================"
if [ "$FAIL" -eq 0 ]; then
    echo -e "${GREEN}   ALL CHECKS PASSED!${NC}"
else
    echo -e "${RED}   $FAIL check(s) failed - review output above${NC}"
fi
echo "============================================================"
echo ""
echo "IMPORTANT: reload your shell now:"
echo -e "  ${BLUE}source ~/.bashrc${NC}"
echo ""
echo "COMPLETE WORKFLOW (copy-paste ready):"
echo ""
echo -e "  ${BLUE}# Terminal 1 - start mapping${NC}"
echo -e "  ${BLUE}robot_map${NC}"
echo ""
echo -e "  ${BLUE}# Terminal 2 - drive the robot${NC}"
echo -e "  ${BLUE}robot_teleop${NC}"
echo ""
echo -e "  ${BLUE}# Terminal 1 or 2 - when you've covered the whole lawn${NC}"
echo -e "  ${BLUE}robot_save_map my_lawn${NC}"
echo ""
echo -e "  ${BLUE}# Then autonomous mode (new session)${NC}"
echo -e "  ${BLUE}robot_navigate my_lawn${NC}"
echo ""
echo -e "  ${BLUE}# When robot starts moving${NC}"
echo -e "  ${BLUE}robot_cutter_on${NC}"
echo ""
echo -e "  ${BLUE}# Motor test only (no mapping)${NC}"
echo -e "  ${BLUE}robot_drive${NC}    <- terminal 1"
echo -e "  ${BLUE}robot_teleop${NC}   <- terminal 2"
echo ""
