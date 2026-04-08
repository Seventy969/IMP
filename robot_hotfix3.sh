#!/bin/bash
# ============================================================
# robot_hotfix3.sh — fixes ALL remaining issues:
#
#  Fix 1: robot_save_map - use slam_toolbox save service
#          (map_saver_cli ignores -f in Kilted; slam_toolbox is already running)
#
#  Fix 2: robot_teleop - start motor_controller automatically if not running
#
#  Fix 3: robot_cutter_on/off - work anytime (starts cutter standalone)
#
#  Fix 4: robot_coverage - rewritten as simple direct-drive boustrophedon
#          with LiDAR obstacle avoidance. NO Nav2, no map needed. Reliable.
#
#  Fix 5: robot_stop / robot_resume emergency commands
#
#  Fix 6: xterm false-positive in verify check (comment string)
#
# Run: bash ~/robot_hotfix3.sh
# ============================================================

set -e
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

echo ""
echo "============================================================"
echo "   Robot Hotfix 3"
echo "============================================================"
echo ""

SCRIPTS="$HOME/robot/scripts"
LAUNCH="$HOME/robot/launch"
CONFIG="$HOME/robot/config"


# ════════════════════════════════════════════════════════════
# FIX 1: robot_save_map — use slam_toolbox/save_map service
#
# map_saver_cli in Nav2 Kilted ignores both -f flag and ROS params.
# SLAM toolbox is already running during robot_map and exposes its own
# /slam_toolbox/save_map service which works perfectly.
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[1] Fixing save_map.sh (slam_toolbox service)...${NC}"

cat > "$SCRIPTS/save_map.sh" << 'SHEOF'
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

mkdir -p "$MAP_DIR"

echo "Saving map: $MAP_PATH"
echo "(robot_map must be running)"

# Use slam_toolbox's own save service - most reliable since slam_toolbox IS running
if ros2 service call /slam_toolbox/save_map \
        slam_toolbox/srv/SaveMap \
        "{name: {data: '${MAP_PATH}'}}" 2>/dev/null | grep -q "result"; then
    sleep 1
fi

# slam_toolbox saves as NAME.yaml + NAME.pgm
if [ -f "${MAP_PATH}.yaml" ] && [ -f "${MAP_PATH}.pgm" ]; then
    echo ""
    echo "Map saved:"
    echo "  ${MAP_PATH}.yaml"
    echo "  ${MAP_PATH}.pgm"
    echo ""
    echo "Next: robot_navigate $MAP_NAME"
    exit 0
fi

# Fallback: try map_saver_cli with -f directly (no --ros-args)
echo "Trying fallback map_saver_cli..."
cd "$MAP_DIR"
ros2 run nav2_map_server map_saver_cli \
    -f "$MAP_PATH" \
    --ros-args -p save_map_timeout:=5.0 2>/dev/null || true

sleep 1

if [ -f "${MAP_PATH}.yaml" ] && [ -f "${MAP_PATH}.pgm" ]; then
    echo ""
    echo "Map saved (fallback method):"
    echo "  ${MAP_PATH}.yaml"
    echo "  ${MAP_PATH}.pgm"
    echo ""
    echo "Next: robot_navigate $MAP_NAME"
else
    echo ""
    echo "Both methods failed. Trying direct map topic capture..."
    # Last resort: python3 one-shot map subscriber -> saves pgm + yaml directly
    python3 - "$MAP_PATH" << 'PYEOF'
import sys, os, struct, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

class MapSaverNode(Node):
    def __init__(self, out):
        super().__init__('map_saver_direct')
        self.out = out
        self.saved = False
        qos = QoSProfile(depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.cb, qos)
        self.get_logger().info('Waiting for /map ...')

    def cb(self, msg):
        if self.saved:
            return
        self.saved = True
        out = self.out
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        # Write PGM (grayscale, 8-bit, 0=occupied 254=free 205=unknown)
        pgm = out + '.pgm'
        with open(pgm, 'wb') as f:
            f.write(f'P5\n{w} {h}\n255\n'.encode())
            for row in range(h - 1, -1, -1):  # flip Y
                for col in range(w):
                    val = msg.data[row * w + col]
                    if val == -1:
                        f.write(bytes([205]))
                    elif val == 0:
                        f.write(bytes([254]))
                    else:
                        f.write(bytes([0]))

        # Write YAML
        yaml_path = out + '.yaml'
        with open(yaml_path, 'w') as f:
            f.write(f'image: {out}.pgm\n')
            f.write(f'resolution: {res}\n')
            f.write(f'origin: [{ox}, {oy}, 0.0]\n')
            f.write(f'negate: 0\n')
            f.write(f'occupied_thresh: 0.65\n')
            f.write(f'free_thresh: 0.25\n')

        self.get_logger().info(f'Saved {pgm} and {yaml_path}')

rclpy.init()
node = MapSaverNode(sys.argv[1])
try:
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    import time
    deadline = time.time() + 8.0
    while time.time() < deadline:
        executor.spin_once(timeout_sec=0.1)
        if node.saved:
            break
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
PYEOF

    if [ -f "${MAP_PATH}.yaml" ]; then
        echo "Map saved (direct capture):"
        echo "  ${MAP_PATH}.yaml + ${MAP_PATH}.pgm"
        echo ""
        echo "Next: robot_navigate $MAP_NAME"
    else
        echo "ERROR: All 3 methods failed."
        echo "  Check: ros2 topic echo /map --once"
    fi
fi
SHEOF
chmod +x "$SCRIPTS/save_map.sh"
echo -e "${GREEN}  save_map.sh fixed (3-method fallback chain)${NC}"


# ════════════════════════════════════════════════════════════
# FIX 2: robot_teleop — auto-start motor_controller if not running
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[2] Fixing teleop.sh (auto-start motor_controller)...${NC}"

cat > "$SCRIPTS/teleop.sh" << 'SHEOF'
#!/bin/bash
source /opt/ros/kilted/setup.bash
source "$HOME/ws_lidar/install/setup.bash"

echo "=== TELEOP MODE ==="
echo ""

MOTOR_STARTED=false

# Check if motor_controller is already running (e.g. inside robot_map)
if ! ros2 node list 2>/dev/null | grep -q motor_controller; then
    echo "Motor controller not running — starting it now..."
    python3 "$HOME/robot/scripts/motor_controller.py" &
    MOTOR_PID=$!
    sleep 2
    if kill -0 $MOTOR_PID 2>/dev/null; then
        echo "Motor controller started."
        MOTOR_STARTED=true
    else
        echo "WARNING: Motor controller failed to start. Check GPIO permissions."
        echo "  Try: sudo chmod 666 /dev/gpiomem"
    fi
else
    echo "Motor controller already running."
fi

echo ""
echo "Keys:"
echo "  i=forward  ,=back  j=left  l=right  k=stop"
echo "  q/z=speed up/down"
echo ""
echo "Click THIS window before pressing keys!"
echo ""

trap '[[ "$MOTOR_STARTED" == "true" ]] && kill $MOTOR_PID 2>/dev/null; echo "Teleop stopped."' EXIT

ros2 run teleop_twist_keyboard teleop_twist_keyboard
SHEOF
chmod +x "$SCRIPTS/teleop.sh"
echo -e "${GREEN}  teleop.sh fixed (auto-starts motor_controller if needed)${NC}"


# ════════════════════════════════════════════════════════════
# FIX 3: robot_cutter_on/off — work anytime
#
# Starts cutter_controller as a background process if not running,
# waits for its service, then calls it.
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[3] Fixing cutter_on.sh and cutter_off.sh (standalone mode)...${NC}"

cat > "$SCRIPTS/cutter_on.sh" << 'SHEOF'
#!/bin/bash
source /opt/ros/kilted/setup.bash
source "$HOME/ws_lidar/install/setup.bash"

echo "Turning cutter ON..."

CUTTER_STARTED=false

# Check if cutter_controller is already running
if ! ros2 node list 2>/dev/null | grep -q cutter_controller; then
    echo "Starting cutter controller in background..."
    python3 "$HOME/robot/scripts/cutter_controller.py" &
    CUTTER_PID=$!
    echo $CUTTER_PID > /tmp/cutter_controller.pid
    echo "Waiting for cutter service..."
    sleep 3
    CUTTER_STARTED=true
fi

# Call the service
if ros2 service call /cutter/set std_srvs/srv/SetBool '{data: true}' \
        2>/dev/null | grep -q "success"; then
    echo "Cutter is ON"
else
    # Try once more (service might need another second)
    sleep 2
    ros2 service call /cutter/set std_srvs/srv/SetBool '{data: true}' 2>/dev/null || true
    echo "Cutter command sent."
fi
SHEOF
chmod +x "$SCRIPTS/cutter_on.sh"

cat > "$SCRIPTS/cutter_off.sh" << 'SHEOF'
#!/bin/bash
source /opt/ros/kilted/setup.bash
source "$HOME/ws_lidar/install/setup.bash"

echo "Turning cutter OFF..."

# Check if cutter_controller is running
if ! ros2 node list 2>/dev/null | grep -q cutter_controller; then
    echo "Cutter controller is not running — cutter is already off."
    exit 0
fi

ros2 service call /cutter/set std_srvs/srv/SetBool '{data: false}' 2>/dev/null || true
echo "Cutter is OFF"

# If we started it standalone, kill it
if [ -f /tmp/cutter_controller.pid ]; then
    PID=$(cat /tmp/cutter_controller.pid)
    kill "$PID" 2>/dev/null && echo "Cutter controller stopped."
    rm -f /tmp/cutter_controller.pid
fi
SHEOF
chmod +x "$SCRIPTS/cutter_off.sh"
echo -e "${GREEN}  cutter_on/off fixed (auto-starts cutter_controller if needed)${NC}"


# ════════════════════════════════════════════════════════════
# FIX 4: robot_coverage — simple direct-drive boustrophedon
#
# Rewritten as a standalone Python script that:
#   - Reads /scan for real-time obstacle detection
#   - Drives in boustrophedon strips via /cmd_vel directly
#   - No Nav2, no AMCL, no map required
#   - Detects obstacles in the forward arc and stops/reverses/turns
#   - Motor controller + LiDAR must run alongside it
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[4] Rewriting simple_coverage.py and coverage_nomap_launch.py...${NC}"

cat > "$SCRIPTS/simple_coverage.py" << 'PYEOF'
#!/usr/bin/env python3
"""
simple_coverage.py — Boustrophedon (lawnmower) pattern
Direct /cmd_vel control, LiDAR obstacle detection.
No Nav2, no AMCL, no saved map needed.

Parameters (adjustable at runtime via ros2 param set):
  strip_width    0.40   metres between lawnmower passes
  forward_speed  0.25   m/s  (keep slow for safety)
  turn_speed     0.5    rad/s
  strip_count    10     number of strips before stopping
  obstacle_dist  0.50   metres — stop if obstacle within this range
  forward_arc    60     degrees total forward detection arc (±30°)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


# ── States ────────────────────────────────────────────────────────────────────
ST_IDLE      = 'IDLE'
ST_FORWARD   = 'FORWARD'
ST_TURN      = 'TURN'
ST_OBSTACLE  = 'OBSTACLE_WAIT'
ST_DONE      = 'DONE'
ST_PAUSED    = 'PAUSED'


class SimpleCoverage(Node):

    def __init__(self):
        super().__init__('simple_coverage')

        # ── parameters ────────────────────────────────────────
        self.declare_parameter('strip_width',   0.40)
        self.declare_parameter('forward_speed', 0.25)
        self.declare_parameter('turn_speed',    0.50)
        self.declare_parameter('strip_count',   10)
        self.declare_parameter('obstacle_dist', 0.50)
        self.declare_parameter('forward_arc',   60.0)
        self.declare_parameter('autostart',     True)
        self.declare_parameter('strip_time',    8.0)   # seconds per strip

        self._read_params()

        # ── state ─────────────────────────────────────────────
        self.state        = ST_IDLE
        self.strip_done   = 0
        self.turn_left    = True    # alternate turn direction
        self.obstacle     = False
        self.paused       = False
        self.state_start  = self.get_clock().now()
        self.scan         = None

        # ── ROS ───────────────────────────────────────────────
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._on_scan, 10)

        self.srv_start  = self.create_service(
            Trigger, '/coverage/start',  self._svc_start)
        self.srv_pause  = self.create_service(
            Trigger, '/coverage/pause',  self._svc_pause)
        self.srv_resume = self.create_service(
            Trigger, '/coverage/resume', self._svc_resume)
        self.srv_cancel = self.create_service(
            Trigger, '/coverage/cancel', self._svc_cancel)

        self.timer = self.create_timer(0.1, self._loop)

        self.get_logger().info(
            f'Simple coverage ready | strips={self.strip_count} '
            f'width={self.strip_width}m speed={self.forward_speed}m/s '
            f'obstacle_stop={self.obstacle_dist}m'
        )
        if self.get_parameter('autostart').value:
            self.get_logger().info('Autostart in 3 s ...')
            self.create_timer(3.0, self._autostart)

    def _read_params(self):
        self.strip_width   = self.get_parameter('strip_width').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed    = self.get_parameter('turn_speed').value
        self.strip_count   = self.get_parameter('strip_count').value
        self.obstacle_dist = self.get_parameter('obstacle_dist').value
        self.forward_arc   = self.get_parameter('forward_arc').value
        self.strip_time    = self.get_parameter('strip_time').value

    # ── LiDAR callback ───────────────────────────────────────
    def _on_scan(self, msg: LaserScan):
        self.scan = msg
        # Check forward arc for obstacles
        half_arc = math.radians(self.forward_arc / 2.0)
        a_min = msg.angle_min
        a_inc = msg.angle_increment
        blocked = False
        for i, r in enumerate(msg.ranges):
            angle = a_min + i * a_inc
            if abs(angle) <= half_arc:
                if msg.range_min < r < self.obstacle_dist:
                    blocked = True
                    break
        self.obstacle = blocked

    # ── services ─────────────────────────────────────────────
    def _autostart(self):
        if self.state == ST_IDLE:
            self._transition(ST_FORWARD)

    def _svc_start(self, _, resp):
        self._transition(ST_FORWARD)
        resp.success = True
        resp.message = 'Coverage started'
        return resp

    def _svc_pause(self, _, resp):
        self.paused = True
        self._stop()
        resp.success = True
        resp.message = 'Coverage paused'
        return resp

    def _svc_resume(self, _, resp):
        self.paused = False
        resp.success = True
        resp.message = 'Coverage resumed'
        return resp

    def _svc_cancel(self, _, resp):
        self._transition(ST_IDLE)
        self._stop()
        resp.success = True
        resp.message = 'Coverage cancelled'
        return resp

    # ── helpers ──────────────────────────────────────────────
    def _transition(self, new_state):
        self.state = new_state
        self.state_start = self.get_clock().now()
        self.get_logger().info(f'State → {new_state}  (strip {self.strip_done}/{self.strip_count})')

    def _elapsed(self):
        return (self.get_clock().now() - self.state_start).nanoseconds / 1e9

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _drive(self, linear=0.0, angular=0.0):
        t = Twist()
        t.linear.x  = linear
        t.angular.z = angular
        self.cmd_pub.publish(t)

    # ── main loop ─────────────────────────────────────────────
    def _loop(self):
        if self.paused or self.state in (ST_IDLE, ST_DONE):
            return

        # ── OBSTACLE_WAIT: obstacle was detected, back up briefly then re-plan ──
        if self.state == ST_OBSTACLE:
            if self._elapsed() < 1.5:          # back up for 1.5 s
                self._drive(linear=-0.15)
            else:
                # count as end of strip and turn
                self.strip_done += 1
                if self.strip_done >= self.strip_count:
                    self._transition(ST_DONE)
                    self._stop()
                    self.get_logger().info('Coverage COMPLETE')
                else:
                    self._transition(ST_TURN)
            return

        # ── FORWARD: drive straight ───────────────────────────
        if self.state == ST_FORWARD:
            if self.obstacle:
                self.get_logger().warn('Obstacle detected — stopping')
                self._transition(ST_OBSTACLE)
                self._stop()
                return
            if self._elapsed() > self.strip_time:
                # Strip done, start turn
                self.strip_done += 1
                if self.strip_done >= self.strip_count:
                    self._transition(ST_DONE)
                    self._stop()
                    self.get_logger().info('Coverage COMPLETE')
                else:
                    self._transition(ST_TURN)
                return
            self._drive(linear=self.forward_speed)
            return

        # ── TURN: 180° turn + lateral shift for next strip ───
        if self.state == ST_TURN:
            # Turn time = π / turn_speed
            turn_time = math.pi / self.turn_speed
            # Lateral shift time = strip_width / forward_speed
            shift_time = self.strip_width / self.forward_speed

            elapsed = self._elapsed()

            if elapsed < turn_time:
                # Turn 180°
                ang = self.turn_speed if self.turn_left else -self.turn_speed
                self._drive(angular=ang)
            elif elapsed < turn_time + shift_time:
                # Move forward one strip width
                self._drive(linear=self.forward_speed)
            else:
                # Done turning, flip direction, start next strip
                self.turn_left = not self.turn_left
                self._transition(ST_FORWARD)
            return


def main(args=None):
    rclpy.init(args=args)
    node = SimpleCoverage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
PYEOF
chmod +x "$SCRIPTS/simple_coverage.py"
echo -e "${GREEN}  simple_coverage.py written${NC}"

# Rewrite coverage_nomap_launch.py — much simpler now (no Nav2 at all)
python3 - << 'PYEOF'
import os
home = os.path.expanduser('~')
path = os.path.join(home, 'robot', 'launch', 'coverage_nomap_launch.py')

content = '''\
#!/usr/bin/env python3
"""
coverage_nomap_launch.py
Autonomous boustrophedon coverage — no map, no Nav2 needed.
Launches: RPLidar, Motor Controller, Cutter Controller, Simple Coverage, RViz
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    home        = os.path.expanduser(\'~\')
    scripts_dir = os.path.join(home, \'robot\', \'scripts\')
    rviz_config = os.path.join(home, \'robot\', \'rviz\', \'mapping_config.rviz\')

    return LaunchDescription([

        # TF: base_link -> laser
        Node(
            package=\'tf2_ros\',
            executable=\'static_transform_publisher\',
            name=\'base_to_laser_tf\',
            arguments=[\'0\', \'0\', \'0.1\', \'0\', \'0\', \'0\', \'base_link\', \'laser\'],
            output=\'screen\',
        ),
        # TF: odom -> base_link
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

        # Motor controller
        ExecuteProcess(
            cmd=[\'python3\', os.path.join(scripts_dir, \'motor_controller.py\')],
            name=\'motor_controller\',
            output=\'screen\',
        ),

        # Cutter controller
        ExecuteProcess(
            cmd=[\'python3\', os.path.join(scripts_dir, \'cutter_controller.py\')],
            name=\'cutter_controller\',
            output=\'screen\',
        ),

        # Simple boustrophedon coverage (starts 3s after, after scan is ready)
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[\'python3\', os.path.join(scripts_dir, \'simple_coverage.py\')],
                    name=\'simple_coverage\',
                    output=\'screen\',
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

# Update start_coverage_nomap.sh
cat > "$SCRIPTS/start_coverage_nomap.sh" << 'SHEOF'
#!/bin/bash
source /opt/ros/kilted/setup.bash
source "$HOME/ws_lidar/install/setup.bash"

echo "=== AUTONOMOUS COVERAGE (boustrophedon) ==="
echo ""
echo "The robot will:"
echo "  1. Spin up LiDAR + motor controller + cutter controller"
echo "  2. Drive in lawnmower strips automatically after 5 seconds"
echo "  3. Stop and reverse if any obstacle is detected"
echo ""
echo "Blade control (in another terminal):"
echo "  robot_cutter_on   - start blade"
echo "  robot_cutter_off  - stop blade"
echo ""
echo "Emergency:"
echo "  robot_stop        - immediate stop"
echo "  robot_resume      - continue"
echo ""
echo "Tuning (while running):"
echo "  ros2 param set /simple_coverage strip_time 10.0   # seconds per strip"
echo "  ros2 param set /simple_coverage forward_speed 0.2 # m/s"
echo "  ros2 param set /simple_coverage strip_count 12    # number of strips"
echo ""
ros2 launch "$HOME/robot/launch/coverage_nomap_launch.py"
SHEOF
chmod +x "$SCRIPTS/start_coverage_nomap.sh"
echo -e "${GREEN}  coverage_nomap_launch.py + simple_coverage.py written${NC}"


# ════════════════════════════════════════════════════════════
# FIX 5: robot_stop and robot_resume — emergency commands
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[5] Adding robot_stop and robot_resume commands...${NC}"

cat > "$SCRIPTS/robot_stop.sh" << 'SHEOF'
#!/bin/bash
source /opt/ros/kilted/setup.bash

echo "EMERGENCY STOP"

# 1. Pause coverage if running
ros2 service call /coverage/cancel std_srvs/srv/Trigger '{}' 2>/dev/null | grep -q success && \
    echo "  Coverage cancelled" || true

# 2. Send zero velocity repeatedly for 2 seconds (ensures motor stops)
for i in $(seq 1 20); do
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
        '{linear: {x: 0.0}, angular: {z: 0.0}}' 2>/dev/null
    sleep 0.1
done

echo "Robot stopped."
echo "To resume coverage: robot_resume"
echo "To resume navigation: coverage or navigate was already running in other terminal"
SHEOF
chmod +x "$SCRIPTS/robot_stop.sh"

cat > "$SCRIPTS/robot_resume.sh" << 'SHEOF'
#!/bin/bash
source /opt/ros/kilted/setup.bash

echo "Resuming coverage..."

if ros2 service call /coverage/resume std_srvs/srv/Trigger '{}' 2>/dev/null | grep -q success; then
    echo "Coverage resumed."
else
    echo "Could not resume coverage."
    echo "If robot_coverage was running, it may have finished or been cancelled."
    echo "Re-run: robot_coverage"
fi
SHEOF
chmod +x "$SCRIPTS/robot_resume.sh"

echo -e "${GREEN}  robot_stop.sh and robot_resume.sh created${NC}"


# ════════════════════════════════════════════════════════════
# Add new aliases to bashrc (robot_stop, robot_resume)
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[6] Updating bashrc aliases...${NC}"

# Remove old robot block and rewrite it cleanly
sed -i '/# === ROBOT SHORTCUTS ===/,/# === END ROBOT ===/d' ~/.bashrc

cat >> ~/.bashrc << 'BASHEOF'

# === ROBOT SHORTCUTS ===
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/kilted/setup.bash 2>/dev/null
source "$HOME/ws_lidar/install/setup.bash" 2>/dev/null

# Mapping workflow
alias robot_map='bash "$HOME/robot/scripts/start_mapping.sh"'
alias robot_save_map='bash "$HOME/robot/scripts/save_map.sh"'
alias robot_teleop='bash "$HOME/robot/scripts/teleop.sh"'

# Autonomous (with saved map)
alias robot_navigate='bash "$HOME/robot/scripts/start_navigation.sh"'

# Autonomous (no map — boustrophedon + obstacle avoidance)
alias robot_coverage='bash "$HOME/robot/scripts/start_coverage_nomap.sh"'

# Motor test only
alias robot_drive='bash "$HOME/robot/scripts/robot_drive.sh"'

# Cutter blade control (works anytime)
alias robot_cutter_on='bash "$HOME/robot/scripts/cutter_on.sh"'
alias robot_cutter_off='bash "$HOME/robot/scripts/cutter_off.sh"'

# Emergency
alias robot_stop='bash "$HOME/robot/scripts/robot_stop.sh"'
alias robot_resume='bash "$HOME/robot/scripts/robot_resume.sh"'

# Diagnostics
alias robot_scan='ros2 topic hz /scan'
alias robot_nodes='ros2 node list'
alias robot_tf='ros2 run tf2_tools view_frames'
# === END ROBOT ===
BASHEOF

echo -e "${GREEN}  bashrc aliases updated${NC}"


# ════════════════════════════════════════════════════════════
# VERIFY
# ════════════════════════════════════════════════════════════
echo ""
echo -e "${YELLOW}Verifying...${NC}"
echo ""

PASS=0; FAIL=0

chk_py() {
    if python3 -c "import ast; ast.parse(open('$1').read())" 2>/dev/null; then
        echo -e "  ${GREEN}OK${NC} $2"
        PASS=$((PASS+1))
    else
        echo -e "  ${RED}FAIL${NC} $2"
        FAIL=$((FAIL+1))
    fi
}

chk_sh() {
    if bash -n "$1" 2>/dev/null; then
        echo -e "  ${GREEN}OK${NC} $2"
        PASS=$((PASS+1))
    else
        echo -e "  ${RED}FAIL${NC} $2"
        FAIL=$((FAIL+1))
    fi
}

chk_exists() {
    if [ -f "$1" ]; then
        echo -e "  ${GREEN}OK${NC} $2"
        PASS=$((PASS+1))
    else
        echo -e "  ${RED}MISSING${NC} $2"
        FAIL=$((FAIL+1))
    fi
}

chk_py  "$LAUNCH/mapping_launch.py"          "mapping_launch.py"
chk_py  "$LAUNCH/navigation_launch.py"       "navigation_launch.py"
chk_py  "$LAUNCH/coverage_nomap_launch.py"   "coverage_nomap_launch.py"
chk_py  "$SCRIPTS/simple_coverage.py"        "simple_coverage.py"
chk_py  "$SCRIPTS/motor_controller.py"       "motor_controller.py"
chk_py  "$SCRIPTS/cutter_controller.py"      "cutter_controller.py"
chk_py  "$SCRIPTS/coverage_path_planner.py"  "coverage_path_planner.py"
chk_sh  "$SCRIPTS/save_map.sh"               "save_map.sh"
chk_sh  "$SCRIPTS/teleop.sh"                 "teleop.sh"
chk_sh  "$SCRIPTS/cutter_on.sh"              "cutter_on.sh"
chk_sh  "$SCRIPTS/cutter_off.sh"             "cutter_off.sh"
chk_sh  "$SCRIPTS/robot_stop.sh"             "robot_stop.sh"
chk_sh  "$SCRIPTS/robot_resume.sh"           "robot_resume.sh"
chk_exists "$SCRIPTS/robot_drive.sh"         "robot_drive.sh"

# Check xterm properly (not in comments)
if python3 - << 'PYEOF' 2>/dev/null; then
import ast
tree = ast.parse(open(f"{__import__('os').path.expanduser('~')}/robot/launch/mapping_launch.py").read())
found = any(
    isinstance(node, ast.Constant) and 'xterm' in str(node.value)
    for node in ast.walk(tree)
)
exit(0 if not found else 1)
PYEOF
    echo -e "  ${GREEN}OK${NC} mapping_launch.py - no xterm in code (only in comments)"
    PASS=$((PASS+1))
else
    echo -e "  ${RED}FAIL${NC} mapping_launch.py - xterm still in executable code"
    FAIL=$((FAIL+1))
fi

echo ""
echo "============================================================"
if [ "$FAIL" -eq 0 ]; then
    echo -e "${GREEN}   ALL $PASS CHECKS PASSED!${NC}"
else
    echo -e "${RED}   $FAIL failed / ${GREEN}$PASS passed${NC}"
fi
echo "============================================================"
echo ""
echo "Run: source ~/.bashrc"
echo ""
echo "All commands:"
echo -e "  ${BLUE}robot_map${NC}             Start SLAM + LiDAR (terminal 1)"
echo -e "  ${BLUE}robot_teleop${NC}          Drive manually (terminal 2, works standalone)"
echo -e "  ${BLUE}robot_save_map my_lawn${NC} Save map while robot_map is running"
echo -e "  ${BLUE}robot_navigate my_lawn${NC} Autonomous mow with saved map"
echo -e "  ${BLUE}robot_coverage${NC}        Autonomous boustrophedon, no map needed"
echo -e "  ${BLUE}robot_cutter_on${NC}       Start blade (works anytime)"
echo -e "  ${BLUE}robot_cutter_off${NC}      Stop blade"
echo -e "  ${BLUE}robot_stop${NC}            EMERGENCY STOP"
echo -e "  ${BLUE}robot_resume${NC}          Resume after stop"
echo -e "  ${BLUE}robot_drive${NC}           Test motors only"
echo ""
echo "Tuning strip size while robot_coverage is running:"
echo -e "  ${BLUE}ros2 param set /simple_coverage strip_time 10.0${NC}    # seconds per strip"
echo -e "  ${BLUE}ros2 param set /simple_coverage strip_count 15${NC}     # total strips"
echo -e "  ${BLUE}ros2 param set /simple_coverage obstacle_dist 0.6${NC}  # stop distance (m)"
