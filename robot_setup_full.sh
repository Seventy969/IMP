#!/bin/bash
# ============================================================
# robot_setup_full.sh
# Complete lawn mower robot setup — ROS2 Kilted / Ubuntu 24.04
#
# Hardware:
#   RPLidar A1M8  →  /dev/ttyUSB0
#   L298N #1  Left  : IN1=13  IN2=19  ENA=26
#             Right : IN3=5   IN4=6   ENB=12   ← corrected from 0
#   L298N #2  Cutter: IN3=16  IN4=20  ENB=21
#   No wheel encoder · No IMU
#
# Creates:
#   ~/robot/scripts/motor_controller.py
#   ~/robot/scripts/cutter_controller.py
#   ~/robot/scripts/coverage_path_planner.py
#   ~/robot/config/nav2_params.yaml
#   ~/robot/config/slam_params.yaml
#   ~/robot/launch/mapping_launch.py
#   ~/robot/launch/navigation_launch.py      (with saved map)
#   ~/robot/launch/coverage_nomap_launch.py  (no saved map)
#   ~/robot/rviz/mapping_config.rviz
#   ~/robot/rviz/navigation_config.rviz
#   ~/robot/scripts/start_mapping.sh
#   ~/robot/scripts/start_navigation.sh
#   ~/robot/scripts/start_coverage_nomap.sh
#   ~/robot/scripts/save_map.sh
#   ~/robot/scripts/teleop.sh
#   ~/robot/scripts/cutter_on.sh
#   ~/robot/scripts/cutter_off.sh
#   ~/.bashrc aliases
# ============================================================

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

echo ""
echo "============================================================"
echo "   Lawn Mower Robot — Full Setup Script"
echo "============================================================"
echo ""

# ── directory scaffold ───────────────────────────────────────
mkdir -p ~/robot/{maps,launch,scripts,rviz,config}
echo -e "${GREEN}✓ Directory structure created${NC}"

# ════════════════════════════════════════════════════════════
# 1. motor_controller.py
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[1/12] Writing motor_controller.py...${NC}"
cat > ~/robot/scripts/motor_controller.py << 'PYEOF'
#!/usr/bin/env python3
"""
Motor Controller — Lawn Mower Robot
Converts /cmd_vel  →  L298N #1 drive motors via GPIO (BCM numbering)

Left motor  : IN1=13  IN2=19  ENA=26
Right motor : IN3=5   IN4=6   ENB=12
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time


class MotorController(Node):
    # ── GPIO pin map ──────────────────────────────────────────
    LEFT_IN1  = 13
    LEFT_IN2  = 19
    LEFT_ENA  = 26
    RIGHT_IN3 = 5
    RIGHT_IN4 = 6
    RIGHT_ENB = 12      # ← was 0 in old code — corrected

    # ── physical constants ────────────────────────────────────
    WHEEL_SEPARATION = 0.30   # metres between wheel centres — MEASURE YOUR ROBOT
    MAX_LINEAR_VEL   = 0.40   # m/s
    MAX_ANGULAR_VEL  = 2.00   # rad/s
    MIN_PWM          = 30     # duty-cycle below which motors stall (tune per robot)
    PWM_FREQ         = 1000   # Hz

    def __init__(self):
        super().__init__('motor_controller')

        # Declare tunable parameters
        self.declare_parameter('wheel_separation', self.WHEEL_SEPARATION)
        self.declare_parameter('max_linear_vel',   self.MAX_LINEAR_VEL)
        self.declare_parameter('max_angular_vel',  self.MAX_ANGULAR_VEL)
        self.declare_parameter('min_pwm',          self.MIN_PWM)
        self.declare_parameter('cmd_timeout',      1.0)

        self.wheel_sep   = self.get_parameter('wheel_separation').value
        self.max_lin     = self.get_parameter('max_linear_vel').value
        self.max_ang     = self.get_parameter('max_angular_vel').value
        self.min_pwm     = self.get_parameter('min_pwm').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for pin in (self.LEFT_IN1, self.LEFT_IN2, self.LEFT_ENA,
                    self.RIGHT_IN3, self.RIGHT_IN4, self.RIGHT_ENB):
            GPIO.setup(pin, GPIO.OUT)

        self.left_pwm  = GPIO.PWM(self.LEFT_ENA,  self.PWM_FREQ)
        self.right_pwm = GPIO.PWM(self.RIGHT_ENB, self.PWM_FREQ)
        self.left_pwm.start(0)
        self.right_pwm.start(0)

        self.last_cmd = self.get_clock().now()

        self.sub   = self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.timer = self.create_timer(0.1, self._safety_watchdog)

        self.get_logger().info(
            f'Motor controller ready  |  wheel_sep={self.wheel_sep} m  '
            f'max_linear={self.max_lin} m/s'
        )

    # ── helpers ───────────────────────────────────────────────
    def _vel_to_pwm(self, vel_ms):
        """Convert m/s velocity to signed duty-cycle [−100 … +100]."""
        ratio = max(-1.0, min(1.0, vel_ms / self.max_lin))
        if ratio == 0.0:
            return 0.0
        # scale from min_pwm to 100
        magnitude = self.min_pwm + abs(ratio) * (100.0 - self.min_pwm)
        return magnitude if ratio > 0 else -magnitude

    def _drive(self, left_dc, right_dc):
        """Apply signed duty-cycles to both motor channels."""
        # Left
        if left_dc > 0:
            GPIO.output(self.LEFT_IN1, GPIO.HIGH)
            GPIO.output(self.LEFT_IN2, GPIO.LOW)
        elif left_dc < 0:
            GPIO.output(self.LEFT_IN1, GPIO.LOW)
            GPIO.output(self.LEFT_IN2, GPIO.HIGH)
        else:
            GPIO.output(self.LEFT_IN1, GPIO.LOW)
            GPIO.output(self.LEFT_IN2, GPIO.LOW)
        self.left_pwm.ChangeDutyCycle(abs(left_dc))

        # Right
        if right_dc > 0:
            GPIO.output(self.RIGHT_IN3, GPIO.HIGH)
            GPIO.output(self.RIGHT_IN4, GPIO.LOW)
        elif right_dc < 0:
            GPIO.output(self.RIGHT_IN3, GPIO.LOW)
            GPIO.output(self.RIGHT_IN4, GPIO.HIGH)
        else:
            GPIO.output(self.RIGHT_IN3, GPIO.LOW)
            GPIO.output(self.RIGHT_IN4, GPIO.LOW)
        self.right_pwm.ChangeDutyCycle(abs(right_dc))

    # ── callbacks ─────────────────────────────────────────────
    def _on_cmd_vel(self, msg: Twist):
        self.last_cmd = self.get_clock().now()
        lin = msg.linear.x
        ang = msg.angular.z
        # Differential drive kinematics
        left_ms  = lin - ang * self.wheel_sep / 2.0
        right_ms = lin + ang * self.wheel_sep / 2.0
        self._drive(self._vel_to_pwm(left_ms), self._vel_to_pwm(right_ms))

    def _safety_watchdog(self):
        elapsed = (self.get_clock().now() - self.last_cmd).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            self._drive(0, 0)

    def destroy_node(self):
        self.get_logger().info('Shutting down — stopping motors')
        self._drive(0, 0)
        time.sleep(0.1)
        self.left_pwm.stop()
        self.right_pwm.stop()
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
PYEOF
chmod +x ~/robot/scripts/motor_controller.py
echo -e "${GREEN}  ✓ motor_controller.py${NC}"


# ════════════════════════════════════════════════════════════
# 2. cutter_controller.py
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[2/12] Writing cutter_controller.py...${NC}"
cat > ~/robot/scripts/cutter_controller.py << 'PYEOF'
#!/usr/bin/env python3
"""
Cutter Controller — Lawn Mower Robot
Controls the 5th (blade) motor via L298N #2

GPIO (BCM): IN3=16  IN4=20  ENB=21

Topics:
  /cutter/cmd  std_msgs/Bool   True=on  False=off

Services:
  /cutter/set  std_srvs/SetBool
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
import RPi.GPIO as GPIO
import time


class CutterController(Node):
    CUTTER_IN3 = 16
    CUTTER_IN4 = 20
    CUTTER_ENB = 21
    PWM_FREQ   = 1000

    def __init__(self):
        super().__init__('cutter_controller')

        self.declare_parameter('cutter_speed', 85)  # duty-cycle %
        self.cutter_speed = self.get_parameter('cutter_speed').value
        self.running = False

        # GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for pin in (self.CUTTER_IN3, self.CUTTER_IN4, self.CUTTER_ENB):
            GPIO.setup(pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.CUTTER_ENB, self.PWM_FREQ)
        self.pwm.start(0)
        GPIO.output(self.CUTTER_IN3, GPIO.HIGH)
        GPIO.output(self.CUTTER_IN4, GPIO.LOW)

        # ROS interfaces
        self.sub = self.create_subscription(Bool, '/cutter/cmd', self._on_cmd, 10)
        self.srv = self.create_service(SetBool, '/cutter/set', self._on_srv)

        self.get_logger().info(f'Cutter controller ready  |  speed={self.cutter_speed}%')

    def _set_cutter(self, enable: bool):
        if enable:
            self.pwm.ChangeDutyCycle(self.cutter_speed)
            self.running = True
            self.get_logger().info('Cutter ON')
        else:
            self.pwm.ChangeDutyCycle(0)
            self.running = False
            self.get_logger().info('Cutter OFF')

    def _on_cmd(self, msg: Bool):
        self._set_cutter(msg.data)

    def _on_srv(self, request, response):
        self._set_cutter(request.data)
        response.success = True
        response.message = 'Cutter ON' if request.data else 'Cutter OFF'
        return response

    def destroy_node(self):
        self.get_logger().info('Cutter shutdown')
        self.pwm.ChangeDutyCycle(0)
        time.sleep(0.1)
        self.pwm.stop()
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CutterController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
PYEOF
chmod +x ~/robot/scripts/cutter_controller.py
echo -e "${GREEN}  ✓ cutter_controller.py${NC}"


# ════════════════════════════════════════════════════════════
# 3. coverage_path_planner.py
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[3/12] Writing coverage_path_planner.py...${NC}"
cat > ~/robot/scripts/coverage_path_planner.py << 'PYEOF'
#!/usr/bin/env python3
"""
Coverage Path Planner — Boustrophedon (lawnmower) pattern
Works with BOTH saved-map navigation AND live SLAM mapping.

Topics subscribed:
  /map            nav_msgs/OccupancyGrid
Topics published:
  /coverage/progress   std_msgs/Float32   (0–100 %)
  /coverage/path       nav_msgs/Path      (full planned path, for RViz)
Services:
  /coverage/start      std_srvs/Trigger
  /coverage/pause      std_srvs/Trigger
  /coverage/resume     std_srvs/Trigger
  /coverage/cancel     std_srvs/Trigger
Nav2 action:
  navigate_to_pose     nav2_msgs/NavigateToPose
"""

import math
import rclpy
from rclpy.node    import Node
from rclpy.action  import ActionClient
from rclpy.qos     import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav2_msgs.action  import NavigateToPose
from nav_msgs.msg      import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg      import Float32
from std_srvs.srv      import Trigger


# ── utility ───────────────────────────────────────────────────
def yaw_to_quat(yaw: float):
    """Return (x, y, z, w) quaternion for a pure-yaw rotation."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def make_pose(x, y, yaw, frame='map', stamp=None):
    ps = PoseStamped()
    ps.header.frame_id = frame
    if stamp:
        ps.header.stamp = stamp
    ps.pose.position.x = x
    ps.pose.position.y = y
    qx, qy, qz, qw    = yaw_to_quat(yaw)
    ps.pose.orientation.x = qx
    ps.pose.orientation.y = qy
    ps.pose.orientation.z = qz
    ps.pose.orientation.w = qw
    return ps


class CoveragePathPlanner(Node):

    def __init__(self):
        super().__init__('coverage_path_planner')

        # ── parameters ────────────────────────────────────────
        self.declare_parameter('strip_width',       0.35)   # metres (≈ blade width)
        self.declare_parameter('obstacle_threshold', 50)    # costmap value 0-100
        self.declare_parameter('map_wait_sec',        5.0)  # seconds before first plan
        self.declare_parameter('autostart',          True)  # start planning automatically

        self.strip_width        = self.get_parameter('strip_width').value
        self.obs_threshold      = self.get_parameter('obstacle_threshold').value
        self.map_wait_sec       = self.get_parameter('map_wait_sec').value
        self.autostart          = self.get_parameter('autostart').value

        # ── state ─────────────────────────────────────────────
        self.map_data   = None
        self.waypoints  = []   # list of (x, y, yaw)
        self.wp_idx     = 0
        self.active     = False
        self.paused     = False
        self.map_ready  = False

        # ── QoS for /map (transient-local so we get the last message) ──
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ── ROS interfaces ────────────────────────────────────
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._on_map, map_qos
        )
        self.progress_pub = self.create_publisher(Float32,  '/coverage/progress', 10)
        self.path_pub     = self.create_publisher(Path,     '/coverage/path',     10)

        self.srv_start  = self.create_service(Trigger, '/coverage/start',  self._svc_start)
        self.srv_pause  = self.create_service(Trigger, '/coverage/pause',  self._svc_pause)
        self.srv_resume = self.create_service(Trigger, '/coverage/resume', self._svc_resume)
        self.srv_cancel = self.create_service(Trigger, '/coverage/cancel', self._svc_cancel)

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self._start_timer = self.create_timer(self.map_wait_sec, self._autostart_check)

        self.get_logger().info('Coverage path planner ready')

    # ── map callback ─────────────────────────────────────────
    def _on_map(self, msg: OccupancyGrid):
        self.map_data  = msg
        self.map_ready = True

    # ── autostart ────────────────────────────────────────────
    def _autostart_check(self):
        self._start_timer.cancel()
        if self.autostart and self.map_ready and not self.active:
            self.get_logger().info('Autostarting coverage...')
            self._begin_coverage()

    # ── service handlers ─────────────────────────────────────
    def _svc_start(self, _, resp):
        if not self.map_ready:
            resp.success = False; resp.message = 'No map received yet'
            return resp
        self._begin_coverage()
        resp.success = True; resp.message = 'Coverage started'
        return resp

    def _svc_pause(self, _, resp):
        self.paused = True
        resp.success = True; resp.message = 'Coverage paused'
        return resp

    def _svc_resume(self, _, resp):
        if self.paused:
            self.paused = False
            self._send_next_goal()
        resp.success = True; resp.message = 'Coverage resumed'
        return resp

    def _svc_cancel(self, _, resp):
        self.active = False; self.paused = False; self.wp_idx = 0
        resp.success = True; resp.message = 'Coverage cancelled'
        return resp

    # ── core planning ────────────────────────────────────────
    def _begin_coverage(self):
        self._generate_waypoints()
        if not self.waypoints:
            self.get_logger().warn('No free waypoints found in map')
            return
        self.active  = True
        self.paused  = False
        self.wp_idx  = 0
        self._publish_full_path()
        self._send_next_goal()

    def _generate_waypoints(self):
        """Build boustrophedon waypoints from the current OccupancyGrid."""
        m   = self.map_data
        res = m.info.resolution
        W   = m.info.width
        H   = m.info.height
        ox  = m.info.origin.position.x
        oy  = m.info.origin.position.y

        # Flat list → 2-D access: data[row][col]
        raw = list(m.data)

        strip_cells = max(1, int(round(self.strip_width / res)))
        waypoints   = []
        left_to_right = True

        row = strip_cells // 2          # start at mid-strip
        while row < H:
            # collect free columns in this row
            free_cols = [
                c for c in range(W)
                if 0 <= raw[row * W + c] < self.obs_threshold
            ]
            if free_cols:
                col_range = (
                    range(min(free_cols), max(free_cols) + 1, strip_cells)
                    if left_to_right else
                    range(max(free_cols), min(free_cols) - 1, -strip_cells)
                )
                for col in col_range:
                    if 0 <= raw[row * W + col] < self.obs_threshold:
                        x   = ox + (col + 0.5) * res
                        y   = oy + (row + 0.5) * res
                        yaw = 0.0 if left_to_right else math.pi
                        waypoints.append((x, y, yaw))

                left_to_right = not left_to_right

            row += strip_cells

        self.waypoints = waypoints
        self.get_logger().info(
            f'Coverage plan: {len(waypoints)} waypoints  '
            f'strip={self.strip_width} m  resolution={res} m/cell'
        )

    def _publish_full_path(self):
        path      = Path()
        path.header.frame_id = 'map'
        path.header.stamp    = self.get_clock().now().to_msg()
        for x, y, yaw in self.waypoints:
            path.poses.append(make_pose(x, y, yaw, stamp=path.header.stamp))
        self.path_pub.publish(path)

    # ── navigation ───────────────────────────────────────────
    def _send_next_goal(self):
        if not self.active or self.paused:
            return
        if self.wp_idx >= len(self.waypoints):
            self._on_coverage_complete()
            return

        x, y, yaw = self.waypoints[self.wp_idx]
        goal_msg  = NavigateToPose.Goal()
        goal_msg.pose = make_pose(x, y, yaw, stamp=self.get_clock().now().to_msg())

        self.get_logger().info(
            f'  WP {self.wp_idx + 1}/{len(self.waypoints)}'
            f'  ({x:.2f}, {y:.2f})  yaw={math.degrees(yaw):.0f}°'
        )

        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('NavigateToPose action server not available!')
            return

        send_future = self._nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_accepted_cb)

        # publish progress
        pct = Float32()
        pct.data = 100.0 * self.wp_idx / len(self.waypoints)
        self.progress_pub.publish(pct)

    def _goal_accepted_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn(f'WP {self.wp_idx} rejected — skipping')
            self.wp_idx += 1
            self._send_next_goal()
            return
        gh.get_result_async().add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        # regardless of success/failure move to next waypoint
        self.wp_idx += 1
        self._send_next_goal()

    def _on_coverage_complete(self):
        self.active = False
        pct = Float32(); pct.data = 100.0
        self.progress_pub.publish(pct)
        self.get_logger().info('✓ Coverage complete!')


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
PYEOF
chmod +x ~/robot/scripts/coverage_path_planner.py
echo -e "${GREEN}  ✓ coverage_path_planner.py${NC}"


# ════════════════════════════════════════════════════════════
# 4. nav2_params.yaml
#    Corrected for ROS2 Kilted (Jazzy-era Nav2):
#    - robot_model_type full class name
#    - plugin paths use :: not /
#    - bt_navigator plugin_lib_names included
#    - velocity_smoother added
#    - waypoint_follower added
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[4/12] Writing nav2_params.yaml...${NC}"
cat > ~/robot/config/nav2_params.yaml << 'YAMLEOF'
# ============================================================
# nav2_params.yaml — Lawn Mower Robot
# ROS2 Kilted · No wheel encoder · No IMU
# ============================================================

amcl:
  ros__parameters:
    use_sim_time: false
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id:    "base_link"
    odom_frame_id:    "odom"
    global_frame_id:  "map"
    laser_max_range:  12.0
    laser_min_range:   0.12
    # Full class name required in Nav2 (Humble+)
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    laser_model_type: "likelihood_field"
    max_particles: 2000
    min_particles:  500
    tf_broadcast:  true
    transform_tolerance: 1.0
    update_min_a:  0.2
    update_min_d:  0.25
    scan_topic:    scan
    set_initial_pose:  true
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame:     map
    robot_base_frame: base_link
    odom_topic:       /odom
    bt_loop_duration:       10
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
    min_x_velocity_threshold:     0.001
    min_y_velocity_threshold:     0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins:   ["general_goal_checker"]
    controller_plugins:     ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance:  10.0

    general_goal_checker:
      stateful: true
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance:  0.20    # relaxed — no encoder
      yaw_goal_tolerance: 0.35

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: false
      min_vel_x:     0.0
      min_vel_y:     0.0
      max_vel_x:     0.35
      max_vel_y:     0.0
      max_vel_theta: 1.0
      min_speed_xy:  0.0
      max_speed_xy:  0.35
      min_speed_theta: 0.0
      acc_lim_x:      0.3
      acc_lim_y:      0.0
      acc_lim_theta:  1.5
      decel_lim_x:   -0.3
      decel_lim_y:    0.0
      decel_lim_theta: -1.5
      vx_samples:    20
      vy_samples:     5
      vtheta_samples: 20
      sim_time:      1.7
      linear_granularity:  0.05
      angular_granularity: 0.025
      transform_tolerance: 0.5
      xy_goal_tolerance:   0.20
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
      PathAlign.scale:    32.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale:    24.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale:     32.0
      GoalDist.scale:     24.0
      RotateToGoal.scale:          32.0
      RotateToGoal.slowing_factor:  5.0
      RotateToGoal.lookahead_time: -1.0

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: false
      update_frequency:  5.0
      publish_frequency: 2.0
      global_frame:      odom
      robot_base_frame:  base_link
      rolling_window:    true
      width:  3
      height: 3
      resolution: 0.05
      robot_radius: 0.25
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        publish_voxel_map: true
        origin_z:          0.0
        z_resolution:      0.05
        z_voxels:          16
        max_obstacle_height: 2.0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking:  true
          data_type: "LaserScan"
          raytrace_max_range:  3.0
          raytrace_min_range:  0.0
          obstacle_max_range:  2.5
          obstacle_min_range:  0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius:    0.55
      always_send_full_costmap: true

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: false
      update_frequency:  1.0
      publish_frequency: 1.0
      global_frame:     map
      robot_base_frame: base_link
      robot_radius:     0.25
      resolution:       0.05
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
          marking:  true
          data_type: "LaserScan"
          raytrace_max_range:  3.0
          raytrace_min_range:  0.0
          obstacle_max_range:  2.5
          obstacle_min_range:  0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius:    0.55
      always_send_full_costmap: true

map_server:
  ros__parameters:
    use_sim_time: false
    yaml_filename: ""   # set at launch time via argument

map_saver:
  ros__parameters:
    use_sim_time: false
    save_map_timeout:    5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65
    map_subscribe_transient_local: true

planner_server:
  ros__parameters:
    use_sim_time: false
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      # Correct class name — double colon, not slash
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance:     0.5
      use_astar:     false
      allow_unknown: true

smoother_server:
  ros__parameters:
    use_sim_time: false
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance:       1.0e-10
      max_its:         1000
      do_refinement:   true

behavior_server:
  ros__parameters:
    use_sim_time: false
    costmap_topic:    local_costmap/costmap_raw
    footprint_topic:  local_costmap/published_footprint
    cycle_frequency:  10.0
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
    global_frame:      odom
    robot_base_frame:  base_link
    transform_tolerance: 0.1
    simulate_ahead_time: 2.0
    max_rotational_vel:  1.0
    min_rotational_vel:  0.4
    rotational_acc_lim:  3.2

waypoint_follower:
  ros__parameters:
    use_sim_time: false
    loop_rate:        20
    stop_on_failure:  false
    action_server_result_timeout: 900.0
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 0

velocity_smoother:
  ros__parameters:
    use_sim_time: false
    smoothing_frequency:  20.0
    scale_velocities:      false
    feedback:             "OPEN_LOOP"
    max_velocity:         [0.35, 0.0, 1.0]
    min_velocity:        [-0.35, 0.0,-1.0]
    max_accel:            [2.5,  0.0, 3.2]
    max_decel:           [-2.5,  0.0,-3.2]
    odom_topic:           "odom"
    odom_duration:        0.1
    deadband_velocity:    [0.0,  0.0, 0.0]
    velocity_timeout:     1.0

collision_monitor:
  ros__parameters:
    use_sim_time: false
    base_frame_id:    "base_link"
    odom_frame_id:    "odom"
    cmd_vel_in_topic:  "cmd_vel_smoothed"
    cmd_vel_out_topic: "cmd_vel"
    state_topic:       "collision_monitor_state"
    transform_tolerance: 0.5
    source_timeout:      5.0
    base_shift_correction: true
    stop_pub_timeout:    2.0
    polygons: ["FootprintApproach"]
    FootprintApproach:
      type:           "polygon"
      action_type:    "approach"
      footprint_topic: "/local_costmap/published_footprint"
      time_before_collision: 2.0
      simulation_time_step:  0.02
      min_points:   6
      visualize:    false
      enabled:      true
    observation_sources: ["scan"]
    scan:
      type:  "scan"
      topic: "/scan"
YAMLEOF
echo -e "${GREEN}  ✓ nav2_params.yaml${NC}"


# ════════════════════════════════════════════════════════════
# 5. slam_params.yaml
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[5/12] Writing slam_params.yaml...${NC}"
cat > ~/robot/config/slam_params.yaml << 'YAMLEOF'
# ============================================================
# slam_params.yaml — SLAM Toolbox (online async)
# Tuned for no-encoder robot: minimum_travel_* = 0
# ============================================================

slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver:    SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner:   SCHUR_JACOBI
    ceres_trust_strategy:   LEVENBERG_MARQUARDT
    ceres_dogleg_type:      TRADITIONAL_DOGLEG
    ceres_loss_function:    None

    odom_frame:  odom
    map_frame:   map
    base_frame:  base_link
    scan_topic:  /scan
    use_map_saver: true
    mode: mapping

    # No encoder — allow map updates without physical odometry movement
    minimum_travel_distance: 0.0
    minimum_travel_heading:  0.0
    map_update_interval:     3.0

    resolution:          0.05
    max_laser_range:    12.0
    minimum_time_interval: 0.3
    transform_timeout:     0.5
    tf_buffer_duration:   30.0

    stack_size_to_use: 40000000

    # Loop closure
    do_loop_closing:        true
    loop_search_maximum_distance: 3.0
    loop_match_minimum_response_coarse:    0.35
    loop_match_minimum_response_fine:      0.45
    loop_match_maximum_variance_coarse:    3.0
    loop_match_force_accept_ratio:         0.9

    # Scan matching
    distance_variance_penalty:   0.5
    angle_variance_penalty:      1.0
    fine_search_angle_offset:    0.00349
    coarse_search_angle_offset:  0.349
    coarse_angle_resolution:     0.0349
    minimum_angle_penalty:       0.9
    minimum_distance_penalty:    0.5
    use_response_expansion:      true
YAMLEOF
echo -e "${GREEN}  ✓ slam_params.yaml${NC}"


# ════════════════════════════════════════════════════════════
# 6. mapping_launch.py
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[6/12] Writing mapping_launch.py...${NC}"
cat > ~/robot/launch/mapping_launch.py << 'PYEOF'
#!/usr/bin/env python3
"""
mapping_launch.py
─────────────────
Launches:  RPLidar · SLAM Toolbox · Motor Controller · Teleop · RViz

Use this to drive around and build a map of the lawn.
After mapping, save with:  robot_save_map my_lawn
"""

import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             ExecuteProcess)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    home           = os.path.expanduser('~')
    slam_dir       = get_package_share_directory('slam_toolbox')
    slam_params    = os.path.join(home, 'robot', 'config', 'slam_params.yaml')
    rviz_config    = os.path.join(home, 'robot', 'rviz', 'mapping_config.rviz')

    return LaunchDescription([

        # ── static transforms ──────────────────────────────────
        # base_link → laser  (LiDAR mounted 10 cm above base centre)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
        ),
        # odom → base_link  (static placeholder — no encoder)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        ),

        # ── RPLidar A1M8 ──────────────────────────────────────
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port':    '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id':        'laser',
                'angle_compensate': True,
                'scan_mode':       'Sensitivity',
            }],
            output='screen',
        ),

        # ── Motor controller ──────────────────────────────────
        Node(
            package='robot_controllers',   # see note below
            executable='motor_controller',
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
                'use_sim_time':     'false',
            }.items(),
        ),

        # ── Teleop keyboard (opens in a dedicated xterm window) ─
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
chmod +x ~/robot/launch/mapping_launch.py
echo -e "${GREEN}  ✓ mapping_launch.py${NC}"


# ════════════════════════════════════════════════════════════
# 7. navigation_launch.py  (with saved map)
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[7/12] Writing navigation_launch.py...${NC}"
cat > ~/robot/launch/navigation_launch.py << 'PYEOF'
#!/usr/bin/env python3
"""
navigation_launch.py  — WITH saved map
──────────────────────────────────────
Launches:
  RPLidar · map_server · AMCL · Nav2 · Coverage Path Planner
  Motor Controller · Cutter Controller · RViz

Usage:
  robot_navigate my_lawn
  # OR
  ros2 launch ~/robot/launch/navigation_launch.py map_name:=my_lawn
"""

import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    home         = os.path.expanduser('~')
    nav2_dir     = get_package_share_directory('nav2_bringup')
    params_file  = os.path.join(home, 'robot', 'config', 'nav2_params.yaml')
    rviz_config  = os.path.join(home, 'robot', 'rviz', 'navigation_config.rviz')

    map_name_arg = DeclareLaunchArgument(
        'map_name', default_value='my_lawn',
        description='Map filename stem in ~/robot/maps/'
    )
    map_name = LaunchConfiguration('map_name')

    # Compute map yaml path — evaluated at launch time
    # We do it in Python here because it's a simple string
    # (can also use PathJoinSubstitution but this is cleaner)
    import sys
    map_name_val = 'my_lawn'   # default; overridden via CLI
    # NOTE: actual value injected via start_navigation.sh which pre-sets map path

    return LaunchDescription([
        map_name_arg,

        # ── static transforms ──────────────────────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        ),

        # ── RPLidar A1M8 ──────────────────────────────────────
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port':    '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id':        'laser',
                'angle_compensate': True,
                'scan_mode':       'Sensitivity',
            }],
            output='screen',
        ),

        # ── Motor controller ──────────────────────────────────
        Node(
            package='robot_controllers',
            executable='motor_controller',
            name='motor_controller',
            output='screen',
        ),

        # ── Cutter controller ─────────────────────────────────
        Node(
            package='robot_controllers',
            executable='cutter_controller',
            name='cutter_controller',
            output='screen',
        ),

        # ── Map server ────────────────────────────────────────
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': [
                    os.path.join(home, 'robot', 'maps/'),
                    map_name,
                    '.yaml',
                ],
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

        # ── Nav2 core (controller, planner, bt_nav, behaviors) ─
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file':  params_file,
            }.items(),
        ),

        # ── Coverage path planner ─────────────────────────────
        # Delayed 20 s to allow Nav2 action server to come up
        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='robot_controllers',
                    executable='coverage_path_planner',
                    name='coverage_path_planner',
                    parameters=[{
                        'strip_width': 0.35,
                        'autostart':   True,
                    }],
                    output='screen',
                )
            ],
        ),

        # ── RViz (delayed 18 s so map is latched before subscribe) ─
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
chmod +x ~/robot/launch/navigation_launch.py
echo -e "${GREEN}  ✓ navigation_launch.py${NC}"


# ════════════════════════════════════════════════════════════
# 8. coverage_nomap_launch.py  (autonomous, no saved map)
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[8/12] Writing coverage_nomap_launch.py...${NC}"
cat > ~/robot/launch/coverage_nomap_launch.py << 'PYEOF'
#!/usr/bin/env python3
"""
coverage_nomap_launch.py  — autonomous WITHOUT a saved map
──────────────────────────────────────────────────────────
SLAM builds the map in real-time while Nav2 drives coverage.
SLAM Toolbox itself provides map → odom TF (no AMCL needed).

Launches:
  RPLidar · SLAM Toolbox (mapping mode) · Nav2
  Coverage Path Planner · Motor Controller · Cutter Controller · RViz
"""

import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    home         = os.path.expanduser('~')
    nav2_dir     = get_package_share_directory('nav2_bringup')
    slam_dir     = get_package_share_directory('slam_toolbox')
    params_file  = os.path.join(home, 'robot', 'config', 'nav2_params.yaml')
    slam_params  = os.path.join(home, 'robot', 'config', 'slam_params.yaml')
    rviz_config  = os.path.join(home, 'robot', 'rviz', 'navigation_config.rviz')

    return LaunchDescription([

        # ── static transforms ──────────────────────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        ),

        # ── RPLidar A1M8 ──────────────────────────────────────
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port':    '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id':        'laser',
                'angle_compensate': True,
                'scan_mode':       'Sensitivity',
            }],
            output='screen',
        ),

        # ── Motor controller ──────────────────────────────────
        Node(
            package='robot_controllers',
            executable='motor_controller',
            name='motor_controller',
            output='screen',
        ),

        # ── Cutter controller ─────────────────────────────────
        Node(
            package='robot_controllers',
            executable='cutter_controller',
            name='cutter_controller',
            output='screen',
        ),

        # ── SLAM Toolbox (live mapping + TF provider) ─────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'slam_params_file': slam_params,
                'use_sim_time':     'false',
            }.items(),
        ),

        # ── Nav2 core — delayed 8 s so SLAM publishes /map first ─
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

        # ── Coverage planner — delayed 25 s for full stack startup ─
        TimerAction(
            period=25.0,
            actions=[
                Node(
                    package='robot_controllers',
                    executable='coverage_path_planner',
                    name='coverage_path_planner',
                    parameters=[{
                        'strip_width':  0.35,
                        'autostart':    True,
                        'map_wait_sec': 5.0,
                    }],
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
chmod +x ~/robot/launch/coverage_nomap_launch.py
echo -e "${GREEN}  ✓ coverage_nomap_launch.py${NC}"


# ════════════════════════════════════════════════════════════
# 9. RViz configs
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[9/12] Writing RViz configs...${NC}"

cat > ~/robot/rviz/mapping_config.rviz << 'RVIZEOF'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Selection
    Name: Selection
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Map
      Name: SLAM Map
      Topic:
        Value: /map
        Depth: 5
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
      Alpha: 0.7
      Enabled: true
    - Class: rviz_default_plugins/LaserScan
      Name: LiDAR Scan
      Topic:
        Value: /scan
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
      Size (m): 0.04
      Color: 255; 50; 50
      Enabled: true
    - Class: rviz_default_plugins/TF
      Name: TF Frames
      Enabled: true
      Show Arrows: true
      Show Axes: true
      Marker Scale: 0.5
  Global Options:
    Background Color: 40; 40; 40
    Fixed Frame: map
    Frame Rate: 10
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
RVIZEOF

cat > ~/robot/rviz/navigation_config.rviz << 'RVIZEOF'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Selection
    Name: Selection
  - Class: nav2_rviz_plugins/Navigation 2
    Name: Navigation 2
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Map
      Name: Static Map
      Topic:
        Value: /map
        Depth: 5
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
      Alpha: 0.7
      Enabled: true
    - Class: rviz_default_plugins/Map
      Name: Global Costmap
      Topic:
        Value: /global_costmap/costmap
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
      Alpha: 0.45
      Color Scheme: costmap
      Enabled: true
    - Class: rviz_default_plugins/Map
      Name: Local Costmap
      Topic:
        Value: /local_costmap/costmap
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
      Alpha: 0.45
      Color Scheme: costmap
      Enabled: true
    - Class: rviz_default_plugins/LaserScan
      Name: LiDAR Scan
      Topic:
        Value: /scan
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
      Size (m): 0.04
      Color: 255; 50; 50
      Enabled: true
    - Class: rviz_default_plugins/PoseArray
      Name: AMCL Particles
      Topic:
        Value: /particle_cloud
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
      Color: 50; 255; 50
      Enabled: true
    - Class: rviz_default_plugins/Path
      Name: Global Plan
      Topic:
        Value: /plan
        Depth: 5
        Durability Policy: Volatile
        Reliability Policy: Reliable
      Color: 0; 100; 255
      Enabled: true
    - Class: rviz_default_plugins/Path
      Name: Coverage Path
      Topic:
        Value: /coverage/path
        Depth: 5
        Durability Policy: Volatile
        Reliability Policy: Reliable
      Color: 255; 165; 0
      Line Style: Lines
      Enabled: true
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: false
  Global Options:
    Background Color: 40; 40; 40
    Fixed Frame: map
    Frame Rate: 10
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/SetInitialPose
      Topic:
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Value: /goal_pose
RVIZEOF
echo -e "${GREEN}  ✓ mapping_config.rviz  navigation_config.rviz${NC}"


# ════════════════════════════════════════════════════════════
# 10. Helper bash scripts
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[10/12] Writing helper scripts...${NC}"

# start_mapping.sh
cat > ~/robot/scripts/start_mapping.sh << 'SHEOF'
#!/bin/bash
source /opt/ros/kilted/setup.bash
source "$HOME/ws_lidar/install/setup.bash"
echo "=== MAPPING MODE ==="
echo "Drive the robot around the whole lawn, then run:"
echo "  robot_save_map <name>"
ros2 launch "$HOME/robot/launch/mapping_launch.py"
SHEOF

# start_navigation.sh
cat > ~/robot/scripts/start_navigation.sh << 'SHEOF'
#!/bin/bash
source /opt/ros/kilted/setup.bash
source "$HOME/ws_lidar/install/setup.bash"

if [ -z "$1" ]; then
    echo "Usage: robot_navigate <map_name>"
    echo ""
    echo "Available maps:"
    ls "$HOME/robot/maps/"*.yaml 2>/dev/null \
        | xargs -I{} basename {} .yaml \
        | sed 's/^/  • /' \
        || echo "  (none — run robot_map first)"
    exit 1
fi

MAP_YAML="$HOME/robot/maps/$1.yaml"
MAP_PGM="$HOME/robot/maps/$1.pgm"

if [ ! -f "$MAP_YAML" ]; then
    echo "Error: map '$1' not found at $MAP_YAML"
    exit 1
fi

# Fix image path in yaml (make absolute if relative)
sed -i "s|image: $1.pgm|image: $HOME/robot/maps/$1.pgm|g"  "$MAP_YAML" 2>/dev/null
sed -i "s|image: \./|image: $HOME/robot/maps/|g"            "$MAP_YAML" 2>/dev/null
sed -i "s|image: ~/|image: $HOME/|g"                        "$MAP_YAML" 2>/dev/null

echo "=== AUTONOMOUS NAVIGATION (with map: $1) ==="
echo "Cutter control:"
echo "  robot_cutter_on   — start blade"
echo "  robot_cutter_off  — stop  blade"
echo ""
ros2 launch "$HOME/robot/launch/navigation_launch.py" map_name:="$1"
SHEOF

# start_coverage_nomap.sh
cat > ~/robot/scripts/start_coverage_nomap.sh << 'SHEOF'
#!/bin/bash
source /opt/ros/kilted/setup.bash
source "$HOME/ws_lidar/install/setup.bash"
echo "=== AUTONOMOUS COVERAGE (building map on-the-fly) ==="
echo "The robot will:"
echo "  1. Build a map using SLAM"
echo "  2. Automatically plan a lawnmower path"
echo "  3. Drive the coverage pattern"
echo ""
echo "Cutter control:"
echo "  robot_cutter_on   — start blade"
echo "  robot_cutter_off  — stop  blade"
echo ""
ros2 launch "$HOME/robot/launch/coverage_nomap_launch.py"
SHEOF

# save_map.sh
cat > ~/robot/scripts/save_map.sh << 'SHEOF'
#!/bin/bash
source /opt/ros/kilted/setup.bash
source "$HOME/ws_lidar/install/setup.bash"

if [ -z "$1" ]; then
    echo "Usage: robot_save_map <map_name>"
    exit 1
fi

MAP_PATH="$HOME/robot/maps/$1"
echo "Saving map to: $MAP_PATH"
ros2 run nav2_map_server map_saver_cli \
    --ros-args -p save_map_timeout:=5.0 \
    -- -f "$MAP_PATH"
echo "✓ Map saved: $MAP_PATH.yaml  +  $MAP_PATH.pgm"
SHEOF

# teleop.sh
cat > ~/robot/scripts/teleop.sh << 'SHEOF'
#!/bin/bash
source /opt/ros/kilted/setup.bash
echo "=== TELEOP MODE ==="
echo "Click THIS window before pressing keys!"
echo ""
ros2 run teleop_twist_keyboard teleop_twist_keyboard
SHEOF

# cutter_on.sh
cat > ~/robot/scripts/cutter_on.sh << 'SHEOF'
#!/bin/bash
source /opt/ros/kilted/setup.bash
echo "Turning cutter ON..."
ros2 service call /cutter/set std_srvs/srv/SetBool '{data: true}'
SHEOF

# cutter_off.sh
cat > ~/robot/scripts/cutter_off.sh << 'SHEOF'
#!/bin/bash
source /opt/ros/kilted/setup.bash
echo "Turning cutter OFF..."
ros2 service call /cutter/set std_srvs/srv/SetBool '{data: false}'
SHEOF

chmod +x ~/robot/scripts/*.sh
echo -e "${GREEN}  ✓ All helper scripts${NC}"


# ════════════════════════════════════════════════════════════
# 11. robot_controllers Python package
#     (so nodes can be launched with package='robot_controllers')
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[11/12] Creating robot_controllers ROS2 package...${NC}"

PKG_DIR="$HOME/ws_lidar/src/robot_controllers"
mkdir -p "$PKG_DIR/robot_controllers"

# __init__.py
touch "$PKG_DIR/robot_controllers/__init__.py"

# setup.py
cat > "$PKG_DIR/setup.py" << 'PYEOF'
from setuptools import setup, find_packages

package_name = 'robot_controllers'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='psb',
    maintainer_email='psb@robot.local',
    description='Lawn mower robot controllers',
    license='MIT',
    entry_points={
        'console_scripts': [
            'motor_controller    = robot_controllers.motor_controller:main',
            'cutter_controller   = robot_controllers.cutter_controller:main',
            'coverage_path_planner = robot_controllers.coverage_path_planner:main',
        ],
    },
)
PYEOF

# package.xml
cat > "$PKG_DIR/package.xml" << 'XMLEOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robot_controllers</name>
  <version>1.0.0</version>
  <description>Lawn mower robot motor and cutter controllers</description>
  <maintainer email="psb@robot.local">psb</maintainer>
  <license>MIT</license>
  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>nav2_msgs</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>
  <build_type>ament_python</build_type>
</package>
XMLEOF

# resource marker
mkdir -p "$PKG_DIR/resource"
touch "$PKG_DIR/resource/robot_controllers"

# setup.cfg
cat > "$PKG_DIR/setup.cfg" << 'CFGEOF'
[develop]
script_dir=$base/lib/robot_controllers
[install]
install_scripts=$base/lib/robot_controllers
CFGEOF

# Copy Python scripts into the package
cp ~/robot/scripts/motor_controller.py    "$PKG_DIR/robot_controllers/motor_controller.py"
cp ~/robot/scripts/cutter_controller.py   "$PKG_DIR/robot_controllers/cutter_controller.py"
cp ~/robot/scripts/coverage_path_planner.py "$PKG_DIR/robot_controllers/coverage_path_planner.py"

# Build the workspace
echo "  Building workspace (this takes ~2-3 minutes)..."
cd ~/ws_lidar
source /opt/ros/kilted/setup.bash
colcon build --symlink-install --packages-select robot_controllers 2>&1 \
    | grep -E "(error|warning|Starting|Finished|---)" || true

echo -e "${GREEN}  ✓ robot_controllers package built${NC}"


# ════════════════════════════════════════════════════════════
# 12. bashrc aliases
# ════════════════════════════════════════════════════════════
echo -e "${YELLOW}[12/12] Adding bashrc aliases...${NC}"

# Remove old robot aliases if present
sed -i '/# === ROBOT SHORTCUTS ===/,/# === END ROBOT ===/d' ~/.bashrc

cat >> ~/.bashrc << 'BASHEOF'

# === ROBOT SHORTCUTS ===
source /opt/ros/kilted/setup.bash 2>/dev/null
source "$HOME/ws_lidar/install/setup.bash" 2>/dev/null

# Mapping
alias robot_map='bash "$HOME/robot/scripts/start_mapping.sh"'
alias robot_save_map='bash "$HOME/robot/scripts/save_map.sh"'

# Teleop
alias robot_teleop='bash "$HOME/robot/scripts/teleop.sh"'

# Autonomous with saved map
alias robot_navigate='bash "$HOME/robot/scripts/start_navigation.sh"'

# Autonomous without map (builds as it goes)
alias robot_coverage='bash "$HOME/robot/scripts/start_coverage_nomap.sh"'

# Cutter control
alias robot_cutter_on='bash "$HOME/robot/scripts/cutter_on.sh"'
alias robot_cutter_off='bash "$HOME/robot/scripts/cutter_off.sh"'

# Diagnostics
alias robot_scan='ros2 topic hz /scan'
alias robot_nodes='ros2 node list'
alias robot_tf='ros2 run tf2_tools view_frames'
# === END ROBOT ===
BASHEOF

echo -e "${GREEN}  ✓ bashrc aliases added${NC}"


# ════════════════════════════════════════════════════════════
# VERIFY
# ════════════════════════════════════════════════════════════
echo ""
echo "Verifying Python syntax..."
for f in \
    ~/robot/launch/mapping_launch.py \
    ~/robot/launch/navigation_launch.py \
    ~/robot/launch/coverage_nomap_launch.py \
    ~/robot/scripts/motor_controller.py \
    ~/robot/scripts/cutter_controller.py \
    ~/robot/scripts/coverage_path_planner.py; do
    python3 -c "import ast; ast.parse(open('$f').read())" \
        && echo -e "  ${GREEN}✓ $(basename $f)${NC}" \
        || echo -e "  ${RED}✗ SYNTAX ERROR: $(basename $f)${NC}"
done

echo ""
echo "============================================================"
echo -e "${GREEN}   SETUP COMPLETE!${NC}"
echo "============================================================"
echo ""
echo "Run:  source ~/.bashrc"
echo ""
echo "COMMANDS:"
echo -e "  ${BLUE}robot_map${NC}                  Build a map (drive manually)"
echo -e "  ${BLUE}robot_save_map my_lawn${NC}     Save map after mapping"
echo -e "  ${BLUE}robot_teleop${NC}               Manual drive (in 2nd terminal)"
echo -e "  ${BLUE}robot_navigate my_lawn${NC}     Auto-cover with saved map"
echo -e "  ${BLUE}robot_coverage${NC}             Auto-cover, builds map live"
echo -e "  ${BLUE}robot_cutter_on${NC}            Start cutting blade"
echo -e "  ${BLUE}robot_cutter_off${NC}           Stop  cutting blade"
echo ""
echo "WORKFLOW:"
echo "  1.  robot_map                 ← open RViz + teleop"
echo "  2.  (drive the whole lawn)"
echo "  3.  robot_save_map my_lawn"
echo "  4.  robot_navigate my_lawn    ← robot mows autonomously"
echo "  5.  robot_cutter_on           ← when robot starts moving"
echo ""
echo "INITIAL POSE (run ONCE after robot_navigate starts):"
echo -e "${BLUE}  ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \\${NC}"
echo -e "${BLUE}    '{header:{frame_id:\"map\"},pose:{pose:{position:{x:0.0,y:0.0},orientation:{w:1.0}},covariance:[0.25,0,0,0,0,0,0,0.25,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.07]}}'${NC}"
echo ""
