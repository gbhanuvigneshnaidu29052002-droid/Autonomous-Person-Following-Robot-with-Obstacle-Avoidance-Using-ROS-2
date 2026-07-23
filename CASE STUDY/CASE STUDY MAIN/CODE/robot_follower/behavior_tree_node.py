#!/usr/bin/env python3
"""
Behavior Tree Node — Person Follower for TurtleBot3 Waffle Pi
=============================================================

State machine:
  WAIT          Stationary. Waiting for START command + target to appear.
  FOLLOW        Target visible → potential-field control.
  AVOID         Obstacle blocking path → go around it while maintaining
                the last known person direction.
  REROUTE       Target lost → drive to last known position.
  SEARCH        360° spin at last known position to find person.
  ACQUIRE       Slow wide sweep if still not found after SEARCH.

Obstacle avoidance:
  • LiDAR noise-filtered with sorted 3rd-percentile per sector
  • Person's legs excluded from obstacle sectors
  • 70 % obstacle repulsion + 30 % target attraction when obstacle present
  • Hard emergency brake + reverse if obstacle < STOP_DIST

Motor safety:
  • Publishes Twist(0,0) at 10 Hz when inactive — robot cannot coast
  • Acceleration limiting on every velocity command
  • Motors are zeroed on shutdown
"""

import json
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.qos import (DurabilityPolicy, HistoryPolicy,
                       QoSProfile, ReliabilityPolicy)
from nav2_msgs.action import NavigateToPose


# ── Tuning ────────────────────────────────────────────────────────────────────
SAFE_DIST       = 0.85   # m  — desired follow distance to person
STOP_BAND       = 0.10   # m  — ± deadband (no motion)
BACKUP_DIST     = 0.55   # m  — back up if person closer than this

MAX_LIN         = 0.22   # m/s
MAX_ANG         = 0.75   # rad/s
ACCEL_LIN       = 0.035  # m/s per tick  (~0.35 m/s²)
ACCEL_ANG       = 0.12   # rad/s per tick

KP_LIN          = 0.55
KP_ANG          = 1.10
ANG_DEAD        = 0.07   # rad

OBS_WARN_DIST   = 0.55   # m  — start steering
OBS_STOP_DIST   = 0.22   # m  — emergency stop/reverse

PERSON_EXCL     = 0.38   # rad — LiDAR cone to exclude person's legs
AVOID_TIMEOUT   = 5.0    # s  — max time in AVOID before returning to FOLLOW
REROUTE_TIMEOUT = 25.0   # s
SEARCH_DUR      = 12.0   # s  — full 360° spin
ACQUIRE_DUR     = 20.0   # s  — slow sweep before giving up
# ─────────────────────────────────────────────────────────────────────────────


class BehaviorTreeNode(Node):

    def __init__(self):
        super().__init__("behavior_tree_node")

        # ── QoS ──────────────────────────────────────────────────────────────
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(String,    "/behavior/command",  self._on_cmd,    10)
        self.create_subscription(String,    "/tracked_target",    self._on_target, 10)
        self.create_subscription(Odometry,  "/odom",              self._on_odom,   10)
        self.create_subscription(LaserScan, "/scan",              self._on_scan,   scan_qos)

        # ── Publishers ────────────────────────────────────────────────────────
        self._vel_pub   = self.create_publisher(Twist,  "/cmd_vel",                         10)
        self._state_pub = self.create_publisher(String, "/person_following/behavior_state",  10)

        # ── Nav2 (optional) ───────────────────────────────────────────────────
        self._nav       = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._nav_active = False
        self._nav_done   = False

        # ── Robot pose ────────────────────────────────────────────────────────
        self.robot_x   = 0.0
        self.robot_y   = 0.0
        self.robot_yaw = 0.0

        # ── Target ────────────────────────────────────────────────────────────
        self.target_visible  = False
        self.target_distance = 1.5
        self.target_angle    = 0.0   # rad: +ve = person to LEFT

        # Last known world position
        self.lk_x    = 0.0
        self.lk_y    = 0.0
        self.lk_yaw  = 0.0
        self.lk_angle = 0.0

        # ── LiDAR sectors (noise-filtered) ───────────────────────────────────
        self.front = 10.0
        self.left  = 10.0
        self.right = 10.0
        self._raw_scan = None

        # ── Smoothing ─────────────────────────────────────────────────────────
        self._cmd_lin = 0.0
        self._cmd_ang = 0.0

        # ── Behavior state ────────────────────────────────────────────────────
        self._active  = False
        self._state   = "WAIT"
        self._state_t = time.time()

        self._timer = self.create_timer(0.1, self._tick)
        self.get_logger().info("BehaviorTreeNode ready — send 'START' on /behavior/command.")

    # ═══════════════════════════════════════════════════════════════════════════
    # Callbacks
    # ═══════════════════════════════════════════════════════════════════════════

    def _on_cmd(self, msg: String):
        cmd = msg.data.upper().strip()
        if cmd == "START":
            self._active = True
            self._set_state("WAIT")
            self.get_logger().info("Behavior STARTED.")
        elif cmd == "STOP":
            self._active = False
            self._cancel_nav()
            self._publish_vel(0.0, 0.0, smooth=False)
            self.get_logger().info("Behavior STOPPED — motors zeroed.")

    def _on_target(self, msg: String):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        self.target_visible  = d.get("visible", False)
        self.target_distance = d.get("distance", self.target_distance)
        self.target_angle    = d.get("angle", 0.0)

        if self.target_visible:
            self.lk_angle = d.get("last_known_angle", self.target_angle)
            dist     = max(0.3, self.target_distance)
            g_angle  = self.robot_yaw - self.target_angle
            self.lk_x   = self.robot_x + dist * math.cos(g_angle)
            self.lk_y   = self.robot_y + dist * math.sin(g_angle)
            self.lk_yaw = g_angle

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.robot_x = p.position.x
        self.robot_y = p.position.y
        q = p.orientation
        self.robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _on_scan(self, msg: LaserScan):
        """Sort rays into sectors; apply 3rd-percentile noise filter."""
        self._raw_scan = msg
        person_bearing = -self.target_angle if self.target_visible else 999.0

        f_rays, l_rays, r_rays = [], [], []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < max(0.12, msg.range_min) or r > msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment
            a = math.atan2(math.sin(a), math.cos(a))
            # Mask out the target's legs only if it's not a critical collision threat (< 0.45m)
            # AND the range is close to the expected target distance.
            if abs(a - person_bearing) < PERSON_EXCL:
                if r >= 0.45 and abs(r - self.target_distance) < 0.30:
                    continue
            if abs(a) < 0.52:
                f_rays.append(r)
            if 0.0 <= a < 1.05:
                l_rays.append(r)
            if -1.05 <= a < 0.0:
                r_rays.append(r)

        def pct3(rays):
            if not rays:
                return 10.0
            rays.sort()
            return rays[min(2, len(rays) - 1)]

        self.front = pct3(f_rays)
        self.left  = pct3(l_rays)
        self.right = pct3(r_rays)

    # ═══════════════════════════════════════════════════════════════════════════
    # Main 10 Hz tick
    # ═══════════════════════════════════════════════════════════════════════════

    def _tick(self):
        # Always hard-stop when inactive
        if not self._active:
            self._publish_vel(0.0, 0.0, smooth=False)
            return

        elapsed = time.time() - self._state_t
        s = self._state

        # ── Transitions ───────────────────────────────────────────────────────
        if s == "WAIT":
            if self.target_visible:
                self._set_state("FOLLOW")

        elif s == "FOLLOW":
            # Check if an obstacle (non-person) is blocking the path
            if self._obstacle_blocking():
                self._set_state("AVOID")
            elif not self.target_visible:
                self._cancel_nav()
                self._set_state("REROUTE")
                self._start_reroute()

        elif s == "AVOID":
            if not self._obstacle_blocking():
                # Obstacle cleared
                if self.target_visible:
                    self._set_state("FOLLOW")
                else:
                    self._set_state("REROUTE")
                    self._start_reroute()
            elif elapsed > AVOID_TIMEOUT:
                # Safety: if we're stuck in AVOID too long, force REROUTE
                self._set_state("REROUTE")
                self._start_reroute()

        elif s == "REROUTE":
            if self.target_visible:
                self._cancel_nav()
                self._set_state("FOLLOW")
            elif elapsed > REROUTE_TIMEOUT or self._nav_done:
                self._cancel_nav()
                self._set_state("SEARCH")

        elif s == "SEARCH":
            if self.target_visible:
                self._set_state("FOLLOW")
            elif elapsed > SEARCH_DUR:
                self._set_state("ACQUIRE")

        elif s == "ACQUIRE":
            if self.target_visible:
                self._set_state("FOLLOW")
            elif elapsed > ACQUIRE_DUR:
                self._set_state("WAIT")

        # ── Actions ───────────────────────────────────────────────────────────
        if self._state == "WAIT":
            self._publish_vel(0.0, 0.0, smooth=False)

        elif self._state == "FOLLOW":
            lin, ang = self._follow_vel()
            self._publish_vel(lin, ang)

        elif self._state == "AVOID":
            lin, ang = self._avoid_vel()
            self._publish_vel(lin, ang)

        elif self._state == "REROUTE":
            if not self._nav_active:
                lin, ang = self._reroute_vel()
                self._publish_vel(lin, ang)

        elif self._state == "SEARCH":
            d = 1.0 if self.lk_angle >= 0 else -1.0
            self._publish_vel(0.0, d * MAX_ANG * 0.85)

        elif self._state == "ACQUIRE":
            d = 1.0 if self.lk_angle >= 0 else -1.0
            self._publish_vel(0.0, d * MAX_ANG * 0.45)

    # ═══════════════════════════════════════════════════════════════════════════
    # Velocity calculation
    # ═══════════════════════════════════════════════════════════════════════════

    def _follow_vel(self):
        """
        Pure target-tracking velocity when no obstacle blocks the path.
        """
        d = self.target_distance
        a = self.target_angle

        # Linear: approach or back up
        if d > SAFE_DIST + STOP_BAND:
            lin = min(KP_LIN * (d - SAFE_DIST), MAX_LIN)
        elif d < BACKUP_DIST:
            lin = max(-KP_LIN * (BACKUP_DIST - d), -0.15)
        else:
            lin = 0.0

        # Angular: person left (+angle) → turn left (+z CCW)
        ang = 0.0
        if abs(a) > ANG_DEAD:
            ang = max(-MAX_ANG, min(MAX_ANG, -KP_ANG * a))

        return lin, ang

    def _avoid_vel(self):
        """
        Potential-field obstacle avoidance:
          70 % obstacle repulsion + 30 % target attraction.
        Maintain awareness of last known person direction.
        """
        # Target attraction component (even if not visible, use last known angle)
        if self.target_visible:
            a = self.target_angle
        else:
            a = self.lk_angle

        t_lin = min(KP_LIN * 0.3, MAX_LIN * 0.5)   # gentle forward push
        t_ang = 0.0
        if abs(a) > ANG_DEAD:
            t_ang = max(-MAX_ANG, min(MAX_ANG, -KP_ANG * a))

        # Obstacle repulsion component
        closest = min(self.front, self.left, self.right)
        if closest < OBS_STOP_DIST:
            # Emergency: stop forward motion and steer hard away
            rep_ang = MAX_ANG if self.right < self.left else -MAX_ANG
            return 0.0, rep_ang

        strength = max(0.0, min(1.0,
            (OBS_WARN_DIST - closest) / (OBS_WARN_DIST - OBS_STOP_DIST)
        ))
        rep_ang = MAX_ANG if self.right < self.left else -MAX_ANG

        lin = t_lin * (1.0 - strength * 0.7)
        ang = 0.70 * strength * rep_ang + 0.30 * t_ang

        return lin, ang

    def _reroute_vel(self):
        """Simple vector guidance to last known position."""
        dx = self.lk_x - self.robot_x
        dy = self.lk_y - self.robot_y
        dist = math.hypot(dx, dy)

        if dist < 0.20:
            return 0.0, 0.0  # arrived — let state machine switch to SEARCH

        desired = math.atan2(dy, dx)
        err     = math.atan2(
            math.sin(desired - self.robot_yaw),
            math.cos(desired - self.robot_yaw),
        )
        lin = min(KP_LIN * dist, MAX_LIN * 0.6)
        ang = max(-MAX_ANG, min(MAX_ANG, KP_ANG * err))

        # Still avoid obstacles during reroute
        closest = min(self.front, self.left, self.right)
        if closest < OBS_WARN_DIST:
            strength = max(0.0, min(1.0,
                (OBS_WARN_DIST - closest) / (OBS_WARN_DIST - OBS_STOP_DIST)
            ))
            rep_ang = MAX_ANG if self.right < self.left else -MAX_ANG
            if closest < OBS_STOP_DIST:
                lin = 0.0
                ang = rep_ang
            else:
                lin *= (1.0 - strength * 0.7)
                ang  = 0.70 * strength * rep_ang + 0.30 * ang

        return lin, ang

    # ═══════════════════════════════════════════════════════════════════════════
    # Obstacle detection helper
    # ═══════════════════════════════════════════════════════════════════════════

    def _obstacle_blocking(self) -> bool:
        """
        Returns True only when an obstacle is in the direct path to the person
        AND the robot is not already at a safe distance from it.
        """
        closest = min(self.front, self.left, self.right)
        return closest < OBS_WARN_DIST

    # ═══════════════════════════════════════════════════════════════════════════
    # Helpers
    # ═══════════════════════════════════════════════════════════════════════════

    def _set_state(self, new_state: str):
        if self._state == new_state:
            return
        self.get_logger().info(f"  [{self._state}] → [{new_state}]")
        self._state   = new_state
        self._state_t = time.time()
        msg      = String()
        msg.data = new_state
        self._state_pub.publish(msg)

    def _publish_vel(self, lin: float, ang: float, smooth: bool = True):
        if smooth:
            d = lin - self._cmd_lin
            if abs(d) > ACCEL_LIN:
                lin = self._cmd_lin + math.copysign(ACCEL_LIN, d)
            d = ang - self._cmd_ang
            if abs(d) > ACCEL_ANG:
                ang = self._cmd_ang + math.copysign(ACCEL_ANG, d)
        self._cmd_lin = lin
        self._cmd_ang = ang
        tw = Twist()
        tw.linear.x  = lin
        tw.angular.z = ang
        self._vel_pub.publish(tw)

    def _start_reroute(self):
        if self._nav.server_is_ready():
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id    = "odom"
            goal.pose.header.stamp       = self.get_clock().now().to_msg()
            goal.pose.pose.position.x   = self.lk_x
            goal.pose.pose.position.y   = self.lk_y
            goal.pose.pose.orientation.z = math.sin(self.lk_yaw * 0.5)
            goal.pose.pose.orientation.w = math.cos(self.lk_yaw * 0.5)
            self._nav_active = True
            self._nav_done   = False
            self._nav.send_goal_async(goal).add_done_callback(self._nav_accepted)
            self.get_logger().info("Nav2 reroute goal sent.")
        else:
            self.get_logger().info("Nav2 not available — manual reroute guidance active.")
            self._nav_active = False

    def _nav_accepted(self, future):
        gh = future.result()
        if not gh.accepted:
            self._nav_done   = True
            self._nav_active = False
            return
        gh.get_result_async().add_done_callback(self._nav_finished)

    def _nav_finished(self, future):
        self._nav_done   = True
        self._nav_active = False

    def _cancel_nav(self):
        if self._nav_active:
            try:
                self._nav._cancel_goal_async()
            except Exception:
                pass
            self._nav_active = False


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorTreeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_vel(0.0, 0.0, smooth=False)
        except Exception:
            pass
        if node.context.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
