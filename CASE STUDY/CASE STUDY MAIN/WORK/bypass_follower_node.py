#!/usr/bin/env python3
"""
Obstacle-Bypass Follower — Specific Scenario
=============================================

Layout (everything on one line):

   [ROBOT] ──────────── [OBSTACLE] ──── [HUMAN]
              ~80 cm           ~20 cm
   ← ─ ─ ─ ─ ─ 1 m total ─ ─ ─ ─ ─ → 

Behaviour sequence
──────────────────
  APPROACH      Robot moves forward toward human.
                When front LiDAR < 0.80 m → obstacle detected → BYPASS_TURN

  BYPASS_TURN   Robot turns 90° toward the clearer side (left or right).
                Turn is complete when the front is clear (> 1.0 m).
                → BYPASS_PASS

  BYPASS_PASS   Robot drives forward a fixed distance to physically clear the
                obstacle (≈ 0.60 m forward arc).
                → SEARCH

  SEARCH        Robot rotates slowly back toward where the human should be
                until perception reports the human as visible.
                → FOLLOW

  FOLLOW        Standard PD follow with safe distance (0.70 m).
                If front obstacle re-appears < 0.80 m → back to BYPASS_TURN

Run with:
  ros2 run robot_follower bypass_follower_node
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.qos import (DurabilityPolicy, HistoryPolicy,
                       QoSProfile, ReliabilityPolicy)

# ── Tuning ────────────────────────────────────────────────────────────────────
OBSTACLE_TRIGGER_DIST = 0.80   # m  — start bypass when obstacle < this
FRONT_CLEAR_DIST      = 1.00   # m  — "front is clear" threshold
BYPASS_FWD_DIST       = 0.60   # m  — how far to drive past the obstacle
SAFE_FOLLOW_DIST      = 0.70   # m  — desired distance when following
STOP_BAND             = 0.08   # m  — ± deadband around safe follow distance
BACKUP_THRESHOLD      = 0.50   # m  — closer than this → back up

TURN_SPEED            = 0.55   # rad/s
FWD_SPEED             = 0.18   # m/s  — slow, controlled
FOLLOW_KP_LIN         = 0.50
FOLLOW_KP_ANG         = 1.00
MAX_ANG               = 0.70   # rad/s
SEARCH_SPEED          = 0.35   # rad/s

OBS_EXCL_RAD          = 0.35   # rad  — exclude person from obstacle LiDAR
ACCEL_LIN             = 0.04   # m/s per tick
ACCEL_ANG             = 0.12   # rad/s per tick
# ─────────────────────────────────────────────────────────────────────────────


class BypassFollowerNode(Node):

    def __init__(self):
        super().__init__("bypass_follower_node")

        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(LaserScan, "/scan",           self._on_scan,   scan_qos)
        self.create_subscription(String,    "/tracked_target", self._on_target, 10)
        self.create_subscription(String,    "/behavior/command", self._on_cmd,  10)

        self._vel_pub   = self.create_publisher(Twist,  "/cmd_vel", 10)
        self._state_pub = self.create_publisher(String, "/person_following/behavior_state", 10)

        # LiDAR sectors
        self.front = 10.0
        self.left  = 10.0
        self.right = 10.0

        # Target
        self.target_visible  = False
        self.target_distance = 2.0
        self.target_angle    = 0.0

        # Odometry-free distance tracking using time + speed
        self._bypass_fwd_start   = None   # time when BYPASS_PASS started
        self._bypass_fwd_target  = BYPASS_FWD_DIST / FWD_SPEED  # seconds to drive

        # Which side to bypass
        self._bypass_dir = 1.0   # +1 = left, -1 = right

        # Smoothing
        self._cmd_lin = 0.0
        self._cmd_ang = 0.0

        # State
        self._active = False
        self._state  = "WAIT"
        self._state_t = time.time()

        self._timer = self.create_timer(0.1, self._tick)
        self.get_logger().info("BypassFollowerNode ready.")
        self.get_logger().info("Send 'START' on /behavior/command to begin.")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_cmd(self, msg: String):
        cmd = msg.data.upper().strip()
        if cmd == "START":
            self._active = True
            self._set_state("APPROACH")
            self.get_logger().info("START received — approaching target line.")
        elif cmd == "STOP":
            self._active = False
            self._pub(0.0, 0.0, smooth=False)
            self.get_logger().info("STOP — motors zeroed.")

    def _on_target(self, msg: String):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        self.target_visible  = d.get("visible", False)
        self.target_distance = d.get("distance", self.target_distance)
        self.target_angle    = d.get("angle", 0.0)

    def _on_scan(self, msg: LaserScan):
        person_bearing = -self.target_angle if self.target_visible else 999.0
        f_rays, l_rays, r_rays = [], [], []

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < max(0.12, msg.range_min) or r > msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment
            a = math.atan2(math.sin(a), math.cos(a))
            if abs(a - person_bearing) < OBS_EXCL_RAD:
                continue
            if abs(a) < 0.45:
                f_rays.append(r)
            if 0.0 <= a < 1.05:
                l_rays.append(r)
            if -1.05 <= a < 0.0:
                r_rays.append(r)

        def pct3(rays):
            if not rays: return 10.0
            rays.sort()
            return rays[min(2, len(rays) - 1)]

        self.front = pct3(f_rays)
        self.left  = pct3(l_rays)
        self.right = pct3(r_rays)

    # ── Main tick ─────────────────────────────────────────────────────────────

    def _tick(self):
        if not self._active:
            self._pub(0.0, 0.0, smooth=False)
            return

        elapsed = time.time() - self._state_t
        s = self._state

        # ── Transitions ───────────────────────────────────────────────────────

        if s == "APPROACH":
            if self.front < OBSTACLE_TRIGGER_DIST:
                # Choose bypass direction: pick whichever side is more open
                self._bypass_dir = 1.0 if self.left >= self.right else -1.0
                side = "LEFT" if self._bypass_dir > 0 else "RIGHT"
                self.get_logger().info(
                    f"Obstacle at {self.front:.2f} m — bypassing {side}."
                )
                self._set_state("BYPASS_TURN")

        elif s == "BYPASS_TURN":
            # Done turning when front is clear enough
            if self.front > FRONT_CLEAR_DIST:
                self._bypass_fwd_start = time.time()
                self._set_state("BYPASS_PASS")

        elif s == "BYPASS_PASS":
            fwd_elapsed = time.time() - self._bypass_fwd_start
            if fwd_elapsed >= self._bypass_fwd_target:
                self._set_state("SEARCH")

        elif s == "SEARCH":
            if self.target_visible:
                self.get_logger().info("Human found — switching to FOLLOW.")
                self._set_state("FOLLOW")
            elif elapsed > 15.0:
                # Safety: if we can't find the human after 15s, go back to approach
                self.get_logger().warn("Human not found after search — retrying APPROACH.")
                self._set_state("APPROACH")

        elif s == "FOLLOW":
            # Re-trigger bypass if obstacle re-appears
            if self.front < OBSTACLE_TRIGGER_DIST and not self.target_visible:
                self._bypass_dir = 1.0 if self.left >= self.right else -1.0
                self._set_state("BYPASS_TURN")

        # ── Actions ───────────────────────────────────────────────────────────

        if self._state == "APPROACH":
            self._action_approach()

        elif self._state == "BYPASS_TURN":
            self._action_bypass_turn()

        elif self._state == "BYPASS_PASS":
            self._action_bypass_pass()

        elif self._state == "SEARCH":
            self._action_search()

        elif self._state == "FOLLOW":
            self._action_follow()

    # ── Actions ───────────────────────────────────────────────────────────────

    def _action_approach(self):
        """Move forward slowly, centred on the person or straight ahead."""
        ang = 0.0
        if self.target_visible and abs(self.target_angle) > 0.07:
            ang = max(-MAX_ANG, min(MAX_ANG, -FOLLOW_KP_ANG * self.target_angle))
        self._pub(FWD_SPEED, ang)

    def _action_bypass_turn(self):
        """Rotate in bypass direction until front clears."""
        self._pub(0.0, self._bypass_dir * TURN_SPEED)

    def _action_bypass_pass(self):
        """Drive forward to physically clear the obstacle."""
        self._pub(FWD_SPEED, 0.0)

    def _action_search(self):
        """Rotate back (opposite of bypass direction) to find the human."""
        # Rotate back toward where the human should be (opposite side)
        self._pub(0.0, -self._bypass_dir * SEARCH_SPEED)

    def _action_follow(self):
        """Standard PD follow."""
        d = self.target_distance
        a = self.target_angle

        if not self.target_visible:
            self._pub(0.0, 0.0)
            return

        # Linear
        if d > SAFE_FOLLOW_DIST + STOP_BAND:
            lin = min(FOLLOW_KP_LIN * (d - SAFE_FOLLOW_DIST), FWD_SPEED * 1.3)
        elif d < BACKUP_THRESHOLD:
            lin = -0.12
        else:
            lin = 0.0

        # Angular (person left → CCW = +z, person right → CW = -z)
        ang = 0.0
        if abs(a) > 0.07:
            ang = max(-MAX_ANG, min(MAX_ANG, -FOLLOW_KP_ANG * a))

        self._pub(lin, ang)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _set_state(self, new_state: str):
        if self._state == new_state:
            return
        self.get_logger().info(f"  [{self._state}] → [{new_state}]")
        self._state   = new_state
        self._state_t = time.time()
        msg      = String()
        msg.data = new_state
        self._state_pub.publish(msg)

    def _pub(self, lin: float, ang: float, smooth: bool = True):
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


def main(args=None):
    rclpy.init(args=args)
    node = BypassFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._pub(0.0, 0.0, smooth=False)
        except Exception:
            pass
        if node.context.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
