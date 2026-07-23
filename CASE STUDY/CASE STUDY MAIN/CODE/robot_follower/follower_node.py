"""
follower_node.py  –  v3  (wide-scan swerve + creep)

⚠  Run this OR behavior_tree_node, NEVER both simultaneously.

This is the SIMPLE fallback node (no Nav2).  Use behavior_tree_node for
the milestone demo.  This node is useful for quick tests or if Nav2 is not
running.

Fixes vs v1:
  - Wide ±60° LiDAR scan (was 7 rays / ~3°)
  - Swerve around obstacles instead of hard-stop
  - Creep forward when target visible but LiDAR distance unknown (-1)
  - BT transition bug fixed (FOLLOW always goes to SEARCH on loss)
"""

import math
import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg   import LaserScan
from std_msgs.msg      import String


class FollowerNode(Node):

    SAFE_DISTANCE        = 1.0
    CREEP_SPEED          = 0.08   # m/s
    MAX_LINEAR_SPEED     = 0.20
    MAX_ANGULAR_SPEED    = 0.50
    ANGLE_TOLERANCE      = 0.10
    KP_LINEAR            = 0.50
    KP_ANGULAR           = 2.0
    EMERGENCY_STOP_DIST  = 0.35
    CAUTION_DIST         = 0.60
    SIDE_CLEAR_DIST      = 0.50

    def __init__(self):
        super().__init__('follower_node')

        self.cmd_pub    = self.create_publisher(Twist,  '/cmd_vel',         10)
        self.target_sub = self.create_subscription(
            String,    '/tracked_target', self.target_callback, 10)
        self.scan_sub   = self.create_subscription(
            LaserScan, '/scan',           self.scan_callback,   10)

        self.min_front   = 999.0
        self.min_left    = 999.0
        self.min_right   = 999.0
        self.front_clear = True
        self.left_clear  = True
        self.right_clear = True

        self.get_logger().info('Follower node v3 started')
        self.get_logger().warn('Do NOT run behavior_tree_node at the same time!')

    # ------------------------------------------------------------------

    def scan_callback(self, msg):
        front, left, right = [], [], []
        for i, r in enumerate(msg.ranges):
            if not (msg.range_min < r < msg.range_max):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if   -0.30 <= angle <= 0.30:
                front.append(r)
            elif  0.30 <  angle <= 1.05:
                left.append(r)
            elif -1.05 <= angle < -0.30:
                right.append(r)

        self.min_front = min(front) if front else 999.0
        self.min_left  = min(left)  if left  else 999.0
        self.min_right = min(right) if right else 999.0
        self.front_clear = self.min_front > self.CAUTION_DIST
        self.left_clear  = self.min_left  > self.SIDE_CLEAR_DIST
        self.right_clear = self.min_right > self.SIDE_CLEAR_DIST

    def target_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        twist   = Twist()
        visible = data.get('visible', False)
        dist    = data.get('distance', -1.0)
        angle   = data.get('angle', 0.0)

        # Emergency stop
        if not self.front_clear and not self.left_clear and not self.right_clear:
            self.get_logger().warn('SAFETY full-stop: all sectors blocked')
            self.cmd_pub.publish(twist)
            return

        if not visible:
            self.cmd_pub.publish(twist)
            return

        # Linear
        if dist < 0:
            if self.front_clear:
                twist.linear.x = self.CREEP_SPEED
        elif dist > self.SAFE_DISTANCE:
            error   = dist - self.SAFE_DISTANCE
            raw_lin = self.KP_LINEAR * error
            if self.min_front < self.CAUTION_DIST:
                scale = max(0.0,
                            (self.min_front - self.EMERGENCY_STOP_DIST) /
                            (self.CAUTION_DIST - self.EMERGENCY_STOP_DIST))
                raw_lin *= scale
            twist.linear.x = min(raw_lin, self.MAX_LINEAR_SPEED)

        # Swerve
        if not self.front_clear:
            gap   = max(0.0, self.min_front - self.EMERGENCY_STOP_DIST)
            scale = gap / max(self.CAUTION_DIST - self.EMERGENCY_STOP_DIST, 1e-6)
            twist.linear.x = min(twist.linear.x,
                                  self.MAX_LINEAR_SPEED * scale * 0.5)
            if self.left_clear and not self.right_clear:
                twist.angular.z =  self.MAX_ANGULAR_SPEED * 0.8
            elif self.right_clear and not self.left_clear:
                twist.angular.z = -self.MAX_ANGULAR_SPEED * 0.8
            else:
                twist.angular.z = (-self.MAX_ANGULAR_SPEED * 0.6
                                   if angle >= 0
                                   else  self.MAX_ANGULAR_SPEED * 0.6)
            self.get_logger().info(
                f'SWERVE F={self.min_front:.2f} L={self.min_left:.2f} '
                f'R={self.min_right:.2f}')
            self.cmd_pub.publish(twist)
            return

        # Angular – turn toward target
        if abs(angle) > self.ANGLE_TOLERANCE:
            raw = -self.KP_ANGULAR * angle
            twist.angular.z = max(-self.MAX_ANGULAR_SPEED,
                                   min(self.MAX_ANGULAR_SPEED, raw))

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()