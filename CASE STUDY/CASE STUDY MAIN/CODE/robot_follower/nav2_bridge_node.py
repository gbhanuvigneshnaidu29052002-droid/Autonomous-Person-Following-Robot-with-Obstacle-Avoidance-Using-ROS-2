#!/usr/bin/env python3
import json
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_from_euler

class Nav2BridgeNode(Node):
    def __init__(self):
        super().__init__('nav2_bridge_node')
        self.goal_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.target_sub = self.create_subscription(String, '/tracked_target', self.target_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cancel_pub = self.create_publisher(String, '/nav2_cancel', 10)
        self.latest_target = None
        self.obstacle_detected = False
        self.min_front_dist = 0.40
        self.last_goal_time = self.get_clock().now()
        self.goal_active = False
        self.goal_cooldown_sec = 3.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self.tick)
        self.get_logger().info('Nav2 bridge ready')

    def scan_callback(self, msg):
        ctr = int((0.0 - msg.angle_min) / msg.angle_increment)
        self.obstacle_detected = False
        for i in range(ctr - 10, ctr + 11):
            if 0 <= i < len(msg.ranges):
                r = msg.ranges[i]
                if msg.range_min <= r <= msg.range_max and r < self.min_front_dist:
                    self.obstacle_detected = True
                    break

    def target_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.latest_target = data

    def tick(self):
        if self.latest_target is None:
            return
        status = self.latest_target.get('status', 'lost')
        visible = self.latest_target.get('visible', False)
        if self.obstacle_detected:
            return
        if visible and status == 'visible':
            now = self.get_clock().now()
            if (now - self.last_goal_time).nanoseconds / 1e9 < self.goal_cooldown_sec and self.goal_active:
                return
            goal = self._build_goal_from_target()
            if goal is None:
                return
            self._send_goal(goal)
            self.last_goal_time = now
            self.goal_active = True
        elif status == 'lost' and self.goal_active:
            self.goal_active = False

    def _build_goal_from_target(self):
        angle = float(self.latest_target.get('angle', 0.0))
        distance = float(self.latest_target.get('distance', -1.0))
        if distance <= 0.0:
            distance = 1.5
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        x = trans.transform.translation.x + math.cos(angle) * max(distance - 0.8, 0.5)
        y = trans.transform.translation.y + math.sin(angle) * max(distance - 0.8, 0.5)
        yaw = math.atan2(math.sin(angle), math.cos(angle))
        q = quaternion_from_euler(0.0, 0.0, yaw)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.x = float(q[0])
        goal.pose.orientation.y = float(q[1])
        goal.pose.orientation.z = float(q[2])
        goal.pose.orientation.w = float(q[3])
        return goal

    def _send_goal(self, pose):
        if not self.goal_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 action server not available')
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.goal_client.send_goal_async(goal_msg)
        self.get_logger().info(f'Sent Nav2 goal: x={pose.pose.position.x:.2f} y={pose.pose.position.y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = Nav2BridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
