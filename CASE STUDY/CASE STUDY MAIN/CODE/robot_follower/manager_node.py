#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class ManagerNode(Node):
    """
    High-level state manager.
    Coordinates Perception (TRAIN) and Behavior (START/STOP).
    """
    def __init__(self):
        super().__init__('manager_node')
        self.perception_pub = self.create_publisher(String, '/perception/command', 10)
        self.behavior_pub = self.create_publisher(String, '/behavior/command', 10)
        self.target_sub = self.create_subscription(String, '/tracked_target', self.target_callback, 10)
        
        self.state = "INIT"
        self.get_logger().info("Manager node started. Training target in 5 seconds...")
        self.timer = self.create_timer(5.0, self.train_target)
        
    def train_target(self):
        self.timer.cancel()
        msg = String()
        msg.data = "TRAIN"
        self.perception_pub.publish(msg)
        self.state = "TRAINING"
        self.get_logger().info("Sent TRAIN command to perception.")
        
    def target_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except:
            return
            
        if self.state == "TRAINING" and data.get("visible", False):
            self.state = "RUNNING"
            bmsg = String()
            bmsg.data = "START"
            self.behavior_pub.publish(bmsg)
            self.get_logger().info("Target locked. Sent START command to behavior.")

def main(args=None):
    rclpy.init(args=args)
    node = ManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
