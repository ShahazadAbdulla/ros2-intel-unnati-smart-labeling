# In nodes/conductor_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from rclpy.qos import QoSProfile, DurabilityPolicy
import time
import os
import signal

# In nodes/conductor_node.py
class ConductorNode(Node):
    def __init__(self):
        super().__init__('conductor_node')
        self.qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)

        self.start_system_pub_ = self.create_publisher(Bool, 'start_system', self.qos_profile)
        self.sequencer_command_pub_ = self.create_publisher(String, 'start_next_product', self.qos_profile)
        self.shutdown_pub_ = self.create_publisher(Bool, 'system_shutdown', self.qos_profile)

        self.command_sub_ = self.create_subscription(String, 'system_command', self.command_callback, self.qos_profile)
        
        self.get_logger().info('Conductor Node is ready.')

    def command_callback(self, msg: String):
        command = msg.data.upper()
        self.get_logger().info(f"Conductor received command: {command}")
        
        if command == "START":
            self.get_logger().info("Relaying command: START")
            self.start_system_pub_.publish(Bool(data=True))
            self.sequencer_command_pub_.publish(String(data="start"))
        elif command == "EXIT":
            self.get_logger().info("Initiating system-wide shutdown...")
            self.shutdown_pub_.publish(Bool(data=True))
            rclpy.shutdown()

    def shutdown_callback(self, msg: Bool): # This is for completeness, though unused in this logic
        if msg.data: self.destroy_node()
        
def main(args=None):
    rclpy.init(args=args)
    node = ConductorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # This handles Ctrl+C gracefully
        pass
    finally:
        # The node might already be destroyed by final_shutdown, so check first
        if rclpy.ok() and node.handle is not None:
             node.destroy_node()

if __name__ == '__main__':
    main()