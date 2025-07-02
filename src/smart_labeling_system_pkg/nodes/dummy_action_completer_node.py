# In nodes/dummy_action_completer_node.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from smart_labeling_interfaces.msg import ValidationOutcome, ProductInfo # <<< Added ProductInfo
from std_msgs.msg import Int32
import re

class DummyActionCompleterNode(Node):
    def __init__(self):
        super().__init__('dummy_action_completer_node')
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)
        
        # --- Publishers ---
        self.action_complete_publisher = self.create_publisher(Int32, 'pybullet_action_complete', qos)
        # <<< NEW: This publisher fakes the "at inspection" signal >>>
        self.at_inspection_publisher = self.create_publisher(Int32, 'product_at_inspection', qos)
        
        # --- Subscribers ---
        # <<< NEW: Listens for new product info to provide the initial trigger >>>
        self.info_subscriber = self.create_subscription(
            ProductInfo, 'product_info', self.info_callback, qos)
        
        # This still listens for the AI's final decision to complete the loop
        self.outcome_subscriber = self.create_subscription(
            ValidationOutcome, 'validation_outcome', self.outcome_callback, qos)
            
        self.get_logger().info('Dummy Action Completer is running (No-Sim Mode).')

    def info_callback(self, msg: ProductInfo):
        """
        When new product info is published, this fakes the signal that the
        product has arrived at the inspection station.
        """
        device_id_str = msg.device_id
        product_id = self._parse_id(device_id_str)
        
        if product_id != -1:
            self.get_logger().info(f"Received info for {device_id_str}. Faking 'at inspection' trigger for ID {product_id}.")
            trigger_msg = Int32()
            trigger_msg.data = product_id
            self.at_inspection_publisher.publish(trigger_msg)
        else:
            self.get_logger().warn(f"Could not parse product ID from '{device_id_str}' in info_callback.")


    def outcome_callback(self, msg: ValidationOutcome):
        """When AI validation is done, send the correct completion ID."""
        device_id_str = msg.device_id
        product_id = self._parse_id(device_id_str)
        
        if product_id != -1:
            self.get_logger().info(f"Received outcome for {device_id_str}. Faking 'action complete' for ID {product_id}.")
            completion_msg = Int32()
            completion_msg.data = product_id
            self.action_complete_publisher.publish(completion_msg)
        else:
            self.get_logger().warn(f"Could not parse product ID from '{device_id_str}' in outcome_callback.")

    def _parse_id(self, device_id: str) -> int:
        """Helper function to extract the integer ID from a device ID string."""
        match = re.search(r'\d+$', device_id)
        if match:
            return int(match.group())
        return -1


def main(args=None):
    rclpy.init(args=args)
    node = DummyActionCompleterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok() and node.handle is not None:
            node.destroy_node()

if __name__ == '__main__':
    main()