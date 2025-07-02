import rclpy
from rclpy.node import Node
import pandas as pd
from ament_index_python.packages import get_package_share_directory
import os

from smart_labeling_interfaces.msg import ProductInfo
from std_msgs.msg import String, Int32
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool

# In nodes/product_sequencer_node.py
class ProductSequencerNode(Node):
    def __init__(self):
        super().__init__('product_sequencer_node')
        self.qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)
        # ... (CSV loading is fine)
        self.declare_parameter('csv_file', 'products_test.csv')
        pkg_share_path = get_package_share_directory('smart_labeling_system_pkg')
        csv_file_name = self.get_parameter('csv_file').get_parameter_value().string_value
        self.csv_path = os.path.join(pkg_share_path, 'config', csv_file_name)
        self.product_data = self._load_product_data()
        self.total_products = len(self.product_data)
        
        self.current_product_id = 0
        self.has_started = False

        self.info_publisher_ = self.create_publisher(ProductInfo, 'product_info', self.qos_profile)
        self.command_subscriber_ = self.create_subscription(String, 'start_next_product', self.command_callback, self.qos_profile)
        self.completion_subscriber_ = self.create_subscription(Int32, 'pybullet_action_complete', self.completion_callback, self.qos_profile)
        self.shutdown_subscriber = self.create_subscription(Bool, 'system_shutdown', self.shutdown_callback, self.qos_profile)
        self.get_logger().info('Product Sequencer Node is ready.')

    def _load_product_data(self):
        try:
            df = pd.read_csv(self.csv_path, dtype=str).fillna('')
            df['RoHS_Compliant'] = df['RoHS_Compliant'].str.upper().map({'TRUE': True, 'FALSE': False}).fillna(False)
            return df.to_dict('records')
        except Exception as e:
            self.get_logger().error(f"Could not read product data file: {e}")
            return []

    def command_callback(self, msg: String):
        command = msg.data.lower()
        if command == "start" and not self.has_started:
            self.get_logger().info("START trigger received. Processing first product.")
            self.has_started = True
            self.current_product_id = 1
            self.publish_product_by_id(self.current_product_id)

    def completion_callback(self, msg: Int32):
        if not self.has_started: return
        self.get_logger().info(f"Completion received for product ID {msg.data}. Processing next.")
        self.current_product_id = msg.data + 1
        self.publish_product_by_id(self.current_product_id)

    def publish_product_by_id(self, product_id):
        index = product_id - 1
        if not (0 <= index < self.total_products):
            self.get_logger().info(f'Sequence complete. All {self.total_products} products have been processed.')
            self.has_started = False # Allow restart
            self.current_product_id = 0
            return
        
        product = self.product_data[index]
        product_msg = ProductInfo()
        product_msg.device_id = str(product.get('DeviceID', ''))
        product_msg.batch_id = str(product.get('BatchID', ''))
        product_msg.rohs_compliant = bool(product.get('RoHS_Compliant', False))
        product_msg.expected_serial_number_qr = str(product.get('Expected_SerialNumber_QR', ''))
        product_msg.expected_batch_ocr = str(product.get('Expected_Batch_OCR', product.get('BatchID', '')))
        product_msg.expected_serial_ocr = str(product.get('Expected_Serial_OCR', product.get('Expected_SerialNumber_QR', '')))
        self.info_publisher_.publish(product_msg)
        self.get_logger().info(f'Published info for product ID {product_id}')

    def shutdown_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Shutdown signal received. Terminating node.')
            self.destroy_node()
            
def main(args=None):
    rclpy.init(args=args)
    node = ProductSequencerNode()
    if node.product_data:
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()