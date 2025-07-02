import rclpy
from rclpy.node import Node
import os
import cv2
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Bool

# Import the message types we need
from smart_labeling_interfaces.msg import ProductInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Int32  # <<< NEW: Import the trigger message type

from rclpy.qos import QoSProfile, DurabilityPolicy

class LabelImageNode(Node):
    def __init__(self):
        super().__init__('label_image_node')

        self.qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)
        
        # <<< NEW: Declare a parameter for the image folder name >>>
        # The default value is what we've been using for our 6-product demo.
        self.declare_parameter('label_images_folder', 'label_images_test')
        
        # --- Class Members ---
        self.bridge = CvBridge()
        self.pkg_share_path = get_package_share_directory('smart_labeling_system_pkg')
        self.current_product_info = None

        # --- Publishers ---
        self.image_publisher_ = self.create_publisher(Image, 'label_image', self.qos_profile)

        # --- Subscribers ---
        self.product_info_subscriber_ = self.create_subscription(
            ProductInfo, 'product_info', self.product_info_callback, self.qos_profile)
        self.inspection_ready_subscriber_ = self.create_subscription(
            Int32, 'product_at_inspection', self.inspection_ready_callback, self.qos_profile)
        self.shutdown_subscriber = self.create_subscription(
            Bool, 'system_shutdown', self.shutdown_callback, self.qos_profile)

        self.get_logger().info('Label Image Node has been started and is waiting for triggers.')
    
    def product_info_callback(self, msg: ProductInfo):
        self.get_logger().info(f"Received and cached info for upcoming product: '{msg.device_id}'")
        self.current_product_info = msg

    def inspection_ready_callback(self, msg: Int32):
        self.get_logger().info(f"Inspection trigger received. Loading image for current product.")
        if self.current_product_info is None:
            self.get_logger().warn("Inspection triggered, but no product info has been cached. Skipping.")
            return

        image_path = self.get_label_image_path(self.current_product_info)
        self.get_logger().info(f"Attempting to load image from: {image_path}")

        if not os.path.exists(image_path):
            self.get_logger().error(f"Image file not found: {image_path}")
            # Still clear the info so we don't try to use it again for the wrong product
            self.current_product_info = None
            return
            
        cv_image = cv2.imread(image_path)
        if cv_image is None:
            self.get_logger().error(f"Failed to load image with OpenCV from path: {image_path}")
            self.current_product_info = None
            return
        
        try:
            ros_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            ros_image_msg.header.stamp = self.get_clock().now().to_msg()
            ros_image_msg.header.frame_id = 'camera_frame'
            self.image_publisher_.publish(ros_image_msg)
            self.get_logger().info(f"Successfully published image for '{self.current_product_info.device_id}'")
        except Exception as e:
            self.get_logger().error(f"Error with CvBridge: {e}")
        
        self.current_product_info = None

    def get_label_image_path(self, product_msg: ProductInfo):
        """
        Determines the file path for a product's label image based on its info.
        """
        # <<< NEW: Get the folder name from the parameter >>>
        image_folder_name = self.get_parameter('label_images_folder').get_parameter_value().string_value
        base_folder = os.path.join(self.pkg_share_path, image_folder_name)
        
        expected_serial_qr = product_msg.expected_serial_number_qr

        # Logic to handle special test cases remains the same
        if expected_serial_qr == 'SN004_MISMATCH':
            image_filename = 'SN004.png'
        elif expected_serial_qr == 'SN_FAIL_BLURRY_EXPECTED':
            image_filename = 'SN_FAIL_BLURRY.png' # Check for typo here in your actual filename
        else:
            image_filename = f"{expected_serial_qr}.png"
        
        return os.path.join(base_folder, image_filename)

    def shutdown_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Shutdown signal received. Terminating node.')
            self.destroy_node()
def main(args=None):
    rclpy.init(args=args)
    node = LabelImageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()