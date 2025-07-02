import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
import cv2
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode
import easyocr
import re
import time
from std_msgs.msg import Bool

# Import all our custom message and service types
from smart_labeling_interfaces.msg import ProductInfo, ProcessStatus, ValidationOutcome
from smart_labeling_interfaces.srv import PredictLabelQuality
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

class AIValidationNode(Node):
    def __init__(self):
        super().__init__('ai_validation_node')

        self.qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)
        
        # --- ROS Setup ---
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)
        self.callback_group = ReentrantCallbackGroup()
        self.bridge = CvBridge()

        # State variables to hold incoming data
        self.current_product_info = None
        self.current_label_image = None
        
        # OCR Reader Initialization
        self.get_logger().info("Initializing EasyOCR reader...")
        try:
            # Forcing CPU to avoid potential CUDA conflicts in ROS 2 context
            self.easyocr_reader = easyocr.Reader(['en'], gpu=False)
            self.get_logger().info("EasyOCR reader initialized on CPU.")
        except Exception as e:
            self.easyocr_reader = None
            self.get_logger().error(f"Failed to initialize EasyOCR: {e}")

        # --- Publishers ---
        self.status_publisher_ = self.create_publisher(ProcessStatus, 'process_status', qos)
        self.outcome_publisher_ = self.create_publisher(ValidationOutcome, 'validation_outcome', qos)

        # --- Subscribers ---
        # This callback just stores the latest data
        self.product_info_sub_ = self.create_subscription(
            ProductInfo, 'product_info', self.product_info_callback, qos, callback_group=self.callback_group)
            
        # <<< THIS IS THE KEY FIX FOR THE ATTRIBUTEERROR >>>
        # This subscription is now the main trigger for validation
        self.label_image_sub_ = self.create_subscription(
            Image, 'label_image', self.label_image_callback_and_trigger, qos, callback_group=self.callback_group)
        
        self.shutdown_subscriber = self.create_subscription(
            Bool, 'system_shutdown', self.shutdown_callback, self.qos_profile)
            
        # --- Service Client ---
        self.ml_quality_client = self.create_client(
            PredictLabelQuality, 'predict_label_quality', callback_group=self.callback_group)
        while not self.ml_quality_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('ML quality service not available, waiting again...')
            
        self.get_logger().info("AI Validation Node is ready.")

    def product_info_callback(self, msg):
        self.current_product_info = msg
        self.get_logger().info(f"Received product info for: {msg.device_id}")

    def label_image_callback_and_trigger(self, msg):
        """
        Receives the label image and, if we have corresponding product info,
        triggers the entire validation pipeline.
        """
        self.get_logger().info("Received label image, this will trigger validation.")
        self.current_label_image = msg
        
        # The image is the last piece of data we need. Now we can run the pipeline.
        self.run_validation_pipeline()
    
    # In AIValidationNode class

    def run_validation_pipeline(self):
        """
        The main pipeline that runs all checks in sequence, publishing intermediate
        status updates for the UI and logger.
        """
        if not all([self.current_product_info, self.current_label_image]):
            self.get_logger().warn("Validation triggered, but missing data. Ignoring.")
            return

        # --- Initial Setup ---
        # Create a status message that we will build up as we go.
        status_msg = ProcessStatus()
        status_msg.product_id = self.current_product_info.device_id
        status_msg.batch_id = self.current_product_info.batch_id
        
        # Publish an initial "STARTING" status to clear the UI checklist
        status_msg.status_text = "Validation Started..."
        self.status_publisher_.publish(status_msg)
        time.sleep(0.3) # Add a small delay for demo visibility

        # --- Image Conversion (this can fail critically) ---
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.current_label_image, "bgr8")
        except Exception as e:
            self.finalize_validation("REJECTED", f"Image conversion error: {e}")
            return

        # --- Sequential Validation Checks ---
        # 1. RoHS Compliance Check
        is_compliant, compliance_reason = self.verify_product_compliance(self.current_product_info)
        status_msg.rohs_status = "PASS" if is_compliant else "FAIL"
        status_msg.status_text = "Checking RoHS..."
        self.status_publisher_.publish(status_msg)
        time.sleep(0.3)
        if not is_compliant:
            self.finalize_validation("REJECTED", compliance_reason)
            return

        # 2. Image Quality (Blur) Check
        is_clear, quality_reason = self.check_image_quality(cv_image)
        status_msg.image_quality_status = "PASS" if is_clear else "FAIL"
        status_msg.status_text = "Checking Image Quality..."
        self.status_publisher_.publish(status_msg)
        time.sleep(0.3)
        if not is_clear:
            self.finalize_validation("REJECTED", quality_reason)
            return

        # 3. ML Print Quality Check
        ml_passes, ml_reason = self.check_ml_quality()
        status_msg.ml_quality_status = "PASS" if ml_passes else "FAIL"
        status_msg.status_text = "Checking ML Quality..."
        self.status_publisher_.publish(status_msg)
        time.sleep(0.3)
        if not ml_passes:
            self.finalize_validation("REJECTED", ml_reason)
            return

        # 4. QR Code Validation
        qr_passes, qr_reason = self.verify_qr_code(cv_image, self.current_product_info.expected_serial_number_qr)
        status_msg.qr_status = "PASS" if qr_passes else "FAIL"
        status_msg.status_text = "Checking QR Code..."
        self.status_publisher_.publish(status_msg)
        time.sleep(0.3)
        if not qr_passes:
            self.finalize_validation("REJECTED", qr_reason)
            return

        # 5. OCR Validation
        ocr_passes, ocr_reason = self.verify_ocr_text(cv_image, self.current_product_info)
        status_msg.ocr_status = "PASS" if ocr_passes else "FAIL"
        status_msg.status_text = "Checking OCR..."
        self.status_publisher_.publish(status_msg)
        time.sleep(0.3)
        if not ocr_passes:
            self.finalize_validation("REJECTED", ocr_reason)
            return
            
        # --- If all checks passed ---
        self.finalize_validation("ACCEPTED", "All validation checks passed.")

    def finalize_validation(self, decision, reason):
        """Publishes the final outcome and resets state."""
        self.get_logger().info(f"--- Validation Complete for {self.current_product_info.device_id} ---")
        self.get_logger().info(f"  Final Decision: {decision}")
        self.get_logger().info(f"  Reason: {reason}")
        
        outcome_msg = ValidationOutcome()
        outcome_msg.device_id = self.current_product_info.device_id
        outcome_msg.overall_decision = decision
        outcome_msg.failure_reason = reason
        self.outcome_publisher_.publish(outcome_msg)
        
        # Reset state for the next product
        self.current_product_info = None
        self.current_label_image = None
        self.inspection_ready = False

    # --- Verification Logic (from your main.py) ---
    def verify_product_compliance(self, product_data):
        if not product_data.rohs_compliant:
            return False, f"RoHS Non-Compliant"
        return True, "RoHS Compliant"

    def check_image_quality(self, cv_image, threshold=100.0):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        variance = cv2.Laplacian(gray, cv2.CV_64F).var()
        if variance < threshold:
            return False, f"Image too blurry (Variance: {variance:.2f})"
        return True, f"Image quality OK (Variance: {variance:.2f})"

    def check_ml_quality(self):
        """Calls the external ML service and waits for the result."""
        self.get_logger().info("Requesting ML print quality prediction...")
        request = PredictLabelQuality.Request()
        request.image = self.current_label_image
        
        # Asynchronous call
        future = self.ml_quality_client.call_async(request)
        
        # This is a synchronous block for simplicity. In a more complex node,
        # you would use `add_done_callback` to not block the executor.
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"ML Service Response: {response.quality_prediction} (Conf: {response.confidence:.2f})")
            if response.quality_prediction.upper() == "GOOD":
                return True, "ML quality check passed."
            else:
                return False, f"ML quality check failed (Prediction: {response.quality_prediction})"
        else:
            self.get_logger().error('Exception while calling ML service')
            return False, "Failed to get response from ML service."
            
    def verify_qr_code(self, cv_image, expected_qr):
        decoded_objects = decode(cv_image)
        if not decoded_objects:
            return False, "QR code not found"
        
        qr_data = decoded_objects[0].data.decode('utf-8')
        if qr_data.strip().upper() == expected_qr.strip().upper():
            return True, f"QR Match ({qr_data})"
        else:
            return False, f"QR Mismatch (Got: {qr_data}, Exp: {expected_qr})"

    # In class AIValidationNode:

    def verify_ocr_text(self, cv_image, product_data):
        if not self.easyocr_reader:
            return False, "OCR-FAIL: Reader not initialized"
        
        try:
            # Note: detail=0 makes it return a simple list of strings
            ocr_texts = self.easyocr_reader.readtext(cv_image, detail=0)
            if not ocr_texts:
                return False, "OCR-FAIL: No text detected on label"
        except Exception as e:
            self.get_logger().error(f"An exception occurred during EasyOCR readtext: {e}")
            return False, "OCR-FAIL: Exception during text recognition"
            
        self.get_logger().info(f"OCR raw detected texts: {ocr_texts}")

        extracted_info = self.extract_specific_ocr_info(ocr_texts)
        self.get_logger().info(f"OCR extracted (cleaned) info: {extracted_info}")
        
        # Get expected values from the product data message
        expected_batch = product_data.expected_batch_ocr.upper().strip()
        expected_serial = product_data.expected_serial_ocr.upper().strip()

        # Perform the checks
        batch_ok = (extracted_info['batch'] is not None and 
                    extracted_info['batch'] == expected_batch)
                    
        serial_ok = (extracted_info['serial'] is not None and 
                     extracted_info['serial'] == expected_serial)
        
        if batch_ok and serial_ok:
            return True, "OCR Match: Batch and Serial OK"
        else:
            # Build a detailed failure reason
            reasons = []
            if not batch_ok:
                reasons.append(f"Batch MISMATCH (Exp: {expected_batch}, Got: {extracted_info['batch']})")
            if not serial_ok:
                reasons.append(f"Serial MISMATCH (Exp: {expected_serial}, Got: {extracted_info['serial']})")
            
            return False, f"OCR-FAIL: {'; '.join(reasons)}"

    # In class AIValidationNode:

    def extract_specific_ocr_info(self, ocr_text_list):
        """
        Extracts Batch and Serial info from OCR text, applying corrections
        for common OCR errors. This logic is ported from the robust standalone script.
        """
        extracted_info = {'batch': None, 'serial': None}
        
        # Define common OCR misinterpretations
        corrections = {'O': '0', 'I': '1', 'L': '1', 'Z': '2', 'B': '8', 'S': '5'}

        def clean_text(text):
            for char, replacement in corrections.items():
                text = text.replace(char, replacement)
            return text

        for text_element in ocr_text_list:
            text_upper = str(text_element).upper()

            # Regex for Batch ID (e.g., BATCH: Bxxxx, B xxxx, Bxxxx)
            # Looks for a "B" followed by at least 3 alphanumeric characters.
            batch_match = re.search(r'(?:BATCH\s*[:\s]*)?(B[0-9A-Z]{3,})', text_upper)
            if batch_match and not extracted_info['batch']:
                raw_text = batch_match.group(1)
                # Clean the alphanumeric part, keeping the 'B' prefix
                prefix = raw_text[0]
                alphanum_part = raw_text[1:]
                extracted_info['batch'] = prefix + clean_text(alphanum_part)

            # Regex for Serial Number (e.g., S/N: SNxxxx, SIN:SNxxxx, SNxxxx)
            # Looks for "SN" followed by at least 3 alphanumeric characters.
            serial_match = re.search(r'(?:S[/\\]N\s*[:\s]*|SIN\s*[:\s]*)?(SN[0-9A-Z]{3,})', text_upper)
            if serial_match and not extracted_info['serial']:
                raw_text = serial_match.group(1)
                # Clean the alphanumeric part, keeping the 'SN' prefix
                prefix = raw_text[:2]
                alphanum_part = raw_text[2:]
                extracted_info['serial'] = prefix + clean_text(alphanum_part)

        return extracted_info
    
    def shutdown_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Shutdown signal received. Terminating node.')
            # Calling destroy_node() will cause rclpy.spin() to exit gracefully.
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AIValidationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()