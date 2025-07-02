import rclpy
from rclpy.node import Node
import os, cv2, numpy as np
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, DurabilityPolicy

try:
    import tensorflow as tf
    from tensorflow.keras import layers, Model, Input
except ImportError:
    tf = None

from smart_labeling_interfaces.srv import PredictLabelQuality

try:
    import tensorflow as tf
    # Disable all GPUs for this process
    tf.config.set_visible_devices([], 'GPU')
    visible_devices = tf.config.get_visible_devices()
    for device in visible_devices:
        assert device.device_type != 'GPU'
    print("--- TensorFlow configured to use CPU only ---")
except Exception as e:
    print(f"Error setting TF device visibility: {e}")
    tf = None

def create_model_architecture(input_shape=(224, 224, 3)):
    """Recreates the model architecture exactly as in the Colab notebook."""
    inputs = Input(shape=input_shape, name="input_layer")
    # NOTE: We DO NOT add data augmentation or preprocessing layers here.
    # This will be done manually on the single image during inference.
    # This makes the deployed model simpler and faster.
    
    base_model = tf.keras.applications.MobileNetV2(
        input_shape=input_shape, include_top=False, weights='imagenet')
    base_model.trainable = False # Freeze the base model

    x = base_model(inputs, training=False)
    x = layers.GlobalAveragePooling2D(name="global_average_pooling")(x)
    x = layers.Dropout(0.2, name="dropout_layer")(x)
    outputs = layers.Dense(1, activation='sigmoid', name="output_layer")(x)
    
    model = Model(inputs=inputs, outputs=outputs)
    return model

class LabelQualityMLNode(Node):
    def __init__(self):
        super().__init__('label_quality_ml_node')
        if not tf: self.get_logger().error("TF not found."); return

        self.declare_parameter('weights_name', 'best_model_cautious_fine_tuned.weights.h5')
        self.declare_parameter('prediction_threshold', 0.43) 
        self.model = self._load_model_with_weights()
        self.bridge = CvBridge()
        
        if self.model:
            qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)
            self.srv = self.create_service(PredictLabelQuality, 'predict_label_quality', self.predict_callback)
            self.get_logger().info('ML Service is ready.')

    def _load_model_with_weights(self):
        pkg_share = get_package_share_directory('smart_labeling_system_pkg')
        weights_name = self.get_parameter('weights_name').get_parameter_value().string_value
        full_weights_path = os.path.join(pkg_share, 'saved_models', weights_name)
        
        if not os.path.exists(full_weights_path):
            self.get_logger().error(f"Weights file not found: {full_weights_path}")
            return None
        
        try:
            self.get_logger().info("Creating model architecture...")
            model = create_model_architecture()
            self.get_logger().info(f"Loading weights from {full_weights_path}...")
            model.load_weights(full_weights_path)
            self.get_logger().info("Weights loaded successfully.")
            model.predict(np.zeros((1, 224, 224, 3), dtype=np.float32), verbose=0)
            self.get_logger().info("Model initialized.")
            return model
        except Exception as e:
            self.get_logger().error(f"Failed to create model or load weights: {e}")
            return None

    def predict_callback(self, request, response):
        if not self.model: # Safety check
            response.quality_prediction, response.confidence = "ERROR", 0.0
            return response

        try:
            threshold = self.get_parameter('prediction_threshold').get_parameter_value().double_value
            cv_image = self.bridge.imgmsg_to_cv2(request.image, "bgr8")
            
            # --- PREPROCESSING to match MobileNetV2 ---
            img_resized = cv2.resize(cv_image, (224, 224))
            img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
            img_array = np.expand_dims(img_rgb, axis=0)
            # We now apply the specific MobileNetV2 preprocessing manually
            img_preprocessed = tf.keras.applications.mobilenet_v2.preprocess_input(img_array.astype(np.float32))

            predictions = self.model.predict(img_preprocessed, verbose=0)
            score = float(predictions[0][0])
            
            response.confidence = score
            response.quality_prediction = "GOOD" if score > threshold else "BAD"

        except Exception as e:
            self.get_logger().error(f"Prediction error: {e}")
            response.quality_prediction, response.confidence = "ERROR", 0.0

        return response
        
def main(args=None): # This needs to be outside the class
    rclpy.init(args=args)
    node = LabelQualityMLNode()
    if getattr(node, 'model', None): rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()