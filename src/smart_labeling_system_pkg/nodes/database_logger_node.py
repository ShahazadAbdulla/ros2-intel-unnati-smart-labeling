# In nodes/database_logger_node.py

import rclpy
from rclpy.node import Node
import sqlite3
import os
from ament_index_python.packages import get_package_share_directory
from datetime import datetime
import threading

from smart_labeling_interfaces.msg import ValidationOutcome, ProcessStatus
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy

class DatabaseLoggerNode(Node):
    def __init__(self):
        super().__init__('database_logger_node')
        
        self.qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)
        self.product_status_cache = {}
        self.lock = threading.Lock()
        
        # --- Database Setup ---
        # Define the default path within the package's install share space.
        default_db_path = os.path.join(
            get_package_share_directory('smart_labeling_system_pkg'), 
            'database', 
            'traceability.db'
        )
        
        # <<< CRITICAL: Declare the path as a parameter so other nodes can find it >>>
        self.declare_parameter('database_path', default_db_path)
        self.db_path = self.get_parameter('database_path').get_parameter_value().string_value

        db_folder_path = os.path.dirname(self.db_path)
        os.makedirs(db_folder_path, exist_ok=True)
        
        self.get_logger().info(f"Database will be stored at: {self.db_path}")
        self.conn = self._create_database_connection()
        if self.conn:
            self._create_table()
            self._log_startup_event()

        # --- Subscribers ---
        self.status_subscriber = self.create_subscription(
            ProcessStatus, 'process_status', self.status_callback, self.qos_profile)
        self.outcome_subscriber = self.create_subscription(
            ValidationOutcome, 'validation_outcome', self.outcome_callback, self.qos_profile)
        self.shutdown_subscriber = self.create_subscription(
            Bool, 'system_shutdown', self.shutdown_callback, self.qos_profile)

        self.get_logger().info('Rich Database Logger Node has been started.')

    def _create_database_connection(self):
        try:
            conn = sqlite3.connect(self.db_path, check_same_thread=False)
            self.get_logger().info("Successfully connected to database.")
            return conn
        except sqlite3.Error as e:
            self.get_logger().error(f"Database connection error: {e}")
            return None

    def _create_table(self):
        create_table_sql = """
        CREATE TABLE IF NOT EXISTS validation_results (
            id INTEGER PRIMARY KEY AUTOINCREMENT, Timestamp TEXT NOT NULL, DeviceID TEXT NOT NULL,
            BatchID TEXT, OverallStatus TEXT NOT NULL, ComplianceStatus TEXT,
            ImageQualityStatus TEXT, MLQualityStatus TEXT, QR_Status TEXT,
            OCR_Status TEXT, ActionDetails TEXT
        );"""
        self._execute_sql(create_table_sql)
        self.get_logger().info("Detailed 'validation_results' table is ready.")

    def _log_startup_event(self):
        sql = ''' INSERT INTO validation_results(
                    Timestamp, DeviceID, BatchID, OverallStatus, ComplianceStatus, 
                    ImageQualityStatus, MLQualityStatus, QR_Status, OCR_Status, ActionDetails
                  ) VALUES(?,?,?,?,?,?,?,?,?,?) '''
        timestamp_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        record = (timestamp_str, "SYSTEM", "N/A", "STARTUP", "---", "---", "---", "---", "---", "New Process Run Started")
        self._execute_sql(sql, record)

    def status_callback(self, msg: ProcessStatus):
        with self.lock:
            self.product_status_cache[msg.product_id] = msg

    def outcome_callback(self, msg: ValidationOutcome):
        with self.lock:
            product_id = msg.device_id
            cached_status = self.product_status_cache.get(product_id)
            
            if cached_status is None:
                self.get_logger().warn(f"Received outcome for {product_id}, but no status was cached. Logging minimal info.")
                cached_status = ProcessStatus()

            sql = ''' INSERT INTO validation_results(
                        Timestamp, DeviceID, BatchID, OverallStatus, ComplianceStatus, 
                        ImageQualityStatus, MLQualityStatus, QR_Status, OCR_Status, ActionDetails
                      ) VALUES(?,?,?,?,?,?,?,?,?,?) '''
            
            timestamp_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            record = (
                timestamp_str, product_id, cached_status.batch_id,
                msg.overall_decision, cached_status.rohs_status,
                cached_status.image_quality_status, cached_status.ml_quality_status,
                cached_status.qr_status, cached_status.ocr_status, msg.failure_reason
            )
            self._execute_sql(sql, record)
            
            if product_id in self.product_status_cache:
                del self.product_status_cache[product_id]

    def _execute_sql(self, sql, params=()):
        if self.conn is None: return
        try:
            cursor = self.conn.cursor()
            cursor.execute(sql, params)
            self.conn.commit()
            if "INSERT" in sql.upper():
                 self.get_logger().info(f"Successfully logged result for DeviceID: {params[1] if len(params) > 1 else 'SYSTEM'}")
        except sqlite3.Error as e:
            self.get_logger().error(f"Failed to execute SQL: {e}")

    def shutdown_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Shutdown signal received. Closing database connection and terminating.')
            if self.conn: self.conn.close()
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DatabaseLoggerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally:
        if hasattr(node, 'conn') and node.conn: node.conn.close()
        if rclpy.ok() and node.handle is not None: node.destroy_node()

if __name__ == '__main__':
    main()