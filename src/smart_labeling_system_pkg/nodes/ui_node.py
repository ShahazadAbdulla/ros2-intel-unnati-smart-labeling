# In nodes/ui_node.py

from tkinter import scrolledtext
import datetime
import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk, messagebox
from std_msgs.msg import String
from sensor_msgs.msg import Image as RosImage
from smart_labeling_interfaces.msg import ProductInfo, ProcessStatus, ValidationOutcome
from cv_bridge import CvBridge
from PIL import Image as PILImage, ImageTk
import cv2
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.parameter import Parameter
import os
import subprocess
import sys
from rcl_interfaces.srv import GetParameters
import sqlite3

class UINode(Node):
    def __init__(self, qos_profile):
        super().__init__('ui_node')
        self.app = None
        self.bridge = CvBridge()
        self.db_path = ""
        
        self.command_publisher_ = self.create_publisher(String, 'system_command', qos_profile)
        self.product_info_subscriber_ = self.create_subscription(ProductInfo, 'product_info', self.product_info_callback, qos_profile)
        self.label_image_subscriber_ = self.create_subscription(RosImage, 'label_image', self.label_image_callback, qos_profile)
        self.status_subscriber_ = self.create_subscription(ProcessStatus, 'process_status', self.process_status_callback, qos_profile)
        self.outcome_subscriber_ = self.create_subscription(ValidationOutcome, 'validation_outcome', self.validation_outcome_callback, qos_profile)

    def product_info_callback(self, msg):
        if self.app: self.app.update_product_info(msg)
    
    def label_image_callback(self, msg):
        if self.app:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                self.app.update_image(cv_image)
            except Exception as e: self.get_logger().error(f"Image display error: {e}")

    def validation_outcome_callback(self, msg: ValidationOutcome):
        if self.app:
            log_text = f"Product: {msg.device_id} | Decision: {msg.overall_decision} | Reason: {msg.failure_reason}"
            self.app.add_log_message(log_text)
            final_color = self.app._get_status_color(msg.overall_decision)
            self.app.overall_status_label.config(text=msg.overall_decision, foreground=final_color)

    def process_status_callback(self, msg):
        if self.app: self.app.update_overall_status(msg)

class App(tk.Tk):
    def __init__(self, ros_node: UINode):
        super().__init__()
        self.ros_node = ros_node
        self.tk_image = None
        self._setup_styles_and_widgets()
        self.protocol("WM_DELETE_WINDOW", self.exit_system)
        # Timer to ask for the database path parameter after the logger node has started
        self.after(2000, self.get_database_path)
        self.update_ros()

    def update_ros(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0.01)
        self.after(50, self.update_ros)
        
    def get_database_path(self):
        """Asynchronously get the database path from the logger node's parameter."""
        # RIGHT
        client = self.ros_node.create_client(GetParameters, '/database_logger/get_parameters')
        if not client.wait_for_service(timeout_sec=1.0):
            self.ros_node.get_logger().warn("Database logger parameter service not available, will retry.")
            self.after(2000, self.get_database_path) # Retry
            return
            
        # RIGHT
        request = GetParameters.Request()
        request.names = ['database_path']
        future = client.call_async(request)
        future.add_done_callback(self.database_path_callback)

    def database_path_callback(self, future):
        try:
            result = future.result()
            if result.values:
                self.ros_node.db_path = result.values[0].string_value
                self.ros_node.get_logger().info(f"UI successfully retrieved database path: {self.ros_node.db_path}")
                self.view_log_button.config(state=tk.NORMAL) # Enable the button
            else:
                self.ros_node.get_logger().warn("Could not retrieve database_path parameter.")
        except Exception as e:
            self.ros_node.get_logger().error(f"Service call to get database path failed: {e}")

    def open_database(self):
        """
        Callback for the 'View Log' button.
        Opens a NEW Tkinter window, reads the SQLite database, and displays
        the contents in a scrollable table.
        """
        db_path = self.ros_node.db_path
        if not db_path or not os.path.exists(db_path):
            messagebox.showerror("Error", "Database file not found. Run a process first.")
            return

        # --- Create the new Toplevel window for the report ---
        report_window = tk.Toplevel(self)
        report_window.title("Traceability Report")
        report_window.geometry("1200x600")
        report_window.configure(bg='#2E2E2E')

        # --- Create a Treeview widget for the table ---
        # Define columns based on your database schema
        columns = ('Timestamp', 'DeviceID', 'BatchID', 'OverallStatus', 'ComplianceStatus', 
                'ImageQualityStatus', 'MLQualityStatus', 'QR_Status', 'OCR_Status', 'ActionDetails')
        
        tree = ttk.Treeview(report_window, columns=columns, show='headings')

        # Define headings and column widths
        for col in columns:
            tree.heading(col, text=col)
            tree.column(col, width=120, anchor=tk.W) # Adjust width as needed

        # Add scrollbars
        vsb = ttk.Scrollbar(report_window, orient="vertical", command=tree.yview)
        vsb.pack(side='right', fill='y')
        hsb = ttk.Scrollbar(report_window, orient="horizontal", command=tree.xview)
        hsb.pack(side='bottom', fill='x')
        tree.configure(yscrollcommand=vsb.set, xscrollcommand=hsb.set)
        
        tree.pack(fill=tk.BOTH, expand=True)

        # --- Read data from SQLite and populate the Treeview ---
        try:
            conn = sqlite3.connect(db_path)
            cursor = conn.cursor()
            # Fetch all records from the table
            cursor.execute("SELECT * FROM validation_results ORDER BY id DESC") # Show newest first
            rows = cursor.fetchall()
            
            # Clear existing items before inserting new ones
            for item in tree.get_children():
                tree.delete(item)

            # Insert data into the treeview
            for row in rows:
                # We skip the 'id' column which is the first one (row[0])
                tree.insert("", tk.END, values=row[1:])

            conn.close()
        except Exception as e:
            messagebox.showerror("Database Error", f"Could not read from the database.\n\nError: {e}", parent=report_window)

    def _get_status_color(self, status_text):
        if not status_text: return "white"
        status_text = status_text.upper()
        if "PASS" in status_text or "ACCEPT" in status_text: return "lightgreen"
        elif "FAIL" in status_text or "REJECT" in status_text: return "red"
        else: return "white"

    def _reset_validation_labels(self):
        for label in self.validation_labels.values():
            label.config(text="PENDING", foreground="orange")

    def _setup_styles_and_widgets(self):
        self.title("Intel Unnati - Smart Labeling System")
        self.geometry("800x800")
        self.configure(bg='#2E2E2E')
        
        self.style = ttk.Style(self)
        self.style.theme_use('clam')
        self.style.configure('TFrame', background='#2E2E2E'); self.style.configure('TLabel', background='#2E2E2E', foreground='white', font=('Helvetica', 10)); self.style.configure('TButton', font=('Helvetica', 11, 'bold'), borderwidth='4', padding=10); self.style.map('TButton', foreground=[('active', 'white')], background=[('active', '#4a4a4a')]); self.style.configure('TLabelFrame', background='#2E2E2E', foreground='white', relief=tk.GROOVE); self.style.configure('TLabelFrame.Label', background='#2E2E2E', foreground='#00BFFF', font=('Helvetica', 12, 'bold'))
        
        main_container = ttk.Frame(self, padding=10); main_container.pack(fill=tk.BOTH, expand=True)
        
        control_frame = ttk.LabelFrame(main_container, text="System Control", padding=10)
        control_frame.pack(pady=5, padx=10, fill=tk.X)
        self.start_button = ttk.Button(control_frame, text="START", width=15, command=self.start_system)
        self.start_button.pack(side=tk.LEFT, padx=5, expand=True)
        # <<< NEW: View Log Button >>>
        self.view_log_button = ttk.Button(control_frame, text="TRACEABILITY REPORT", width=25, command=self.open_database, state=tk.DISABLED)
        self.view_log_button.pack(side=tk.LEFT, padx=5, expand=True)
        self.exit_button = ttk.Button(control_frame, text="EXIT", width=15, command=self.exit_system)
        self.exit_button.pack(side=tk.RIGHT, padx=5, expand=True)

        # ... (rest of the widgets are the same)
        product_info_frame = ttk.LabelFrame(main_container, text="Current Product", padding=10); product_info_frame.pack(pady=5, padx=10, fill=tk.X)
        self.product_id_label = ttk.Label(product_info_frame, text="Product ID: ---", font=('Helvetica', 14, 'bold')); self.product_id_label.pack()
        self.batch_id_label = ttk.Label(product_info_frame, text="Batch ID: ---", font=('Helvetica', 12)); self.batch_id_label.pack()
        content_frame = ttk.Frame(main_container); content_frame.pack(pady=5, padx=10, fill=tk.BOTH, expand=True)
        image_frame = ttk.LabelFrame(content_frame, text="Label Image", padding=10); image_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        self.image_label = ttk.Label(image_frame, text="\n\nLabel Image Appears Here\n\n", anchor=tk.CENTER); self.image_label.pack(fill=tk.BOTH, expand=True)
        validation_frame = ttk.LabelFrame(content_frame, text="Validation Checks", padding=10); validation_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(5, 0))
        self.validation_labels = {"RoHS": self.create_validation_row(validation_frame, "RoHS Compliance"),"ImgQuality": self.create_validation_row(validation_frame, "Image Quality"),"MLQuality": self.create_validation_row(validation_frame, "ML Print Quality"),"QR": self.create_validation_row(validation_frame, "QR Code"),"OCR": self.create_validation_row(validation_frame, "OCR Text"),}
        status_frame = ttk.LabelFrame(main_container, text="Overall Status", padding=10); status_frame.pack(pady=5, padx=10, fill=tk.X)
        self.overall_status_label = ttk.Label(status_frame, text="IDLE", font=('Helvetica', 18, 'bold'), foreground='lightblue', anchor=tk.CENTER); self.overall_status_label.pack(fill=tk.X, expand=True)
        log_frame = ttk.LabelFrame(main_container, text="Event Log", padding=10); log_frame.pack(pady=5, padx=10, fill=tk.BOTH, expand=True)
        self.log_text = scrolledtext.ScrolledText(log_frame, height=8, bg="#1C1C1C", fg="cyan", font=("Consolas", 9), wrap=tk.WORD, state=tk.DISABLED); self.log_text.pack(fill=tk.BOTH, expand=True)
        
        self.reset_ui_to_idle()

    def create_validation_row(self, parent, text):
        row_frame = ttk.Frame(parent); row_frame.pack(fill=tk.X, pady=2)
        name_label = ttk.Label(row_frame, text=f"{text}:", width=18, anchor=tk.W); name_label.pack(side=tk.LEFT)
        status_label = ttk.Label(row_frame, text="N/A", font=('Courier', 10, 'bold'), width=10, anchor=tk.W); status_label.pack(side=tk.LEFT, padx=5)
        return status_label

    def start_system(self):
        self.ros_node.command_publisher_.publish(String(data="START"))
        self.start_button.config(state=tk.DISABLED)

    def exit_system(self):
        self.ros_node.command_publisher_.publish(String(data="EXIT"))
        self.destroy()

    def add_log_message(self, log_msg: str):
        if not self.log_text.winfo_exists(): return
        self.log_text.config(state=tk.NORMAL)
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        self.log_text.insert('1.0', f"[{timestamp}] {log_msg}\n")
        self.log_text.config(state=tk.DISABLED)

    def update_product_info(self, msg: ProductInfo):
        self.product_id_label.config(text=f"Product ID: {msg.device_id}")
        self.batch_id_label.config(text=f"Batch ID: {msg.batch_id}")
        self.image_label.config(image=None, text="\n\nLoading Image...\n\n"); self.tk_image = None
        self.overall_status_label.config(text="PROCESSING...", foreground="orange")
        self._reset_validation_labels()

    def update_image(self, cv_image):
        pil_img = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)); pil_img.thumbnail((400, 400), PILImage.LANCZOS)
        self.tk_image = ImageTk.PhotoImage(image=pil_img); self.image_label.config(image=self.tk_image, text="")

    def update_overall_status(self, msg: ProcessStatus):
        status_map = {'rohs_status':'RoHS', 'image_quality_status':'ImgQuality', 'ml_quality_status':'MLQuality', 'qr_status':'QR', 'ocr_status':'OCR'}
        is_initial = not any(getattr(msg, f) for f in status_map.keys())
        if is_initial: self._reset_validation_labels()
        for field, key in status_map.items():
            val = getattr(msg, field)
            if val: self.validation_labels[key].config(text=val.upper(), foreground=self._get_status_color(val))
        if msg.status_text:
            self.overall_status_label.config(text=msg.status_text.upper(), foreground=self._get_status_color(msg.status_text))

    def reset_ui_to_idle(self):
        self.start_button.config(state=tk.NORMAL)
        self.product_id_label.config(text="Product ID: ---"); self.batch_id_label.config(text="Batch ID: ---")
        self.image_label.config(image=None, text="\n\nLabel Image Appears Here\n\n"); self.tk_image = None
        self.overall_status_label.config(text="IDLE", foreground="lightblue")
        self._reset_validation_labels()

def main(args=None):
    rclpy.init(args=args)
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)
    node = UINode(qos_profile=qos)
    app = App(node)
    node.app = app
    try:
        app.mainloop()
    except KeyboardInterrupt: pass
    finally:
        node.get_logger().info('UI mainloop finished. Shutting down.')
        if rclpy.ok() and node.handle is not None: node.destroy_node()

if __name__ == '__main__':
    main()