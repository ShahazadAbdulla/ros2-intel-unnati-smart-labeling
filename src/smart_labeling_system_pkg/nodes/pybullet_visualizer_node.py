# In nodes/pybullet_visualizer_node.py

import rclpy
from rclpy.node import Node
import pybullet as p
import pybullet_data
import math

# Import message types
from smart_labeling_interfaces.msg import ValidationOutcome
from std_msgs.msg import Bool, Int32
from rclpy.qos import QoSProfile, DurabilityPolicy

class PybulletVisualizerNode(Node):
    def __init__(self):
        super().__init__('pybullet_visualizer_node')
        
        self.qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)
        
        self.get_logger().info("Connecting to PyBullet GUI...")
        try:
            self.physics_client = p.connect(p.GUI)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0, 0, -9.81)
            self.plane_id = p.loadURDF("plane.urdf")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to PyBullet: {e}"); rclpy.shutdown(); return

        # --- Scene, Conveyor, and Product Setup ---
        self.product_half_extents = [0.05, 0.05, 0.05] 
        self.inspection_x_limit = 0.0 
        self.conveyor_half_extents = [1.5, 0.2, 0.05] 
        self.conveyor_height_above_ground = 0.5
        self.conveyor_end_x = self.conveyor_half_extents[0] - self.product_half_extents[0] - 0.05 

        # --- Setup calls ---
        self._setup_environment()
        self._spawn_products()
        self._setup_rejector()
        self._setup_visuals() # Add the decorative visuals
        
        p.resetDebugVisualizerCamera(cameraDistance=2.5, cameraYaw=50, cameraPitch=-30, cameraTargetPosition=[0.0, 0.0, self.conveyor_height_above_ground])
        
        # --- State variables ---
        self.product_speed = 0.2 
        self.simulation_time_step = 1./240.
        self.system_running = False
        self.product_at_inspection_idx = -1
        self.decision_for_inspected_product = None
        self.pusher_state = "RETRACTED"
        self.pusher_animation_speed = 0.3
        self.pusher_target_y_extended = self.pusher_initial_position[1] - (abs(-(self.conveyor_half_extents[1] + self.product_half_extents[1] * 2 + 0.25) - 0))
        self.pusher_retracted_y = self.pusher_initial_position[1]
        self.pusher_extended_state_timer = 0
        self.active_rejected_product_id = None

        # --- Publishers ---
        self.at_inspection_publisher_ = self.create_publisher(Int32, 'product_at_inspection', self.qos_profile)
        self.action_complete_publisher_ = self.create_publisher(Int32, 'pybullet_action_complete', self.qos_profile)
        
        # --- Subscribers ---
        self.system_command_subscriber_ = self.create_subscription(Bool, 'start_system', self.system_command_callback, self.qos_profile)
        self.decision_subscriber_ = self.create_subscription(ValidationOutcome, 'validation_outcome', self.decision_callback, self.qos_profile)
        self.shutdown_subscriber = self.create_subscription(Bool, 'system_shutdown', self.shutdown_callback, self.qos_profile)
        
        self.timer = self.create_timer(self.simulation_time_step, self.simulation_step)
        
        self.get_logger().info('PyBullet Visualizer Node has been started.')

    def system_command_callback(self, msg: Bool):
        self.system_running = msg.data
        if self.system_running:
            self.get_logger().info("START command received. Conveyor system is now active.")
        # No else needed as we've removed the PAUSE logic

    def decision_callback(self, msg: ValidationOutcome):
        if self.product_at_inspection_idx != -1:
            self.decision_for_inspected_product = msg.overall_decision.upper()
            self.get_logger().info(f"Received decision '{self.decision_for_inspected_product}' for product index {self.product_at_inspection_idx}.")

    def simulation_step(self):
        conveyor_is_running = self.system_running
        if self.product_at_inspection_idx != -1 and self.products_data[self.product_at_inspection_idx]["state"] == "AT_INSPECTION":
            conveyor_is_running = False
        if self.pusher_state != "RETRACTED":
            conveyor_is_running = False
            
        frontmost_product_idx = self._get_next_active_product()
        if frontmost_product_idx != -1 and abs(self.products_data[frontmost_product_idx]["current_x"] - self.conveyor_end_x) < 0.01:
            conveyor_is_running = False

        if conveyor_is_running:
            delta_x = self.product_speed * self.simulation_time_step
            for i in range(len(self.products_data)):
                prod_data = self.products_data[i]
                if prod_data["state"] == "ON_CONVEYOR":
                    new_x = prod_data["current_x"] + delta_x
                    candidate_idx = self._get_next_inspection_candidate()
                    if i == candidate_idx and new_x >= self.inspection_x_limit:
                        new_x = self.inspection_x_limit; prod_data["state"] = "AT_INSPECTION"; self.product_at_inspection_idx = i
                        msg = Int32(); msg.data = i + 1; self.at_inspection_publisher_.publish(msg)
                        self.get_logger().info(f"Product index {i} (ID {i+1}) at inspection.")
                    prod_data["current_x"] = new_x
                    p.resetBasePositionAndOrientation(prod_data["body_id"], [new_x, prod_data["current_y"], prod_data["current_z"]], prod_data["orientation"])
                elif prod_data["state"] == "ACCEPTED_TO_END":
                    new_x = prod_data["current_x"] + delta_x
                    if new_x >= self.conveyor_end_x:
                        new_x = self.conveyor_end_x; prod_data["state"] = "PROCESSED"
                    prod_data["current_x"] = new_x
                    p.resetBasePositionAndOrientation(prod_data["body_id"], [new_x, prod_data["current_y"], prod_data["current_z"]], prod_data["orientation"])

        if self.decision_for_inspected_product and self.product_at_inspection_idx != -1:
            inspected_idx = self.product_at_inspection_idx
            prod_inspect_data = self.products_data[inspected_idx]
            if self.decision_for_inspected_product == "ACCEPTED":
                prod_inspect_data["state"] = "ACCEPTED_TO_END"
                self._publish_action_complete(inspected_idx)
                self.product_at_inspection_idx = -1
            elif self.decision_for_inspected_product == "REJECTED":
                prod_inspect_data["state"] = "REJECTED_ANIMATING"
                self.active_rejected_product_id = prod_inspect_data["body_id"]
                self.pusher_state = "EXTENDING"
            self.decision_for_inspected_product = None
        
        self._animate_pusher(); p.stepSimulation()

    def _get_next_inspection_candidate(self):
        for i in range(len(self.products_data)):
            if self.products_data[i]["state"] == "ON_CONVEYOR": return i
        return -1

    def _get_next_active_product(self):
        for i in range(len(self.products_data)):
            if self.products_data[i]["state"] != "PROCESSED": return i
        return -1
        
    def _publish_action_complete(self, product_index):
        msg = Int32(); msg.data = product_index + 1
        self.action_complete_publisher_.publish(msg)
        self.get_logger().info(f"Published action complete for product {product_index + 1}.")

    def _animate_pusher(self):
        if self.pusher_state == "RETRACTED" or self.active_rejected_product_id is None: return
        pusher_pos, _ = p.getBasePositionAndOrientation(self.pusher_id)
        arm_pos, arm_orn = p.getBasePositionAndOrientation(self.arm_id)
        prod_data, prod_idx = None, -1
        for i, p_data in enumerate(self.products_data):
            if p_data["body_id"] == self.active_rejected_product_id:
                prod_data, prod_idx = p_data, i; break
        if not prod_data: self.pusher_state = "RETRACTED"; self.active_rejected_product_id = None; return

        if self.pusher_state == "EXTENDING":
            new_y = pusher_pos[1] - self.pusher_animation_speed * self.simulation_time_step
            if new_y <= self.pusher_target_y_extended: new_y = self.pusher_target_y_extended; self.pusher_state = "EXTENDED"
            p.resetBasePositionAndOrientation(self.pusher_id, [pusher_pos[0], new_y, pusher_pos[2]], p.getQuaternionFromEuler([0,0,0]))
            p.resetBasePositionAndOrientation(self.arm_id, [arm_pos[0], new_y + self.initial_arm_y_offset_from_pusher, arm_pos[2]], arm_orn)
            if (new_y - self.pusher_half_extents[1]) <= (prod_data["current_y"] + self.product_half_extents[1]):
                prod_data["current_y"] = (new_y - self.pusher_half_extents[1]) - self.product_half_extents[1]
                p.resetBasePositionAndOrientation(prod_data["body_id"], [prod_data["current_x"], prod_data["current_y"], prod_data["current_z"]], prod_data["orientation"])
        elif self.pusher_state == "EXTENDED":
            self.pusher_extended_state_timer += 1
            if self.pusher_extended_state_timer >= int(240 * 0.5): self.pusher_state = "RETRACTING"; self.pusher_extended_state_timer = 0
        elif self.pusher_state == "RETRACTING":
            new_y = pusher_pos[1] + self.pusher_animation_speed * self.simulation_time_step
            if new_y >= self.pusher_retracted_y:
                new_y, self.pusher_state = self.pusher_retracted_y, "RETRACTED"
                self.get_logger().info(f"Pusher retracted. Product index {prod_idx} (ID {prod_idx+1}) REJECTED & processed.")
                prod_data["state"] = "PROCESSED"; self.active_rejected_product_id = None
                if self.product_at_inspection_idx == prod_idx: self.product_at_inspection_idx = -1
                self._publish_action_complete(prod_idx)
            p.resetBasePositionAndOrientation(self.pusher_id, [pusher_pos[0], new_y, pusher_pos[2]], p.getQuaternionFromEuler([0,0,0]))
            p.resetBasePositionAndOrientation(self.arm_id, [arm_pos[0], new_y + self.initial_arm_y_offset_from_pusher, arm_pos[2]], arm_orn)

    def _setup_environment(self):
        conveyor_pos = [0, 0, self.conveyor_height_above_ground + self.conveyor_half_extents[2]]
        vis_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=self.conveyor_half_extents, rgbaColor=[.5, .5, .5, 1])
        col_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=self.conveyor_half_extents)
        p.createMultiBody(0, col_shape, vis_shape, basePosition=conveyor_pos)
        leg_ext = [0.05, 0.05, self.conveyor_height_above_ground / 2.0]
        leg_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=leg_ext, rgbaColor=[0.4, 0.4, 0.4, 1])
        leg_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=leg_ext)
        leg_x_off = self.conveyor_half_extents[0] * 0.8; leg_z_pos = self.conveyor_height_above_ground / 2.0
        p.createMultiBody(0, leg_col, leg_vis, [-leg_x_off, 0, leg_z_pos])
        p.createMultiBody(0, leg_col, leg_vis, [leg_x_off, 0, leg_z_pos])

    def _spawn_products(self):
        self.products_data = []; mass = 0.2; num_prods = 6; spacing = 0.25; offset = 0.2
        colors = [[0.8,0.2,0.2,1],[0.2,0.8,0.2,1],[0.2,0.2,0.8,1],[0.8,0.8,0.2,1]]
        for i in range(num_prods):
            start_x = self.inspection_x_limit - offset - (i * spacing)
            start_pos = [start_x, 0, self.conveyor_height_above_ground + self.conveyor_half_extents[2]*2 + self.product_half_extents[2]]
            b_idx = 0
            if i < 1:
                b_idx = 0  # Product 1 -> Red
            elif i < 2:
                b_idx = 1  # Product 2 -> Green
            elif i < 4:
                b_idx = 2  # Product 3,4 -> Blue
            else:
                b_idx = 3  # Product 5,6 -> Yellow
            vis_id=p.createVisualShape(p.GEOM_BOX, halfExtents=self.product_half_extents, rgbaColor=colors[b_idx])
            col_id=p.createCollisionShape(p.GEOM_BOX, halfExtents=self.product_half_extents)
            body_id=p.createMultiBody(mass, col_id, vis_id, start_pos)
            self.products_data.append({"id":i,"body_id":body_id,"current_x":start_x,"current_y":0,"current_z":start_pos[2],"orientation":p.getQuaternionFromEuler([0,0,0]),"state":"ON_CONVEYOR"})
    
    def _setup_rejector(self):
        self.pusher_half_extents = [self.product_half_extents[0]*1.2, 0.025, self.product_half_extents[2]]
        self.pusher_initial_position = [self.inspection_x_limit, self.conveyor_half_extents[1] + self.pusher_half_extents[1] + 0.01, self.conveyor_height_above_ground + self.conveyor_half_extents[2]*2 + self.product_half_extents[2]]
        pusher_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=self.pusher_half_extents, rgbaColor=[0.5,0.5,0.5,1])
        pusher_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=self.pusher_half_extents)
        self.pusher_id = p.createMultiBody(0, pusher_col, pusher_vis, self.pusher_initial_position)
        arm_rad, arm_len = 0.02, 0.15
        arm_pos = [self.pusher_initial_position[0], self.pusher_initial_position[1] + self.pusher_half_extents[1] + (arm_len/2.0), self.pusher_initial_position[2]]
        arm_orn = p.getQuaternionFromEuler([math.pi/2,0,0])
        arm_vis = p.createVisualShape(p.GEOM_CYLINDER,radius=arm_rad,length=arm_len,rgbaColor=[0.3,0.3,0.3,1])
        arm_col = p.createCollisionShape(p.GEOM_CYLINDER,radius=arm_rad,height=arm_len)
        self.arm_id = p.createMultiBody(0, arm_col, arm_vis, arm_pos, arm_orn)
        self.initial_arm_y_offset_from_pusher = arm_pos[1] - self.pusher_initial_position[1]
        
    def _setup_visuals(self):
        """Creates the static visual-only elements like the sensor and camera."""
        self.get_logger().info("Setting up visual elements (sensor, camera)...")
        
        # Sensor Visual
        # <<< FIX: Use consistent variable names >>>
        sensor_radius, sensor_length, sensor_color = 0.03, 0.05, [0.1, 0.1, 0.5, 1]
        sensor_pos_x = self.inspection_x_limit
        sensor_pos_y = -(self.conveyor_half_extents[1] + sensor_radius + 0.25)
        sensor_pos_z = self.conveyor_height_above_ground + self.conveyor_half_extents[2] + sensor_length / 2.0
        sensor_orientation = p.getQuaternionFromEuler([math.pi / 2, 0, 0])
        sensor_visual_id = p.createVisualShape(
            p.GEOM_CYLINDER, 
            radius=sensor_radius, 
            length=sensor_length, 
            rgbaColor=sensor_color
        )
        p.createMultiBody(
            baseMass=0, 
            baseVisualShapeIndex=sensor_visual_id, 
            basePosition=[sensor_pos_x, sensor_pos_y, sensor_pos_z], 
            baseOrientation=sensor_orientation
        )
        
        # Camera Body Visual
        # <<< FIX: Use consistent variable names >>>
        cam_body_extents = [0.04, 0.06, 0.02]
        cam_body_color = [0.2, 0.2, 0.2, 1]
        cam_body_pos_x = self.inspection_x_limit
        cam_body_pos_y = 0.0
        cam_body_pos_z = self.conveyor_height_above_ground + self.conveyor_half_extents[2] + 0.25
        cam_body_visual_id = p.createVisualShape(
            p.GEOM_BOX, 
            halfExtents=cam_body_extents, 
            rgbaColor=cam_body_color
        )
        p.createMultiBody(
            baseMass=0, 
            baseVisualShapeIndex=cam_body_visual_id, 
            basePosition=[cam_body_pos_x, cam_body_pos_y, cam_body_pos_z]
        )
        
        # Camera Lens Visual
        # <<< FIX: Use consistent variable names >>>
        cam_lens_radius, cam_lens_length, cam_lens_color = 0.025, 0.04, [0.1, 0.1, 0.1, 1]
        cam_lens_pos_z = cam_body_pos_z - cam_body_extents[2] - (cam_lens_length / 2.0)
        cam_lens_visual_id = p.createVisualShape(
            p.GEOM_CYLINDER, 
            radius=cam_lens_radius, 
            length=cam_lens_length, 
            rgbaColor=cam_lens_color
        )
        p.createMultiBody(
            baseMass=0, 
            baseVisualShapeIndex=cam_lens_visual_id, 
            basePosition=[cam_body_pos_x, cam_body_pos_y, cam_lens_pos_z]
        )

    def destroy_node(self):
        self.get_logger().info("Disconnecting from PyBullet.")
        if hasattr(self, 'physics_client') and self.physics_client >= 0 and p.isConnected(self.physics_client):
            p.disconnect(self.physics_client)
        super().destroy_node()

    def shutdown_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Shutdown signal received. Terminating node.')
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PybulletVisualizerNode()
    if 'physics_client' in node.__dict__ and node.physics_client >= 0:
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
        finally:
            if rclpy.ok() and hasattr(node, 'handle') and node.handle is not None: node.destroy_node()
    else:
        node.get_logger().error("PyBullet node failed to initialize.")
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()