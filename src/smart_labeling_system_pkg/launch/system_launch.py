# In launch/system_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # --- Get Package Directory ---
    pkg_dir = get_package_share_directory('smart_labeling_system_pkg')

    # --- Declare All Launch Arguments ---
    
    # 1. Simulation Mode Argument
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='True',
        description='If True, run with PyBullet simulation. If False, run in headless (dummy) mode.'
    )
    
    # 2. CSV File Argument
    csv_file_arg = DeclareLaunchArgument(
        'csv_file',
        default_value='products_test.csv',
        description='Name of the product CSV file in the config directory.'
    )
    
    # 3. Label Images Folder Argument
    images_folder_arg = DeclareLaunchArgument(
        'images_folder',
        default_value='label_images_test',
        description='Name of the folder containing label images within the package.'
    )

    # 4. ML Model Threshold Argument
    ml_threshold_arg = DeclareLaunchArgument(
        'ml_threshold',
        default_value='0.5',
        description='Confidence threshold for the ML model (0.0 to 1.0).'
    )

    # 5. Database Reset Argument
    reset_db_arg = DeclareLaunchArgument(
        'reset_db',
        default_value='True',
        description='If True, wipes the traceability database on startup.'
    )
    
    # --- Node Definitions ---

    # Common nodes that run in both modes
    conductor_node = Node(
        package='smart_labeling_system_pkg',
        executable='conductor_node',
        name='conductor',
        output='screen',
        emulate_tty=True
    )

    product_sequencer_node = Node(
        package='smart_labeling_system_pkg',
        executable='product_sequencer_node',
        name='product_sequencer',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'csv_file': LaunchConfiguration('csv_file')
        }]
    )

    label_image_node = Node(
        package='smart_labeling_system_pkg',
        executable='label_image_node',
        name='label_image_loader',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'label_images_folder': LaunchConfiguration('images_folder')
        }]
    )
    
    ai_validation_node = Node(
        package='smart_labeling_system_pkg',
        executable='ai_validation_node',
        name='ai_validator',
        output='screen',
        emulate_tty=True
    )
    
    label_quality_ml_node = Node(
        package='smart_labeling_system_pkg',
        executable='label_quality_ml_node',
        name='ml_quality_service',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'prediction_threshold': LaunchConfiguration('ml_threshold')
        }]
    )

    database_path = os.path.join(pkg_dir, 'database', 'traceability.db')
    database_logger_node = Node(
        package='smart_labeling_system_pkg',
        executable='database_logger_node',
        name='database_logger',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'database_path': database_path},
            {'reset_on_startup': LaunchConfiguration('reset_db')}
        ]
    )

    ui_node = Node(
        package='smart_labeling_system_pkg',
        executable='ui_node',
        name='ui_controller',
        output='screen',
        emulate_tty=True,
    )

    # --- Conditional Nodes (The Magic Happens Here) ---
    
    # This node will ONLY be launched if the 'sim_mode' argument is 'True'
    pybullet_visualizer_node = Node(
        package='smart_labeling_system_pkg',
        executable='pybullet_visualizer_node',
        name='pybullet_visualizer',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('sim_mode'))
    )

    # This node will ONLY be launched if the 'sim_mode' argument is 'False'
    dummy_action_completer_node = Node(
        package='smart_labeling_system_pkg',
        executable='dummy_action_completer_node',
        name='dummy_action_completer',
        output='screen',
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration('sim_mode'))
    )


    # --- Assemble the Launch Description ---
    
    return LaunchDescription([
        # Declare all arguments
        sim_mode_arg,
        csv_file_arg,
        images_folder_arg,
        ml_threshold_arg,
        reset_db_arg,

        # List all nodes
        conductor_node,
        product_sequencer_node,
        label_image_node,
        ai_validation_node,
        label_quality_ml_node,
        database_logger_node,
        ui_node,
        
        # Add the conditional nodes
        pybullet_visualizer_node,
        dummy_action_completer_node
    ])