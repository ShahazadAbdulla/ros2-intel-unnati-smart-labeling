from setuptools import setup
import os
from glob import glob

package_name = 'smart_labeling_system_pkg'

setup(
    name=package_name,
    version='0.0.1',
    # Explicitly tell setuptools what our Python packages are
    # and where to find them.
    packages=['nodes'],
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # IMPORTANT: Install the package.xml
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'label_images_test'), glob('label_images_test/*')),
        (os.path.join('share', package_name, 'label_images'), glob('label_images/*')),
        (os.path.join('share', package_name, 'saved_models'), glob('saved_models/*')),
    ],
    install_requires=['setuptools', 'pandas', 'opencv-python', 'pybullet', 'pillow', 'easyocr', 'pyzbar'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@todo.com',
    description='Smart Product Labeling and Traceability System for Intel Unnati',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # The path to main() is now nodes.module:function
            'conductor_node = nodes.conductor_node:main',
            'product_sequencer_node = nodes.product_sequencer_node:main',
            'label_image_node = nodes.label_image_node:main',
            'pybullet_visualizer_node = nodes.pybullet_visualizer_node:main',
            'ui_node = nodes.ui_node:main',
            'ai_validation_node = nodes.ai_validation_node:main',         
            'label_quality_ml_node = nodes.label_quality_ml_node:main',
            'dummy_action_completer_node = nodes.dummy_action_completer_node:main',
            'database_logger_node = nodes.database_logger_node:main',
        ],
    },
)