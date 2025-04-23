from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arm_delivery_project'

setup(
    name=package_name,
    version='1.0.0',  # Updated version to reflect improvements
    packages=find_packages(exclude=['test', 'docs']),  # Exclude docs and test folders
    data_files=[
        ('share/ament_index/resource_index/packages',
            [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        # Auto-include URDF, launch, RViz, and config files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'numpy',          # Optional useful dependency
        'pyyaml'          # For reading configs if needed
    ],
    zip_safe=False,  # Changed to False for better debug and source install support
    maintainer='Abuthwahir M',
    maintainer_email='abuthwahir@roboticslab.com',
    description='ROS 2 package for simulating a delivery robot arm using RViz and Python nodes',
    license='MIT',
    keywords=['ROS2', 'Robot Arm', 'Simulation', 'Delivery Robot'],
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'delivery_node = arm_delivery_project.delivery_node:main',
            'control_node = arm_delivery_project.control_node:main',  # Example: new script entry
        ],
    },
)
