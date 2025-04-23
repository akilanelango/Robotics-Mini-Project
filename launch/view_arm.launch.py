from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('arm_delivery_project')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'arm.urdf')
    
    # Use the updated RViz config
    rviz_config_dir = os.path.join(pkg_dir, 'rviz', 'arm.rviz')
    
    return LaunchDescription([
        # Joint state publisher
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(urdf_path).read()
            }]
        ),
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_dir] if os.path.exists(rviz_config_dir) else []
        ),
        # Start the delivery node
        Node(
            package='arm_delivery_project',
            executable='delivery_node',
            name='delivery_node',
            output='screen'
        )
    ])
