from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the shared directory of the package
    pkg_dir = get_package_share_directory('arm_delivery_project')

    # Construct path to the URDF file
    urdf_path = os.path.join(pkg_dir, 'urdf', 'arm.urdf')
    
    # Construct path to the RViz configuration file
    rviz_config_dir = os.path.join(pkg_dir, 'rviz', 'arm.rviz') 

    return LaunchDescription([
        # Launch the Joint State Publisher GUI node
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        
        # Launch the Robot State Publisher node with robot description from URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(urdf_path).read()
            }]
        ),

        # Launch RViz with a specific config file if it exists
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_dir] if os.path.exists(rviz_config_dir) else []
        ),

        # Launch the delivery node from the arm_delivery_project package
        Node(
            package='arm_delivery_project',
            executable='delivery_node',
            name='delivery_node',
            output='screen'
        )
    ])
