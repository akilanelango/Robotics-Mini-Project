from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('arm_delivery_project')
    urdf = os.path.join(pkg, 'urdf', 'arm.urdf')
    rviz_cfg = os.path.join(pkg, 'rviz', 'arm.rviz')
    desc = {'robot_description': open(urdf).read()}

    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='jsp_gui',
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='rsp',
            parameters=[desc],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_cfg] if os.path.exists(rviz_cfg) else [],
            output='screen'
        ),
        Node(
            package='arm_delivery_project',
            executable='delivery_node',
            name='delivery',
            output='screen'
        )
    ])
