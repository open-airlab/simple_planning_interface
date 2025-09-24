import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('simple_planning_interface')

    # Paths to YAML and RViz config
    waypoints_yaml = os.path.join(pkg_dir, 'cfg', 'waypoints.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'rviz_interface.rviz')

    # frame_id launch argument
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Frame ID for the planning interface'
    )
    frame_id = LaunchConfiguration('frame_id')

    # Node for your basic interface
    basic_interface_node = Node(
        package='simple_planning_interface',
        executable='basic_interface_node',  # ROS2 uses "executable" instead of "type"
        name='basic_interface_node',
        output='screen',
        parameters=[
            waypoints_yaml,
            {'frame_id': frame_id},
        ]
    )

    # Node for RViz
    rviz_node = Node(
        package='rviz2',  # In ROS2 it's "rviz2"
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        frame_id_arg,
        basic_interface_node,
        rviz_node
    ])
