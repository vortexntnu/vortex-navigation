import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

config_file = os.path.join(
    get_package_share_directory("mapping_interface_example"),
    "config",
    "mapping_interface_example.yaml",
    )

def generate_launch_description():
    container = ComposableNodeContainer(
        name='voxel_mapping_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='mapping_interface_example',
                plugin='MappingInterfaceNode',
                name='mapping_interface_node',
                parameters=[
                    config_file
                ]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
