import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

config_file = os.path.join(
    get_package_share_directory("exploration_manager"),
    "config",
    "exploration_manager_config.yaml",
    )

def generate_launch_description():
    container = ComposableNodeContainer(
        name='voxel_mapping_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='exploration_manager',
                plugin='ExplorationManagerNode',
                name='exploration_manager_node',
                parameters=[
                    config_file
                ]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
