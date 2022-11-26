import os
from struct import pack
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'two_wheeled_robot'
    map_file_name = 'house_map_new.yaml'
    config_name = 'nav2_amcl.yaml'
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    map_file = os.path.join(pkg_share, 'maps',map_file_name)
    nav2_yaml = os.path.join(pkg_share,'config',config_name)

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'yaml_filename':map_file}])
    
    map_odom_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        parameters=["0", "0", "0", "0", "0", "0","10", "map", "odom"])
    
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_yaml])
    
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name = 'lifecycle_manager_node_mapping',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server',
                                    'amcl']}]
    )

    ld = LaunchDescription()
    ld.add_action(map_server_node)
    ld.add_action(map_odom_transform_node)
    ld.add_action(amcl_node)
    
    ld.add_action(lifecycle_manager_node)

    return ld