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
    rviz_config_file_path = 'rviz/path_planning_config.rviz'
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    map_file = os.path.join(pkg_share, 'maps',map_file_name)
    nav2_yaml = os.path.join(pkg_share,'config',config_name)
    controller_yaml = os.path.join(pkg_share,'config','controller_config.yaml')
    planner_yaml = os.path.join(pkg_share,'config','planner_config.yaml')
    bt_yaml = os.path.join(pkg_share,'config','behaviour_config.yaml')
    recoveries_yaml = os.path.join(pkg_share,'config','recoveries_config.yaml')
    default_rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)

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
        arguments=["0", "0", "0", "0", "0", "0", "10", "map", "odom"])
    
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_yaml])
    
    nav2_controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_yaml])
    
    nav2_planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_yaml])
    
    nav2_recoveries_node = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[recoveries_yaml])
    
    nav2_bt_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_yaml])
    
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path])
    
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name = 'lifecycle_manager_node',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server',
                                    'amcl',
                                    'planner_server',
                                    'controller_server',
                                    'recoveries_server',
                                    'bt_navigator']}]
    )

    ld = LaunchDescription()
    ld.add_action(map_server_node)
    ld.add_action(map_odom_transform_node)
    ld.add_action(start_rviz_cmd)
    ld.add_action(amcl_node)
    ld.add_action(nav2_controller_node)
    ld.add_action(nav2_planner_node)
    ld.add_action(nav2_recoveries_node)
    ld.add_action(nav2_bt_node)
    ld.add_action(lifecycle_manager_node)

    return ld