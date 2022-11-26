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
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    controller_yaml = os.path.join(pkg_share,'config','controller_config.yaml')
    planner_yaml = os.path.join(pkg_share,'config','planner_config.yaml')
    bt_yaml = os.path.join(pkg_share,'config','behaviour_config.yaml')
    recoveries_yaml = os.path.join(pkg_share,'config','recoveries_config.yaml')

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

    nav2_path_plan_lifecyle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name = 'lifecycle_manager_path_planning_node',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['planner_server',
                                    'controller_server',
                                    'recoveries_server',
                                    'bt_navigator']}]
    )
    
    
    


    ld = LaunchDescription()
    ld.add_action(nav2_controller_node)
    ld.add_action(nav2_planner_node)
    ld.add_action(nav2_recoveries_node)
    ld.add_action(nav2_bt_node)
    ld.add_action(nav2_path_plan_lifecyle_node)

    return ld