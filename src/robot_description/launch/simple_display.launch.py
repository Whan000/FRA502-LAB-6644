#!/usr/bin/env python3

"""
LAB 4 Launch File - 3R Kinematics (Revised)
Complete system launch including controller, random pose node, and visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():
    
    # 1. Configuration Constants
    package_name = 'robot_description'
    default_urdf_name = '01-myfirst.urdf'
    
    # 2. Setup Paths
    pkg_share = get_package_share_directory(package_name)
    rviz_config_path = os.path.join(pkg_share, 'config', 'display.rviz')
    urdf_path = os.path.join(pkg_share, 'robot', 'visual', default_urdf_name)
    
    # 3. Define Launch Arguments
    urdf_model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=default_urdf_name,
        description='Name of the URDF file to load from robot/visual/'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    launch_rviz_arg = DeclareLaunchArgument(
        name='launch_rviz',
        default_value='true',
        description='Launch RViz2'
    )
    
    launch_gui_arg = DeclareLaunchArgument(
        name='launch_gui',
        default_value='true',
        description='Launch operator GUI'
    )

    # 4. Load URDF Content
    try:
        with open(urdf_path, 'r') as inf:
            robot_desc = inf.read()
    except FileNotFoundError:
        print(f"ERROR: URDF file not found at {urdf_path}")
        robot_desc = ""

    # 5. Define Nodes
    
    # Robot State Publisher (Essential - always needed)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Joint State Publisher (For manual testing without controller)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Main Robot Controller (handles all 3 modes: IPK, TO, AM)
    controller_node = Node(
        package='robot_description',
        executable='controller.py',
        name='robot_controller',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Random Pose Node (for Part 1.2 and Auto Mode)
    random_pose_node = Node(
        package='robot_description',
        executable='random_pose_node.py',
        name='random_pose_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # End Effector Publisher (for Part 1.3) - DISABLE THIS TO STOP JIGGLING
    # Comment out this node - it conflicts with controller
    # end_effector_node = Node(
    #     package='robot_description',
    #     executable='end_effector_publisher.py',
    #     name='end_effector_publisher',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': LaunchConfiguration('use_sim_time')
    #     }]
    # )

    # Workspace Analyzer Node (for Part 1.1)
    workspace_analyzer_node = Node(
        package='robot_description',
        executable='workspace_analyzer.py',
        name='workspace_analyzer',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Operator GUI (your existing GUI with mode selection)
    operator_gui = Node(
        package='robot_description',
        executable='OperatorGUI.py',
        name='operator_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_gui')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # RViz2 Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Static transform publisher for world frame
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'link_0'],
        output='screen'
    )

    # 6. Return Launch Description
    return LaunchDescription([
        # Launch arguments
        urdf_model_arg,
        use_sim_time_arg,
        launch_rviz_arg,
        launch_gui_arg,
        
        # Essential nodes (always launch these)
        robot_state_publisher_node,
        static_tf_node,
        
        # Lab-specific nodes (comment out if not ready)
        # Uncomment as you develop each component
        # joint_state_publisher_node,  # Remove when controller is ready
        controller_node,           
        random_pose_node,         
        # end_effector_node,       
        # workspace_analyzer_node,
        
        # GUI and visualization
        operator_gui,
        rviz_node
    ])