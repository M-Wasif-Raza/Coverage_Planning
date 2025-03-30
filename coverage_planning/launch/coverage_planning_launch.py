# Copyright (c) 2023 Open Navigation LLC
# Modified by Wasif Raza

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    coverage_demo_dir = get_package_share_directory('coverage_planning')
    mts_dept_dir = get_package_share_directory('mts_department')
    
    world = os.path.join(mts_dept_dir, 'worlds', 'mts_dept.world')
    param_file_path = os.path.join(coverage_demo_dir, 'params.yaml')

    sdf = os.path.join(nav2_bringup_dir, 'worlds', 'waffle.model')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    # start the simulation
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        cwd=[coverage_demo_dir], output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
         cmd=['gzclient'],
         cwd=[coverage_demo_dir], output='screen')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    param_substitutions = {
        'use_sim_time':use_sim_time}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=param_file_path,
            root_key="",
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)


    urdf = os.path.join(nav2_bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')

    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'robot_description': robot_description}])

    start_gazebo_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'tb3',
            '-file', sdf,
            '-x', '0.0', '-y', '0.0', '-z', '0.10',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0'])
    

    # start the visualization
    rviz_config = os.path.join(coverage_demo_dir, 'coverage_planning.rviz')
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        launch_arguments={'namespace': '', 'rviz_config': rviz_config}.items())

    # start navigation
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coverage_demo_dir, 'bringup_launch.py')),
        launch_arguments={'params_file': param_file_path}.items())

    # world->odom transform, no localization. For visualization & controller transform
    fake_localization_cmd = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'])

    # start the demo task
    demo_cmd = Node(
        package='coverage_planning',
        executable='coverage',
        emulate_tty=True,
        output='screen')
        
    ld = LaunchDescription()       
    ld.add_action(declare_use_sim_time_cmd)
     
    # ld.add_action(fake_localization_cmd)   
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)   
    ld.add_action(start_gazebo_spawner_cmd)    
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(demo_cmd)
    
    return ld
