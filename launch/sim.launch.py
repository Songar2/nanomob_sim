import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


SENSOR_ARGS = {'caster_r': '0.09', 'cam_x': '0.56', 'cam_y': '0.40', 'cam_z': '0.20', 'cam_pitch': '0.0',
               'lidar_x': '-0.56', 'lidar_y': '0.0', 'lidar_z': '0.20'}   # urdf/nanomob.urdf.xacro 인자


def generate_launch_description():
    pkg = get_package_share_directory('nanomob_sim')
    ros_gz = get_package_share_directory('ros_gz_sim')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    xacro_file = os.path.join(pkg, 'urdf', 'nanomob.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', xacro_file] + sum([[f' {k}:=', LaunchConfiguration(k)] for k in SENSOR_ARGS], [])), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true'),
        *[DeclareLaunchArgument(k, default_value=v) for k, v in SENSOR_ARGS.items()],
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'nanomob_town.sdf')),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg, 'models')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(ros_gz, 'launch', 'gz_sim.launch.py')),
                                 launch_arguments={'gz_args': ['-r -s --headless-rendering ', world]}.items(), condition=IfCondition(headless)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(ros_gz, 'launch', 'gz_sim.launch.py')),
                                 launch_arguments={'gz_args': ['-r ', world]}.items(), condition=UnlessCondition(headless)),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-topic', 'robot_description', '-name', 'nanomob', '-x', '20.0', '-y', '20.0', '-z', '0.26', '-Y', '1.5708']),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-file', os.path.join(pkg, 'models', 'pedestrian', 'model.sdf'), '-name', 'ped1', '-x', '0.0', '-y', '0.0', '-z', '-5.0']),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-file', os.path.join(pkg, 'models', 'bus', 'model.sdf'), '-name', 'bus1', '-x', '0.0', '-y', '0.0', '-z', '-10.0']),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-file', os.path.join(pkg, 'models', 'fence', 'model.sdf'), '-name', 'fence_a', '-x', '0.0', '-y', '5.0', '-z', '-10.0']),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-file', os.path.join(pkg, 'models', 'fence', 'model.sdf'), '-name', 'fence_b', '-x', '0.0', '-y', '10.0', '-z', '-10.0']),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-file', os.path.join(pkg, 'models', 'spectator', 'model.sdf'), '-name', 'spectator', '-x', '0.0', '-y', '0.0', '-z', '30.0']),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-file', os.path.join(pkg, 'models', 'car', 'model.sdf'), '-name', 'car1', '-x', '0.0', '-y', '15.0', '-z', '-10.0']),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-file', os.path.join(pkg, 'models', 'parked_row', 'model.sdf'), '-name', 'row_a', '-x', '0.0', '-y', '20.0', '-z', '-10.0']),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-file', os.path.join(pkg, 'models', 'parked_row', 'model.sdf'), '-name', 'row_b', '-x', '0.0', '-y', '25.0', '-z', '-10.0']),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-file', os.path.join(pkg, 'models', 'e5_lanes', 'model.sdf'), '-name', 'e5_lanes', '-x', '0.0', '-y', '0.0', '-z', '0.0']),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=['/world/nanomob_town/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
             parameters=[{'config_file': os.path.join(pkg, 'config', 'bridge.yaml'), 'use_sim_time': True}]),
        Node(package='nanomob_sim', executable='safety_controller', output='screen', parameters=[{'use_sim_time': True}],
             remappings=[('/cmd_vel', '/nanomob/cmd_safe')]),
        Node(package='nanomob_sim', executable='slope_plant', output='screen', parameters=[{'use_sim_time': True}]),
    ])
