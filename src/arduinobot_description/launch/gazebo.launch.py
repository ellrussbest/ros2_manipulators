from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable, 
                            IncludeLaunchDescription)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    ld = LaunchDescription()
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    arduinobot_description_path = get_package_share_directory('arduinobot_description')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    default_model_path = PathJoinSubstitution([arduinobot_description_path, 'urdf', 'gazebo.arduinobot.urdf.xacro'])

    ld.add_action(DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute Path to the robot URDF file'
    ))

    ld.add_action(SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[str(Path(arduinobot_description_path).parent.resolve())]
    ))
    
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]))
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [' -r -v 4 empty.sdf '],
                'on_exit_shutdown': 'True'
            }.items(),
    ))

    ld.add_action(Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'arduinobot']
    ))

    ld.add_action(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    ))

    return ld