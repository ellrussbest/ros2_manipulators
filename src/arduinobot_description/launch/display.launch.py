from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    ld = LaunchDescription()

    arduinobot_description_path = FindPackageShare('arduinobot_description')
    default_model_path = PathJoinSubstitution([arduinobot_description_path, 'urdf', 'arduinobot.urdf.xacro'])
    default_rviz_config_path = PathJoinSubstitution([arduinobot_description_path, 'rviz', 'display.rviz'])

    # launch file customizable arguments
    ld.add_action(DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute Path to the robot URDF file'
    ))

    ld.add_action(DeclareLaunchArgument(
        name='rviz_config',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file'
    ))

    # ros2 node dependencies
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]))
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    ))

    ld.add_action(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    ))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    ))

    return ld