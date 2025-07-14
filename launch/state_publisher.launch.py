from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue 

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    xacro_file = PathJoinSubstitution([
        FindPackageShare('obstacle_avoidance'),
        'models/urdf',
        'obstacle_bot.xacro'
    ])

    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}  

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use simulation clock if true'),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'obstacle_bot', '-topic', 'robot_description'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
        
        # Add this node to run your obstacle avoidance automatically
        Node(
            package='obstacle_avoidance',
            executable='obstacle_avoidance_node',  # Make sure this matches your actual node executable name
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
