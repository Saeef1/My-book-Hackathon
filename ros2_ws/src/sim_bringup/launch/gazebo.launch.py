from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            os.getenv('AMENT_PREFIX_PATH').split(os.pathsep)[0],
            'share', 'gazebo_ros', 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': LaunchConfiguration('world_file_path')}.items(),
    )

    # Spawn robot
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'simple_robot',
                                   '-topic', 'robot_description',
                                   '-x', '0', '-y', '0', '-z', '0.2'],
                        output='screen')

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': '<robot><link name="world"/><joint name="world_fixed" type="fixed"><parent link="world"/><child link="base_link"/></joint></robot>'}], # Minimal URDF
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_file_path',
            default_value=[os.path.join(os.getenv('HOME'), 'my-book', 'gazebo_sim', 'worlds', 'empty.world')],
            description='Path to the Gazebo world file'
        ),
        gazebo,
        spawn_entity,
        robot_state_publisher,
    ])