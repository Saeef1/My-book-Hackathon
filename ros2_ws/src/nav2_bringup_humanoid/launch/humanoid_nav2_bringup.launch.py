from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
import os

def generate_launch_description():
    # Placeholder for Nav2 launch arguments and parameters
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    # Path to Nav2's bringup launch directory
    nav2_bringup_dir = os.path.join(
        os.getenv('AMENT_PREFIX_PATH').split(os.pathsep)[0],
        'share', 'nav2_bringup', 'launch')

    # Nav2 Launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'bringup_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'use_composition': 'True', # Recommended for performance
            'container_name': 'nav2_container'
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('autostart', default_value='True'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(os.getenv('HOME'), 'my-book', 'ros2_ws', 'src', 'nav2_bringup_humanoid', 'config', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'
        ),
        nav2_launch
    ])

# Instructions for human:
# 1. This launch file is a basic template. You will need to customize 'nav2_params.yaml'
#    to suit the specific kinematics and sensor characteristics of your bipedal humanoid robot.
# 2. Ensure Nav2 packages are installed in your ROS 2 environment.
# 3. This assumes Isaac ROS VSLAM or another localization method is providing /tf and /odom.
# 4. To launch: ros2 launch nav2_bringup_humanoid humanoid_nav2_bringup.launch.py
