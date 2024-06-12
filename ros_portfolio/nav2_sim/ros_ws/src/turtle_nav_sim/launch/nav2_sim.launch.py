import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess

def generate_launch_description():
    #path to the turtlebot3_world.launch.py file
    gazebo_launch_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py'
    )

    #path to the navigation2.launch.py file
    nav2_launch_path = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'), 'launch', 'navigation2.launch.py'
    )

    #gazebo launch description
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_path])
    )

    #navigation2 launch description with launch arguments
    nav2sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_path]),
        launch_arguments={'use_sim_time':'True','map':"/home/icorinthian/Documents/ros/git/ROS_Portfolio/ros_portfolio/nav2_sim/ros_ws/src/turtle_nav_sim/map/sim_map.yaml"}.items()
    )

    # Launch
    return LaunchDescription([
        nav2sim,
        gazebo,
    ])
