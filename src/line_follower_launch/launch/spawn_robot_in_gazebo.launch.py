import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    line_follower_world = "/home/soham/line_follower_ws/src/line_follower_world/worlds/maze.urdf"
    robot_urdf = "/home/soham/line_follower_ws/robot.urdf"

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
    )

    # Spawn the maze (ground)
    spawn_maze = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-file", line_follower_world, "-entity", "maze"],
        output="screen"
    )

    # Spawn the robot
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-file", robot_urdf, "-entity", "FLF"],
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        spawn_maze,
        spawn_robot,
    ])

