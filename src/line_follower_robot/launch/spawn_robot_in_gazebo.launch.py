import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directories
    pkg_dir = get_package_share_directory('line_follower_robot')
    
    # Set Gazebo environment variables
    os.environ['GAZEBO_MODEL_PATH'] = os.path.join(pkg_dir, 'urdf')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = os.path.join(pkg_dir, 'worlds', 'line_follower.world')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'robot.urdf')
    
    # Read the URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
        }.items()
    )
    
    
    # Launch RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'config.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz', default='true'))
    )


    
    # Return a launch description with all the nodes
    return LaunchDescription([
        gazebo,
        rviz
    ])