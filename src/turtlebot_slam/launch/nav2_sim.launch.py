import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Set environment variables
    env_turtlebot_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')
    
    # Get package directories
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_turtlebot_slam = get_package_share_directory('turtlebot_slam')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')
    
    # Paths
    world = os.path.join(
        pkg_turtlebot_slam,
        'worlds',
        'office_small.world'
    )
    
    map_path = os.path.join(
        pkg_turtlebot_slam,
        'map',
        'office.yaml'
    )
    
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world,
            'verbose': 'true'
        }.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    # Robot state publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # Modified spawn command with explicit path and timeout
    spawn_turtlebot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'burger',
            '-file', os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_burger', 'model.sdf'),
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
            '-timeout', '60.0'
        ],
        output='screen'
    )
    
    # Navigation stack components
    localization_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_path,
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(pkg_nav2_bringup, 'params', 'nav2_params.yaml')
        }.items()
    )
    
    navigation_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(pkg_nav2_bringup, 'params', 'nav2_params.yaml')
        }.items()
    )
    
    rviz_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'rviz_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz_config': os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')
        }.items()
    )
    
    ld = LaunchDescription()
    
    # Add environment variable
    ld.add_action(env_turtlebot_model)
    
    # Add gazebo commands
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    
    # Add robot state publisher
    ld.add_action(robot_state_publisher_cmd)
    
    # Add spawn command
    ld.add_action(spawn_turtlebot_cmd)
    
    # Add navigation stack with proper timing
    ld.add_action(TimerAction(
        period=5.0,  # Wait for robot to spawn properly
        actions=[localization_launch_cmd]
    ))
    
    ld.add_action(TimerAction(
        period=10.0,  # Wait for localization to initialize
        actions=[navigation_launch_cmd]
    ))
    
    ld.add_action(TimerAction(
        period=15.0,  # Wait for navigation to initialize
        actions=[rviz_launch_cmd]
    ))
    
    return ld
