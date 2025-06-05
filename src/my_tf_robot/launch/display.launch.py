import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('my_tf_robot'),
        'urdf',
        'my_robot.urdf'
    )

    with open(urdf_path, 'r') as inf:
        robot_desc = inf.read()
        
    return LaunchDescription([
        # 启动 robot_state_publisher 解析 URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False},{'robot_description': robot_desc}],
            # arguments=[urdf_path]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # 启动你自己的位置发布节点
        Node(
            package='my_tf_robot',
            executable='pose_publisher',
            name='pose_publisher',
            output='screen'
        ),

        # 可选：rviz 可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', 'rviz/view_config.rviz']
        ),
    ])