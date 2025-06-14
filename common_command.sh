git remote set-url origin "ssh://git@ssh.github.com:443/mouzhengdong/ros2_ws.git" && ssh -T -p 443 git@ssh.github.com
# 创建node
ros2 pkg create py_transform --build-type ament_python --dependencies rclpy geometry_msgs tf2_ros
pkg create cpp_tf2_transform --build-type ament_cmake --dependencies rclcpp geometry_msgs tf2_ros
rm -rf build install log && colcon build --symlink-install --packages-select my_diffbot && source install/setup.bash
ros2 run pkg node

# 通过urdf发布机器人模型
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat /src/my_tf_robot/urdf/my_robot.urdf)"
ros2 topic echo /robot_description
ros2 run tf2_ros tf2_echo base_link map


# gazebo相关
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/workspaces/ros2_ws/gazebo_models_worlds_collection/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/workspaces/ros2_ws/gazebo_models_worlds_collection/worlds
ros2 launch gazebo_ros gazebo.launch.py world:=/workspaces/ros2_ws/src/gazebo_models_worlds_collection/worlds/office_small.world

source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot_slam launch.py
ros2 run rqt_graph rqt_graph

export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
export TURTLEBOT3_MODEL=burger && ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
export TURTLEBOT3_MODEL=burger && ros2 run turtlebot3_teleop teleop_keyboard

export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml

# 手柄控制 X + 左遥杆
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'

# 导航相关
source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot_slam launch.py
ros2 launch nav2_bringup localization_launch.py map:=/workspaces/ros2_ws/src/turtlebot_slam/map/office.yaml use_sim_time:=true
ros2 launch nav2_bringup rviz_launch.py use_sim_time:=true
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true