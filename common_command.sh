source ./install/setup.bash
# 创建node
ros2 pkg create py_transform --build-type ament_python --dependencies rclpy geometry_msgs tf2_ros
pkg create cpp_tf2_transform --build-type ament_cmake --dependencies rclcpp geometry_msgs tf2_ros
# 编译node
rm -rf build install log && colcon build --packages-select my_diffbot && source install/setup.bash
ros2 run pkg node
# 通过urdf发布机器人模型
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat /src/my_tf_robot/urdf/my_robot.urdf)"
# 查看话题
ros2 topic echo /robot_description
# 查看tf之间的变换关系
ros2 run tf2_ros tf2_echo base_link map
# gazebo 启动是source以下，设置必要的环境变量
source /usr/share/gazebo/setup.sh

ros2 launch my_diffbot gazebo.launch.py
# 节点-话题关系图
ros2 run rqt_graph rqt_graph  

