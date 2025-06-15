# pkg create cpp_tf2_transform --build-type ament_cmake --dependencies rclcpp geometry_msgs tf2_ros
cmake_minimum_required(VERSION 3.8)
project(cpp_tf2_transform)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(tf2_transform src/tf2_transform.cpp)  # 将 C++ 源文件编译成可执行文件
ament_target_dependencies(tf2_transform rclcpp geometry_msgs tf2_ros) # 链接目标可执行文件与 ROS 2 包和依赖库

# target_link_libraries(tf2_transform
#   ${rclcpp_LIBRARIES}
#   ${geometry_msgs_LIBRARIES}
# )  #用于指定将目标（例如可执行文件或库）与特定的库链接在一起

install(TARGETS
  tf2_transform
  DESTINATION lib/${PROJECT_NAME}) #构建过程中将可执行文件安装到指定位置

ament_package()
