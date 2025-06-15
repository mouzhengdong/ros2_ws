cmake_minimum_required(VERSION 3.8)
project(cpp_action_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成 action 接口代码
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CountUntil.action"
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 可执行文件
add_executable(action_server src/action_server.cpp)
add_executable(action_client src/action_client.cpp)

# 添加依赖
ament_target_dependencies(action_server rclcpp rclcpp_action)
ament_target_dependencies(action_client rclcpp rclcpp_action)

# 连接自动生成的接口
target_link_libraries(action_server ${cpp_typesupport_target})
target_link_libraries(action_client ${cpp_typesupport_target})

install(TARGETS
  action_server
  action_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
