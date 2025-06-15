cmake_minimum_required(VERSION 3.8)
project(point_match)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译器警告
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "-Wno-error=deprecated-copy -Wno-deprecated-copy")

list(APPEND CMAKE_MODULE_PATH /workspaces/ros2_ws/third_party/g2o/cmake_modules)
set(G2O_ROOT /usr/local/include/g2o)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(G2O REQUIRED)
find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)


# 包含目录
‘(
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
)

install(DIRECTORY image
  DESTINATION share/${PROJECT_NAME}
)

# 可执行文件配置
add_executable(icp_match src/icp.cpp)
target_link_libraries(icp_match 
  Eigen3::Eigen
)

add_executable(ceres_curve_fitting src/ceres_curve_fitting.cpp)
target_link_libraries(ceres_curve_fitting 
  ${CERES_LIBRARIES}
)

add_executable(pnp src/pnp.cpp)
target_link_libraries(pnp
  g2o_core 
  g2o_stuff
  fmt::fmt
  ${OpenCV_LIBS}
  ${G2O_LIBRARIES}
)

# 安装配置
install(
  TARGETS
  icp_match
  ceres_curve_fitting
  pnp
  DESTINATION lib/${PROJECT_NAME}
)

# 测试配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()