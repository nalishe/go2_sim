cmake_minimum_required(VERSION 3.8)
project(go2_description)

find_package(ament_cmake REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# 安裝資源（把資料夾照原樣裝到 share/go2_description/）
install(DIRECTORY
  xacro
  dae
  meshes
  urdf
  rviz
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
