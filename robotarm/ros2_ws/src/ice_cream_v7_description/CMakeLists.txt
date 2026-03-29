cmake_minimum_required(VERSION 3.8)
project(ice_cream_v7_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
