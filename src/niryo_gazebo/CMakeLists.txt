cmake_minimum_required(VERSION 3.8)
project(niryo_gazebo)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(gazebo_ros REQUIRED)  # supports Gazebo Classic with ROS2

# Install URDF files
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install Gazebo models (optional)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/models")
  install(
    DIRECTORY models
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# Install world files
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
