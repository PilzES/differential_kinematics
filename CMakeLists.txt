cmake_minimum_required(VERSION 3.8)
project(differential_kinematics)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ###########
# ## Build ##
# ###########

ament_export_dependencies(std_msgs tf nav_msgs sensor_msgs)
ament_export_include_directories(common/include)
ament_export_libraries(${PROJECT_NAME})

add_library(${PROJECT_NAME} common/src/DiffDrive2WKinematics.cpp)

#Differential kinematics node
add_executable(differential_kinematics_node ros/src/differential_kinematics_node.cpp)

ament_target_dependencies(differential_kinematics_node
  rclcpp
  tf2_ros
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  trajectory_msgs
  tf2_geometry_msgs)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2_ros
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  trajectory_msgs
  tf2_geometry_msgs)

# Drive bridge node
add_executable(drive_bridge_node ros/src/drive_bridge.cpp)

ament_target_dependencies(drive_bridge_node
  rclcpp
  sensor_msgs
  trajectory_msgs
)

target_link_libraries(differential_kinematics_node ${PROJECT_NAME})

install(TARGETS 
  differential_kinematics_node
  drive_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY common/include/
  DESTINATION common/include/
)

ament_package()