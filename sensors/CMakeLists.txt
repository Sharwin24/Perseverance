cmake_minimum_required(VERSION 3.8)
project(sensors)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set the module path to find custom CMake modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# ROS Dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Other Dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Doxygen)

# Common include path for all targets
set(SENSORS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(
  include
  include/sensors
  ${Eigen3_INCLUDE_DIRS}
)

# --- Drivers ---
add_library(bno055_driver
  src/drivers/bno055.cpp
)
target_include_directories(bno055_driver PUBLIC
  $<BUILD_INTERFACE:${SENSORS_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(bno055_driver rclcpp sensor_msgs)

# --- Managers ---
add_library(laser_manager_lib
  src/managers/laser_manager.cpp
)
target_include_directories(laser_manager_lib PUBLIC
  $<BUILD_INTERFACE:${SENSORS_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(laser_manager_lib rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs)

# --- Simulated Sensors ---
add_library(simulated_imu_lib
  src/sim/simulated_imu.cpp
)
target_include_directories(simulated_imu_lib PUBLIC
  $<BUILD_INTERFACE:${SENSORS_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(simulated_imu_lib rclcpp sensor_msgs)

# BN0055 Node
# add_executable(BNO055_Sensor src/bno055_node.cpp src/bno055.cpp)
# ament_target_dependencies(BNO055_Sensor rclcpp sensor_msgs)
add_executable(bno055_node src/nodes/bno055_node.cpp)
target_include_directories(bno055_node PUBLIC
  $<BUILD_INTERFACE:${SENSORS_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(bno055_node bno055_driver)
ament_target_dependencies(bno055_node rclcpp sensor_msgs)

# Simulated IMU Node
# add_executable(simulated_imu src/simulated_imu.cpp)
# ament_target_dependencies(simulated_imu rclcpp sensor_msgs)
add_executable(simulated_imu src/sim/simulated_imu.cpp)
target_include_directories(simulated_imu PUBLIC
  $<BUILD_INTERFACE:${SENSORS_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(simulated_imu simulated_imu_lib)
ament_target_dependencies(simulated_imu rclcpp sensor_msgs)

# Laser Manager
add_executable(laser_manager_node src/managers/laser_manager.cpp)
target_include_directories(laser_manager_node PUBLIC
  $<BUILD_INTERFACE:${SENSORS_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(laser_manager_node laser_manager_lib)
ament_target_dependencies(laser_manager_node rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs)

install(TARGETS
  bno055_driver
  simulated_imu_lib
  laser_manager_lib
  bno055_node
  simulated_imu
  laser_manager_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# Install launch and configfiles
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp sensor_msgs nav_msgs std_msgs geometry_msgs tf2_ros)

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

# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build documentation" OFF)

if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
  # Turn the README.md into the homepage of the doxygen docs
  # set(DOXYGEN_USE_MDFILE_AS_MAINPAGE ../README.md)

  # Tell Doxygen where to find the documentation
  doxygen_add_docs(doxygen ${PROJECT_SOURCE_DIR}/include/ ALL)

  # The documentation will be in the build/html directory
  # The main page is build/html/index.html
endif()

ament_package()
