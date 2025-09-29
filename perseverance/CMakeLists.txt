cmake_minimum_required(VERSION 3.8)
project(perseverance)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set the module path to find custom CMake modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Other Dependencies
find_package(ament_cmake REQUIRED)
find_package(Doxygen)
find_package(Eigen3 REQUIRED)

# ROS Dependencies
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(
  include
  include/${PROJECT_NAME}
  ${Eigen3_INCLUDE_DIRS}
)

# --- Sensor Drivers (Library) ---
add_library(bno055_driver src/rover_sensors/drivers/bno055.cpp)
target_include_directories(bno055_driver PUBLIC
  $<BUILD_INTERFACE:${INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bno055_driver rclcpp sensor_msgs)

# --- Sensor Managers (Library) ---
add_library(laser_manager_lib src/rover_sensors/managers/laser_manager.cpp)
target_include_directories(laser_manager_lib PUBLIC
  $<BUILD_INTERFACE:${INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(laser_manager_lib rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs)

# --- Simulated Sensors (Library) ---
add_library(sim_sensors_lib
  src/rover_sensors/sim/sim_imu.cpp
  src/rover_sensors/sim/sim_lidar.cpp
)
target_include_directories(sim_sensors_lib PUBLIC
  $<BUILD_INTERFACE:${INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sim_sensors_lib rclcpp sensor_msgs geometry_msgs tf2_ros)

# --- BNO055 (IMU) Node ---
add_executable(bno055_node src/rover_sensors/nodes/bno055_node.cpp)
target_include_directories(bno055_node PUBLIC
  $<BUILD_INTERFACE:${INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(bno055_node bno055_driver)
ament_target_dependencies(bno055_node rclcpp sensor_msgs)

# --- Simulated IMU Node ---
add_executable(simulated_imu src/rover_sensors/sim/sim_imu.cpp)
target_include_directories(simulated_imu PUBLIC
  $<BUILD_INTERFACE:${INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(simulated_imu sim_sensors_lib)
ament_target_dependencies(simulated_imu rclcpp sensor_msgs)

# --- Simulated Lidar Node ---
add_executable(simulated_lidar src/rover_sensors/sim/sim_lidar.cpp)
target_include_directories(simulated_lidar PUBLIC
  $<BUILD_INTERFACE:${INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(simulated_lidar sim_sensors_lib)
ament_target_dependencies(simulated_lidar rclcpp sensor_msgs geometry_msgs tf2_ros)

# --- Laser Manager Node ---
add_executable(laser_manager_node src/rover_sensors/managers/laser_manager.cpp)
target_include_directories(laser_manager_node PUBLIC
  $<BUILD_INTERFACE:${INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(laser_manager_node laser_manager_lib)
ament_target_dependencies(laser_manager_node rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs)

# --- State Estimation Node ---
add_executable(state_estimator
  src/state_estimation/state_estimation.cpp
  src/state_estimation/kalman_filter.cpp
)
ament_target_dependencies(state_estimator
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
)

# --- WheelDataSPI Node ---
add_executable(wheel_data_spi src/communications/wheel_data_spi_node.cpp)
target_include_directories(wheel_data_spi PUBLIC
  $<BUILD_INTERFACE:${INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(wheel_data_spi rclcpp std_msgs geometry_msgs)

# Install Targets (Nodes and libraries)
install(TARGETS
  sim_sensors_lib
  laser_manager_lib
  bno055_node
  bno055_driver
  laser_manager_node
  simulated_imu
  simulated_lidar
  state_estimator
  wheel_data_spi
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install Mesh files
install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # # the following line skips the linter which checks for copyrights
  # # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # # the following line skips cpplint (only works in a git repo)
  # # comment the line when this package is in a git repo and when
  # # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

  # find_package(ament_cmake_uncrustify REQUIRED)

  # # Run uncrustify with your custom config
  # ament_uncrustify(
  #   CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/uncrustify.cfg
  #   # Optionally restrict to certain languages
  #   LANGUAGE C CXX
  #   # Optionally exclude vendored/3rd-party files
  #   EXCLUDE
  #   include/perseverance/rover_sensors/drivers/smbus_functions.h
  # )

  find_package(ament_cmake_cpplint REQUIRED)
  # Runs cpplint over your C/C++ sources as a CTest
  ament_cpplint(
    EXCLUDE
    include/perseverance/rover_sensors/drivers/smbus_functions.h
  )
endif()

# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" OFF)

# build just because Doxygen is missing
if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
  # Turn the README.md into the homepage of the doxygen docs
  set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

  # Tell Doxygen where to find the documentation
  doxygen_add_docs(doxygen ${PROJECT_SOURCE_DIR}/include/ ${PROJECT_SOURCE_DIR}/README.md ALL)

  # The documentation will be in the build/html directory
  # The main page is build/html/index.html
endif()

ament_package()
