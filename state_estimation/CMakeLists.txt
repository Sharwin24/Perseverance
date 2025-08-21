cmake_minimum_required(VERSION 3.8)
project(state_estimation)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set the module path to find custom CMake modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# ROS Dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Other Dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensors REQUIRED)
find_package(Doxygen)

include_directories(
  include
  include/state_estimation
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(state_estimator src/state_estimation.cpp src/kalman_filter.cpp)
ament_target_dependencies(state_estimator rclcpp geometry_msgs sensor_msgs nav_msgs std_msgs
  tf2_ros tf2_geometry_msgs)

# Install
install(TARGETS
  state_estimator
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

# if(BUILD_TESTING)
# find_package(ament_lint_auto REQUIRED)
# # the following line skips the linter which checks for copyrights
# # comment the line when a copyright and license is added to all source files
# set(ament_cmake_copyright_FOUND TRUE)
# # the following line skips cpplint (only works in a git repo)
# # comment the line when this package is in a git repo and when
# # a copyright and license is added to all source files
# set(ament_cmake_cpplint_FOUND TRUE)
# ament_lint_auto_find_test_dependencies()
# endif()

# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" OFF)

# build just because Doxygen is missing
if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
  # Turn the README.md into the homepage of the doxygen docs
  set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

  # Tell Doxygen where to find the documentation
  doxygen_add_docs(doxygen include/ src/ README.md ALL)

  # The documentation will be in the build/html directory
  # The main page is build/html/index.html
endif()

ament_package()
