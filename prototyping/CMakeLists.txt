cmake_minimum_required(VERSION 3.14)
project(ackermann_planner_demo LANGUAGES CXX)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Warnings (optional but recommended)
if (MSVC)
  add_compile_options(/W4)
else()
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Library: Ackermann planner core
add_library(ackermann_planner STATIC
  ackermann_planner.cpp
)

# Public include directories for the library (so consumers can include the header)
target_include_directories(ackermann_planner
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link dependencies for the library
target_link_libraries(ackermann_planner
  PUBLIC
    Eigen3::Eigen
)

# Demo executable
add_executable(ackermann_planner_demo
  ackermann_planner_demo.cpp
)

target_link_libraries(ackermann_planner_demo
  PRIVATE
    ackermann_planner
    Eigen3::Eigen
)

# Helpful output
message(STATUS "Eigen3 include dirs: ${EIGEN3_INCLUDE_DIRS}")
