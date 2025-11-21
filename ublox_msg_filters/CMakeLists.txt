cmake_minimum_required(VERSION 3.5)
project(ublox_msg_filters)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-unused-function)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(ublox_msgs REQUIRED)
find_package(message_filters REQUIRED)

# Include directories
include_directories(
  include
)

# Example executable
add_executable(${PROJECT_NAME}_example
  src/example.cpp
)
ament_target_dependencies(${PROJECT_NAME}_example
  rclcpp
  rcpputils
  ublox_msgs
  message_filters
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}_example
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}")

ament_package()