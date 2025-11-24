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

# Create interface library for header-only usage
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME} INTERFACE
  rclcpp
  rcpputils
  ublox_msgs
  message_filters
)

# Example executable
add_executable(synchronizer
  src/example.cpp
)
ament_target_dependencies(synchronizer
  rclcpp
  rcpputils
  ublox_msgs
  message_filters
)
target_include_directories(synchronizer PRIVATE include)

# Install targets
install(TARGETS
  synchronizer
  DESTINATION lib/${PROJECT_NAME}
)

# Install the interface library
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Python executables
install(PROGRAMS
  scripts/talker.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Export the targets
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rcpputils
  ublox_msgs
  message_filters
)
ament_export_include_directories(include/${PROJECT_NAME})

ament_package()