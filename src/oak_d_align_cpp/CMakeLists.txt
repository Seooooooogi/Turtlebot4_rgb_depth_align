cmake_minimum_required(VERSION 3.10)
project(oak_d_align_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(depthai CONFIG REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  image_transport
  cv_bridge
  OpenCV
)

# --- Composable node library ---
add_library(oakd_sender_component SHARED
  src/oakd_sender.cpp
)
target_include_directories(oakd_sender_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(oakd_sender_component
  depthai::core
  depthai::opencv
)
ament_target_dependencies(oakd_sender_component ${dependencies})

rclcpp_components_register_nodes(oakd_sender_component
  "oak_d_align_cpp::OakdSender"
)

# --- Standalone executable ---
add_executable(oakd_sender src/oakd_sender_main.cpp)
target_link_libraries(oakd_sender oakd_sender_component)
ament_target_dependencies(oakd_sender ${dependencies})

# --- Install ---
install(TARGETS oakd_sender_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS oakd_sender
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config systemd
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(oakd_sender_component)
ament_export_dependencies(${dependencies})

ament_package()
