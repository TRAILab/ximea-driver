cmake_minimum_required(VERSION 3.8)
project(ximea_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake REQUIRED)

# Targets

include_directories(include)
add_library(ximea_driver SHARED src/ximea_driver.cpp)
target_link_libraries(ximea_driver m3api)
ament_target_dependencies(ximea_driver
  sensor_msgs rclcpp
)

add_executable(ximea_driver_node src/ximea_driver_node.cpp)
target_link_libraries(ximea_driver_node ximea_driver)

# Installation

install(TARGETS ximea_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS ximea_driver_node DESTINATION lib/${PROJECT_NAME})

# Testing

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
