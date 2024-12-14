cmake_minimum_required(VERSION 3.8)
project(cv_bridge_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(cvbridge_example_cpp src/cvbridge_example_cpp.cpp)
ament_target_dependencies(cvbridge_example_cpp rclcpp std_msgs sensor_msgs cv_bridge OpenCV)

install(PROGRAMS
  # Add the python node to the programs to be installed
  scripts/cvbridge_example_py.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  cvbridge_example_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()