cmake_minimum_required(VERSION 3.8)
project(object_mover_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FrankaJointRequest.srv"
  "srv/FrankaPoseRequest.srv"
  "srv/AddBox.srv"
  "srv/BoxName.srv"
  "srv/PickPose.srv"
  "srv/CartesianPathRequest.srv"
  "srv/GripperRequest.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(geometry_msgs rosidl_default_generators)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright check temporarily
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint temporarily
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
