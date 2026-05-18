cmake_minimum_required(VERSION 3.8)
project(myrobot_plugin_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Custom message definitions. A real plugin would either define these here or
# import them from the manufacturer's own ROS 2 interface package.
file(GLOB_RECURSE msg_files "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")
file(GLOB_RECURSE service_files "${CMAKE_CURRENT_SOURCE_DIR}/srv/*.srv")

foreach(msg ${msg_files})
  file(RELATIVE_PATH msg_rel "${CMAKE_CURRENT_SOURCE_DIR}" ${msg})
  list(APPEND interface_files ${msg_rel})
endforeach()

foreach(srv ${service_files})
  file(RELATIVE_PATH srv_rel "${CMAKE_CURRENT_SOURCE_DIR}" ${srv})
  list(APPEND interface_files ${srv_rel})
endforeach()

if(interface_files)
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${interface_files}
    DEPENDENCIES builtin_interfaces
  )
endif()

ament_export_dependencies(rosidl_default_runtime)

# Install the Python plugin module. The plugin imports `ros_sugar` at runtime
# (declared in package.xml); it is not needed at build time.
ament_python_install_package(myrobot_plugin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
