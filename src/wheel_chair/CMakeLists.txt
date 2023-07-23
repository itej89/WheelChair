cmake_minimum_required(VERSION 3.8)
project(wheel_chair)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  wheel_chair
  SHARED
  hardware/src/WheelChairSystemHardware.cpp
)

target_compile_features(wheel_chair PUBLIC cxx_std_17)
target_include_directories(wheel_chair PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/wheel_chair>
)

ament_target_dependencies(
  wheel_chair PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "WHEEL_CHAIR_BUILDING_DLL")


# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface plugins.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/wheel_chair
)
  

install(
  DIRECTORY description/launch description/ros2_control description/urdf description/rviz
  DESTINATION share/wheel_chair
)

install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/wheel_chair
)

install(TARGETS wheel_chair
  EXPORT wheel_chair
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

## EXPORTS
ament_export_targets(wheel_chair HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()