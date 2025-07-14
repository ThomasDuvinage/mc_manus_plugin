cmake_minimum_required(VERSION 3.14)

project(
  CameraPlugin
  LANGUAGES CXX
  VERSION 1.0.0)

include(CTest)
enable_testing()

option(WITH_ROS "Enable ROS communication" OFF)

# Find mc_rtc if we are not building inside mc_rtc itself
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

find_package(OpenCV REQUIRED)

if(WITH_ROS)
  find_package(cv_bridge REQUIRED)
  find_package(image_transport REQUIRED)
  find_package(sensor_msgs REQUIRED)
endif()

add_library(mc_camera SHARED src/CameraDevice.cpp src/CameraDevice.h)
target_compile_definitions(mc_camera PRIVATE -DWITH_ROS=${WITH_ROS})
target_link_libraries(
  mc_camera PUBLIC ${OpenCV_LIBRARIES} mc_rtc::mc_rbdyn cv_bridge::cv_bridge
                   image_transport::image_transport "${sensor_msgs_TARGETS}")

set(MC_CAMERA_EXPORT_NAME mc_camera)
install(
  TARGETS mc_camera
  EXPORT ${MC_CAMERA_EXPORT_NAME}Targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(FILES src/CameraDevice.h DESTINATION include/mc_camera)

if(WITH_ROS)
  set(ROS_DEPENDENCIES
      [[
find_dependency(cv_bridge)
find_dependency(image_transport)
find_dependency(sensor_msgs)
]])
else()
  set(ROS_DEPENDENCIES "")
endif()

# Generate and install CMake package config files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/mc_cameraConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion)

install(
  EXPORT ${MC_CAMERA_EXPORT_NAME}Targets
  FILE ${MC_CAMERA_EXPORT_NAME}Targets.cmake
  NAMESPACE mc_rbdyn::
  DESTINATION lib/cmake/mc_camera)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/mc_cameraConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/mc_cameraConfig.cmake"
  INSTALL_DESTINATION lib/cmake/mc_camera)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/mc_cameraConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/mc_cameraConfigVersion.cmake"
        DESTINATION lib/cmake/mc_camera)

# Install the plugin configuration
install(FILES etc/CameraPlugin.yaml
        DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")

set(plugin_SRC src/CameraPlugin.cpp)

set(plugin_HDR src/CameraPlugin.h)

# The add_plugin macro is provided by mc_rtc
add_plugin(CameraPlugin "${plugin_SRC}" "${plugin_HDR}")
target_compile_definitions(CameraPlugin PRIVATE -DWITH_ROS=${WITH_ROS})
target_link_libraries(CameraPlugin PUBLIC mc_camera)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
