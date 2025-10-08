cmake_minimum_required(VERSION 3.14)

project(
  ManusPlugin
  LANGUAGES CXX
  VERSION 1.0.0)

include(CTest)
enable_testing()

option(WITH_ROS "Enable ROS communication" ON)

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

add_library(mc_manus SHARED src/ManusDevice.cpp src/ManusDevice.h)
target_link_libraries(mc_manus PUBLIC ${OpenCV_LIBRARIES} mc_rtc::mc_rbdyn)

if(WITH_ROS)
  target_compile_definitions(mc_manus PRIVATE WITH_ROS)
  target_link_libraries(
    mc_manus PUBLIC cv_bridge::cv_bridge image_transport::image_transport
                     "${sensor_msgs_TARGETS}")

endif()

set(MC_CAMERA_EXPORT_NAME mc_manus)
install(
  TARGETS mc_manus
  EXPORT ${MC_CAMERA_EXPORT_NAME}Targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(FILES src/ManusDevice.h DESTINATION include/mc_manus)

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
  "${CMAKE_CURRENT_BINARY_DIR}/mc_manusConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion)

install(
  EXPORT ${MC_CAMERA_EXPORT_NAME}Targets
  FILE ${MC_CAMERA_EXPORT_NAME}Targets.cmake
  NAMESPACE mc_rbdyn::
  DESTINATION lib/cmake/mc_manus)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/mc_manusConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/mc_manusConfig.cmake"
  INSTALL_DESTINATION lib/cmake/mc_manus)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/mc_manusConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/mc_manusConfigVersion.cmake"
        DESTINATION lib/cmake/mc_manus)

# Install the plugin configuration
install(FILES etc/ManusPlugin.yaml
        DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")

set(plugin_SRC src/ManusPlugin.cpp)

set(plugin_HDR src/ManusPlugin.h)

# The add_plugin macro is provided by mc_rtc
add_plugin(ManusPlugin "${plugin_SRC}" "${plugin_HDR}")
target_link_libraries(ManusPlugin PUBLIC mc_manus)

if(WITH_ROS)
  target_compile_definitions(ManusPlugin PRIVATE WITH_ROS)

endif()

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
