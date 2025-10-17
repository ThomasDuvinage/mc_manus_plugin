cmake_minimum_required(VERSION 3.14)

project(
  ManusPlugin
  LANGUAGES CXX
  VERSION 1.0.0)

include(CTest)
enable_testing()

if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

find_package(manus_ros2_msgs REQUIRED)

add_library(mc_manus SHARED src/ManusDevice.cpp src/ManusDevice.h)
target_link_libraries(mc_manus PUBLIC mc_rtc::mc_rbdyn mc_rtc::mc_rtc_ros)
ament_target_dependencies(mc_manus PUBLIC manus_ros2_msgs)

set(MC_MANUS_EXPORT_NAME mc_manus)
install(
  TARGETS mc_manus
  EXPORT ${MC_MANUS_EXPORT_NAME}Targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(FILES src/ManusDevice.h DESTINATION include/mc_manus)

# Generate and install CMake package config files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/mc_manusConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion)

install(
  EXPORT ${MC_MANUS_EXPORT_NAME}Targets
  FILE ${MC_MANUS_EXPORT_NAME}Targets.cmake
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

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
