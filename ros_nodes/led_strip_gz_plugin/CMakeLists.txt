cmake_minimum_required(VERSION 3.10.2)
project(led_strip_gz_plugin)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(led_interfaces REQUIRED)
find_package(gz-cmake3 REQUIRED)

gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
gz_find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

add_library(LedStripVisualization SHARED src/LedStripVisualization.cc)
target_compile_features(LedStripVisualization PUBLIC cxx_std_17)
target_link_libraries(LedStripVisualization
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
)
ament_target_dependencies(LedStripVisualization
  rclcpp
  std_msgs
  led_interfaces
)

install(TARGETS LedStripVisualization
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

ament_package()
