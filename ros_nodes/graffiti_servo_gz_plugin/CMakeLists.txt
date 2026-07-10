cmake_minimum_required(VERSION 3.10.2)
project(graffiti_servo_gz_plugin)

find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)

gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

gz_find_package(gz-transport13 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

gz_find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

add_library(GraffitiServoKinematics SHARED src/GraffitiServoKinematics.cc)
target_compile_features(GraffitiServoKinematics PUBLIC cxx_std_17)
target_link_libraries(GraffitiServoKinematics
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
  gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
)

install(TARGETS GraffitiServoKinematics
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

ament_package()
