cmake_minimum_required(VERSION 3.10.2)
project(unitree_l2_rotor_gz_plugin)

find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)

gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

add_library(UnitreeL2Rotor SHARED src/UnitreeL2Rotor.cc)
target_compile_features(UnitreeL2Rotor PUBLIC cxx_std_17)
target_link_libraries(UnitreeL2Rotor
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
)

install(TARGETS UnitreeL2Rotor
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

ament_package()
