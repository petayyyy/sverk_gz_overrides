cmake_minimum_required(VERSION 3.8)
project(mtf01_px4_bridge)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc video)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(flow_estimator src/flow.cpp)
target_include_directories(flow_estimator PUBLIC include)
target_compile_features(flow_estimator PUBLIC cxx_std_17)
target_link_libraries(flow_estimator ${OpenCV_LIBS})

add_executable(mtf01_px4_bridge src/node.cpp)
target_include_directories(mtf01_px4_bridge PRIVATE include)
target_compile_features(mtf01_px4_bridge PUBLIC cxx_std_17)
target_link_libraries(mtf01_px4_bridge flow_estimator ${OpenCV_LIBS})
ament_target_dependencies(mtf01_px4_bridge
  cv_bridge
  px4_msgs
  rclcpp
  sensor_msgs
)

install(TARGETS flow_estimator mtf01_px4_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_flow test/test_flow.cpp)
  if(TARGET test_flow)
    target_include_directories(test_flow PRIVATE include)
    target_compile_features(test_flow PRIVATE cxx_std_17)
    target_link_libraries(test_flow flow_estimator ${OpenCV_LIBS})
  endif()
endif()

ament_package()
