cmake_minimum_required(VERSION 3.18)
project(beckhoff_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# colcon does not set a build type: without one, the code would not be optimized
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(beckhoff_interfaces REQUIRED)
find_package(nlohmann_json 3.9 REQUIRED)  # 3.9 introduced ordered_json
find_package(rclcpp REQUIRED)
find_package(Threads REQUIRED)

# C++ API: BeckhoffPLC and its ADS client, independent of ROS
add_library(${PROJECT_NAME} SHARED
  src/ads_connection.cpp
  src/ads_error.cpp
  src/ads_protocol.cpp
  src/ads_route.cpp
  src/beckhoff_plc.cpp
  src/buffer.cpp
  src/plc_types.cpp
  src/value.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME}
  PUBLIC nlohmann_json::nlohmann_json
  PRIVATE Threads::Threads)

# ROS 2 node
add_executable(beckhoff_ros_node
  src/beckhoff_ros_node.cpp
  src/main.cpp)
target_link_libraries(beckhoff_ros_node
  ${PROJECT_NAME}
  ${beckhoff_interfaces_TARGETS}
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp)

# measures the latency of the PLC requests (ros2 run beckhoff_ros benchmark ...)
add_executable(benchmark src/benchmark.cpp)
target_include_directories(benchmark PRIVATE src)
target_link_libraries(benchmark ${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS beckhoff_ros_node benchmark DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS test/ads_simulator.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the source files have no copyright headers (the license is in LICENSE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_value test/test_value.cpp)
  target_link_libraries(test_value ${PROJECT_NAME})
  ament_add_gtest(test_plc_types test/test_plc_types.cpp)
  target_link_libraries(test_plc_types ${PROJECT_NAME})
  ament_add_gtest(test_buffer test/test_buffer.cpp)
  target_link_libraries(test_buffer ${PROJECT_NAME})
  # the ADS client against a scripted server
  ament_add_gtest(test_ads_connection test/test_ads_connection.cpp TIMEOUT 120)
  target_include_directories(test_ads_connection PRIVATE src)
  target_link_libraries(test_ads_connection ${PROJECT_NAME} Threads::Threads)
  # BeckhoffPLC against a simulated PLC (test/ads_simulator.py)
  ament_add_gtest(test_beckhoff_plc test/test_beckhoff_plc.cpp TIMEOUT 300)
  target_compile_definitions(test_beckhoff_plc PRIVATE
    ADS_SIMULATOR="${CMAKE_CURRENT_SOURCE_DIR}/test/ads_simulator.py")
  target_link_libraries(test_beckhoff_plc ${PROJECT_NAME})

  # the node against a simulated PLC (test/ads_simulator.py on 127.0.0.1:48898)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_beckhoff_ros_node.py TIMEOUT 120)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(nlohmann_json)
ament_package()
