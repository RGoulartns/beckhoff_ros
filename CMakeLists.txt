cmake_minimum_required(VERSION 3.18)
project(beckhoff_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
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

# Beckhoff's ADS library (https://github.com/Beckhoff/ADS), the library pyads wraps, taken from:
#  1. a CMake installation of it (find_package(ads))
#  2. a meson installation of it or Beckhoff's libads-dev package (libAdsLib.a, include/AdsLib)
#  3. otherwise its sources are downloaded (BECKHOFF_ROS_FETCH_ADS). An existing copy can be
#     used instead with -DFETCHCONTENT_SOURCE_DIR_BECKHOFF_ADS=/path/to/ADS
option(BECKHOFF_ROS_FETCH_ADS "Download Beckhoff's ADS library if it is not installed" ON)
set(BECKHOFF_ROS_ADS_GIT_REPOSITORY "https://github.com/Beckhoff/ADS.git" CACHE STRING
  "Git repository of Beckhoff's ADS library")
set(BECKHOFF_ROS_ADS_GIT_TAG "master" CACHE STRING
  "Branch, tag or commit of Beckhoff's ADS library to download")

find_package(ads CONFIG QUIET)
if(TARGET ads::ads)
  message(STATUS "Using Beckhoff's ADS library from ${ads_DIR}")
else()
  find_path(ADSLIB_INCLUDE_DIR NAMES AdsLib.h PATH_SUFFIXES AdsLib)
  # the static library: the shared one built by meson exports a C interface
  find_library(ADSLIB_LIBRARY NAMES libAdsLib.a AdsLib)
  if(ADSLIB_INCLUDE_DIR AND ADSLIB_LIBRARY)
    message(STATUS "Using Beckhoff's ADS library ${ADSLIB_LIBRARY}")
    add_library(ads::ads UNKNOWN IMPORTED)
    set_target_properties(ads::ads PROPERTIES
      IMPORTED_LOCATION "${ADSLIB_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${ADSLIB_INCLUDE_DIR}"
      INTERFACE_LINK_LIBRARIES Threads::Threads)
  elseif(BECKHOFF_ROS_FETCH_ADS)
    message(STATUS "Beckhoff's ADS library not found, using its sources "
      "(${BECKHOFF_ROS_ADS_GIT_REPOSITORY} ${BECKHOFF_ROS_ADS_GIT_TAG})")
    include(FetchContent)
    fetchcontent_declare(beckhoff_ads
      GIT_REPOSITORY ${BECKHOFF_ROS_ADS_GIT_REPOSITORY}
      GIT_TAG ${BECKHOFF_ROS_ADS_GIT_TAG}
      SOURCE_SUBDIR AdsLib
      UPDATE_DISCONNECTED TRUE)
    # only AdsLib/ is built (target "ads"), statically
    set(_beckhoff_ros_shared_libs ${BUILD_SHARED_LIBS})
    set(BUILD_SHARED_LIBS OFF)
    fetchcontent_makeavailable(beckhoff_ads)
    set(BUILD_SHARED_LIBS ${_beckhoff_ros_shared_libs})
    target_compile_definitions(ads PRIVATE CONFIG_DEFAULT_LOGLEVEL=1)
    set_target_properties(ads PROPERTIES POSITION_INDEPENDENT_CODE ON)
  else()
    message(FATAL_ERROR "Beckhoff's ADS library (https://github.com/Beckhoff/ADS) was not "
      "found. Install it, or enable BECKHOFF_ROS_FETCH_ADS to download it.")
  endif()
endif()

# C++ API: BeckhoffPLC (port of beckhoff_plc.py), independent of ROS
add_library(${PROJECT_NAME} SHARED
  src/beckhoff_plc.cpp
  src/plc_types.cpp
  src/value.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME}
  PUBLIC nlohmann_json::nlohmann_json
  PRIVATE ads::ads)
# finds a shared ADS library installed outside the system paths
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

# ROS 2 node (port of beckhoff_ros_node.py)
add_executable(beckhoff_ros_node
  src/beckhoff_ros_node.cpp
  src/main.cpp)
target_link_libraries(beckhoff_ros_node
  ${PROJECT_NAME}
  ${beckhoff_interfaces_TARGETS}
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS beckhoff_ros_node DESTINATION lib/${PROJECT_NAME})
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

  # node + BeckhoffPLC against a simulated PLC (test/ads_simulator.py on 127.0.0.1:48898)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_beckhoff_ros_node.py TIMEOUT 120)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(nlohmann_json)
ament_package()
