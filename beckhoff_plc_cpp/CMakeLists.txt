cmake_minimum_required(VERSION 3.18)  # FetchContent SOURCE_SUBDIR
project(beckhoff_plc VERSION 1.0.0 LANGUAGES CXX)

if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
  set(_beckhoff_plc_top_level ON)
else()
  set(_beckhoff_plc_top_level OFF)
endif()
if(DEFINED BUILD_TESTING)
  set(_beckhoff_plc_tests_default ${BUILD_TESTING})
else()
  set(_beckhoff_plc_tests_default ${_beckhoff_plc_top_level})
endif()

option(BECKHOFF_PLC_BUILD_TESTS "Build the tests" ${_beckhoff_plc_tests_default})
option(BECKHOFF_PLC_BUILD_EXAMPLES "Build the examples" ${_beckhoff_plc_top_level})
option(BECKHOFF_PLC_BUILD_BENCHMARKS "Build the latency benchmark" OFF)

if(_beckhoff_plc_top_level AND NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)  # timestamps of downloaded archives
endif()

include(FetchContent)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
find_package(Threads REQUIRED)

# ---- Dependencies -----------------------------------------------------------

# nlohmann_json >= 3.9 (ordered_json): Ubuntu/ROS package nlohmann-json3-dev,
# downloaded when not installed.
find_package(nlohmann_json 3.9 CONFIG QUIET)
if(NOT nlohmann_json_FOUND)
  message(STATUS "nlohmann_json not found: downloading it")
  set(JSON_Install ON CACHE INTERNAL "")
  FetchContent_Declare(nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
    URL_HASH SHA256=d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d
  )
  FetchContent_MakeAvailable(nlohmann_json)
endif()

include(cmake/BeckhoffAds.cmake)

# ---- Library ----------------------------------------------------------------

add_library(beckhoff_plc
  src/beckhoff_plc.cpp
  src/config.cpp
  src/errors.cpp
  src/types.cpp
  src/util.cpp
  src/value.cpp
)
add_library(beckhoff_plc::beckhoff_plc ALIAS beckhoff_plc)
target_include_directories(beckhoff_plc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_features(beckhoff_plc PUBLIC cxx_std_17)
target_link_libraries(beckhoff_plc
  PUBLIC nlohmann_json::nlohmann_json
  PRIVATE beckhoff_plc_ads Threads::Threads
)
set_target_properties(beckhoff_plc PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(beckhoff_plc PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

# ---- Install (find_package(beckhoff_plc) / colcon) -------------------------

install(TARGETS beckhoff_plc beckhoff_plc_ads
  EXPORT beckhoff_plcTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT beckhoff_plcTargets
  NAMESPACE beckhoff_plc::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/beckhoff_plc
)
configure_package_config_file(cmake/beckhoff_plcConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/beckhoff_plcConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/beckhoff_plc
)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/beckhoff_plcConfigVersion.cmake
  COMPATIBILITY SameMajorVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/beckhoff_plcConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/beckhoff_plcConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/beckhoff_plc
)

# ---- Tests, examples, benchmark --------------------------------------------

if(BECKHOFF_PLC_BUILD_TESTS)
  enable_testing()
endif()
if(BECKHOFF_PLC_BUILD_TESTS OR BECKHOFF_PLC_BUILD_BENCHMARKS)
  add_subdirectory(test)
endif()
if(BECKHOFF_PLC_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()
if(BECKHOFF_PLC_BUILD_BENCHMARKS)
  add_subdirectory(benchmark)
endif()
