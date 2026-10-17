# Fake PLC (ADS server) used by the integration tests and the benchmark.
add_library(beckhoff_plc_fake_ads_server STATIC fake_ads_server.cpp)
target_include_directories(beckhoff_plc_fake_ads_server PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(beckhoff_plc_fake_ads_server PUBLIC beckhoff_plc Threads::Threads)

add_executable(fake_ads_server fake_ads_server_main.cpp)
target_link_libraries(fake_ads_server PRIVATE beckhoff_plc_fake_ads_server)

if(NOT BECKHOFF_PLC_BUILD_TESTS)
  return()
endif()

find_package(GTest CONFIG QUIET)
if(NOT GTest_FOUND)
  message(STATUS "GoogleTest not found: downloading it")
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG f8d7d77c06936315286eb55f8de22cd23c188571  # v1.14.0
  )
  FetchContent_MakeAvailable(googletest)
endif()
include(GoogleTest)

# Pure unit tests (no network): types, values, configuration.
add_executable(beckhoff_plc_unit_tests test_types.cpp test_value.cpp test_config.cpp)
target_link_libraries(beckhoff_plc_unit_tests PRIVATE beckhoff_plc GTest::gtest_main)
target_compile_definitions(beckhoff_plc_unit_tests PRIVATE
  BECKHOFF_PLC_REPO_PARAMS="${PROJECT_SOURCE_DIR}/../config/params.json"
)
gtest_discover_tests(beckhoff_plc_unit_tests)

# End-to-end tests: the real ADS client library against the fake PLC. They
# share TCP port 48898, hence the resource lock.
add_executable(beckhoff_plc_integration_tests test_beckhoff_plc.cpp)
target_link_libraries(beckhoff_plc_integration_tests PRIVATE beckhoff_plc_fake_ads_server GTest::gtest_main)
gtest_discover_tests(beckhoff_plc_integration_tests PROPERTIES RESOURCE_LOCK ads_tcp_port)
