cmake_minimum_required(VERSION 3.24)
project(BeckhoffPLC VERSION 0.1.0 LANGUAGES CXX)

include(FetchContent)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CTest)
find_package(Threads REQUIRED)

option(BECKHOFF_PLC_WITH_ADS "Build the standalone Beckhoff ADS transport" ON)
option(BECKHOFF_PLC_ADS_INTEGRATION_TEST "Run local pyads test-server integration" OFF)

find_package(nlohmann_json 3.11 CONFIG QUIET)
if(NOT nlohmann_json_FOUND)
    set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
    set(JSON_Install ON CACHE BOOL "" FORCE)
    FetchContent_Declare(json
        URL https://codeload.github.com/nlohmann/json/tar.gz/9cca280a4d0ccf0c08f47a99aa71d1b0e52f8d03
        URL_HASH SHA256=0dbc5e40a01ff142e7e68c03e85247a4dcede2f592d12d3677dee3664d17975a
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    FetchContent_MakeAvailable(json)
endif()

add_library(beckhoff_plc src/types.cpp src/beckhoff_plc.cpp)
add_library(BeckhoffPLC::beckhoff_plc ALIAS beckhoff_plc)
target_compile_features(beckhoff_plc PUBLIC cxx_std_17)
target_include_directories(beckhoff_plc PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(beckhoff_plc PUBLIC nlohmann_json::nlohmann_json Threads::Threads)
set_target_properties(beckhoff_plc PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(BECKHOFF_PLC_WITH_ADS)
    # Upstream's top-level CMake assumes it is the root project; use its library
    # subdirectory directly to avoid building AdsTool or upstream test programs.
    FetchContent_Declare(adslib_source
        URL https://codeload.github.com/Beckhoff/ADS/tar.gz/24e598a113edfdc0d34414bfefb847930d94cb87
        URL_HASH SHA256=9230ffe55d77346ee6a921902bb1b1f1a14da81c2f20d9321f8d8defdb823cf3
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        SOURCE_SUBDIR AdsLib)
    FetchContent_MakeAvailable(adslib_source)
    target_compile_features(AdsLib PUBLIC cxx_std_14)
    target_compile_definitions(AdsLib PRIVATE CONFIG_DEFAULT_LOGLEVEL=1)
    if(MSVC)
        target_compile_definitions(AdsLib PRIVATE NOMINMAX _CRT_SECURE_NO_WARNINGS _CRT_NONSTDC_NO_WARNINGS)
        target_compile_options(AdsLib PRIVATE /utf-8)
    endif()
    set_target_properties(AdsLib PROPERTIES POSITION_INDEPENDENT_CODE ON)
    target_sources(beckhoff_plc PRIVATE src/ads_transport.cpp)
    target_link_libraries(beckhoff_plc PRIVATE AdsLib)
    install(TARGETS AdsLib EXPORT BeckhoffPLCTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ads)
else()
    target_sources(beckhoff_plc PRIVATE src/no_ads_transport.cpp)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(beckhoff_plc PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(beckhoff_example examples/basic.cpp)
target_link_libraries(beckhoff_example PRIVATE BeckhoffPLC::beckhoff_plc)
add_executable(beckhoff_codec_benchmark benchmarks/codec.cpp)
target_link_libraries(beckhoff_codec_benchmark PRIVATE BeckhoffPLC::beckhoff_plc)

if(BUILD_TESTING)
    add_executable(beckhoff_types_test tests/types_test.cpp)
    target_link_libraries(beckhoff_types_test PRIVATE BeckhoffPLC::beckhoff_plc)
    add_test(NAME types COMMAND beckhoff_types_test)
    add_executable(beckhoff_plc_test tests/plc_test.cpp)
    target_link_libraries(beckhoff_plc_test PRIVATE BeckhoffPLC::beckhoff_plc)
    add_test(NAME plc COMMAND beckhoff_plc_test)
    set_tests_properties(plc PROPERTIES TIMEOUT 15)
    if(BECKHOFF_PLC_WITH_ADS)
        add_executable(beckhoff_ads_transport_test tests/ads_transport_test.cpp src/ads_transport.cpp)
        target_compile_features(beckhoff_ads_transport_test PRIVATE cxx_std_17)
        target_include_directories(beckhoff_ads_transport_test PRIVATE include ${adslib_source_SOURCE_DIR}/AdsLib)
        target_link_libraries(beckhoff_ads_transport_test PRIVATE Threads::Threads)
        add_test(NAME ads_transport COMMAND beckhoff_ads_transport_test)
        set_tests_properties(ads_transport PROPERTIES TIMEOUT 15)
    endif()
    if(BECKHOFF_PLC_WITH_ADS AND BECKHOFF_PLC_ADS_INTEGRATION_TEST)
        find_package(Python3 REQUIRED COMPONENTS Interpreter)
        add_executable(beckhoff_ads_probe tests/ads_probe.cpp)
        target_link_libraries(beckhoff_ads_probe PRIVATE BeckhoffPLC::beckhoff_plc)
        add_test(NAME ads_integration COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/ads_integration.py $<TARGET_FILE:beckhoff_ads_probe>)
        set_tests_properties(ads_integration PROPERTIES TIMEOUT 30)
    endif()
endif()

install(TARGETS beckhoff_plc EXPORT BeckhoffPLCTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT BeckhoffPLCTargets NAMESPACE BeckhoffPLC::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/BeckhoffPLC)
configure_package_config_file(cmake/BeckhoffPLCConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/BeckhoffPLCConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/BeckhoffPLC)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/BeckhoffPLCConfigVersion.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/BeckhoffPLCConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/BeckhoffPLCConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/BeckhoffPLC)
