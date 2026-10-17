# Provides the target `beckhoff_plc_ads`: Beckhoff's ADS library
# (https://github.com/Beckhoff/ADS), the same library pyads uses on Linux.
#
# By default the pinned version below is downloaded and compiled as a static
# library. Alternatives:
#  - offline builds: -DFETCHCONTENT_SOURCE_DIR_BECKHOFF_ADS=/path/to/ADS (a checkout)
#  - an installed upstream library: -DBECKHOFF_PLC_USE_SYSTEM_ADS=ON (find_package(Ads))
#
# The upstream CMakeLists.txt needs CMake >= 3.23 and builds tools and tests,
# so the library sources are compiled here instead.

option(BECKHOFF_PLC_USE_SYSTEM_ADS "Use an installed Beckhoff ADS library (find_package(Ads)) instead of the pinned one" OFF)

set(BECKHOFF_PLC_ADS_FIND_DEPENDENCY "")

if(BECKHOFF_PLC_USE_SYSTEM_ADS)
  find_package(Ads CONFIG REQUIRED)
  add_library(beckhoff_plc_ads INTERFACE)
  target_link_libraries(beckhoff_plc_ads INTERFACE Ads::AdsLib)
  set(BECKHOFF_PLC_ADS_FIND_DEPENDENCY "find_dependency(Ads)")
  return()
endif()

include(FetchContent)
FetchContent_Declare(beckhoff_ads
  GIT_REPOSITORY https://github.com/Beckhoff/ADS.git
  GIT_TAG 24e598a113edfdc0d34414bfefb847930d94cb87  # 113.0.36-1
  # Only download: the sources are compiled below, not with the upstream CMakeLists.txt.
  SOURCE_SUBDIR _beckhoff_plc_sources_only
)
FetchContent_MakeAvailable(beckhoff_ads)

set(_ads_dir "${beckhoff_ads_SOURCE_DIR}/AdsLib")
if(NOT EXISTS "${_ads_dir}/standalone/AdsLib.cpp")
  message(FATAL_ERROR "Beckhoff ADS sources not found in ${beckhoff_ads_SOURCE_DIR}")
endif()

# Standalone (non-TwinCAT router) variant: what is needed for ADS client
# communication over TCP and to add routes.
add_library(beckhoff_plc_ads STATIC
  ${_ads_dir}/AdsDef.cpp
  ${_ads_dir}/AdsLib.cpp
  ${_ads_dir}/Frame.cpp
  ${_ads_dir}/Log.cpp
  ${_ads_dir}/Sockets.cpp
  ${_ads_dir}/standalone/AdsLib.cpp
  ${_ads_dir}/standalone/AmsConnection.cpp
  ${_ads_dir}/standalone/AmsNetId.cpp
  ${_ads_dir}/standalone/AmsPort.cpp
  ${_ads_dir}/standalone/AmsRouter.cpp
  ${_ads_dir}/standalone/NotificationDispatcher.cpp
)
target_include_directories(beckhoff_plc_ads SYSTEM PUBLIC $<BUILD_INTERFACE:${_ads_dir}>)
target_compile_features(beckhoff_plc_ads PUBLIC cxx_std_14)
target_compile_definitions(beckhoff_plc_ads PRIVATE CONFIG_DEFAULT_LOGLEVEL=2)  # warnings: level 1 also reports clean disconnects
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_definitions(beckhoff_plc_ads PRIVATE _GNU_SOURCE)
elseif(MSVC)
  target_compile_definitions(beckhoff_plc_ads PRIVATE NOMINMAX _CRT_SECURE_NO_WARNINGS _CRT_NONSTDC_NO_WARNINGS)
endif()
set_target_properties(beckhoff_plc_ads PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(beckhoff_plc_ads PUBLIC Threads::Threads)
if(WIN32)
  target_link_libraries(beckhoff_plc_ads PUBLIC ws2_32)
endif()
