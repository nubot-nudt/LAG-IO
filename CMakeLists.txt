cmake_minimum_required(VERSION 3.16)
project(LAG_IO VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Fixed-lag smoothing is provided by gtsam_unstable in GTSAM 4.2.
set(GTSAM_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GTSAM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GTSAM_BUILD_UNSTABLE ON CACHE BOOL "" FORCE)
set(GTSAM_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GTSAM_BUILD_WRAP OFF CACHE BOOL "" FORCE)
set(GTSAM_INSTALL_CPPUNITLITE OFF CACHE BOOL "" FORCE)
set(GTSAM_WITH_TBB OFF CACHE BOOL "" FORCE)
set(GTSAM_WITH_EIGEN_MKL OFF CACHE BOOL "" FORCE)
set(GTSAM_USE_SYSTEM_EIGEN OFF CACHE BOOL "" FORCE)
add_subdirectory(ThirdParty/gtsam-4.2.0 EXCLUDE_FROM_ALL)

set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
add_subdirectory(ThirdParty/yaml-cpp-0.7.0 EXCLUDE_FROM_ALL)
set(ABSL_PROPAGATE_CXX_STD ON)
set(ABSL_BUILD_TESTING OFF)
add_subdirectory(ThirdParty/abseil-cpp-20220623.1 EXCLUDE_FROM_ALL)

add_executable(lag_io
  src/main.cpp
  src/config/configuration.cpp
  src/fileio/fileloader.cpp
  src/fileio/filesaver.cpp
  src/graph/GraphRunner.cpp
  src/graph/IMUTrack.cpp
  src/graph/system.cpp
)
target_include_directories(lag_io PRIVATE src)
target_link_libraries(lag_io PRIVATE gtsam gtsam_unstable yaml-cpp
  absl::strings absl::str_format absl::time)
if(MSVC)
  target_compile_definitions(lag_io PRIVATE _USE_MATH_DEFINES)
  target_compile_options(lag_io PRIVATE /utf-8 /W4)
else()
  target_compile_options(lag_io PRIVATE -Wall -Wextra -Wpedantic -Wno-deprecated-declarations)
endif()
