cmake_minimum_required(VERSION 3.8)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

add_executable(RunTrignoEmgClient
  include/TrignoEmgClient/TrignoEmgClient.h
  src/main.cpp
  src/TrignoEmgClient.cpp
)

target_include_directories(RunTrignoEmgClient PUBLIC include)

target_link_libraries(RunTrignoEmgClient PUBLIC
  CONAN_PKG::boost
  CONAN_PKG::hdf5
)