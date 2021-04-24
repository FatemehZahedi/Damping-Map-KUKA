cmake_minimum_required(VERSION 3.8)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

add_library(TrignoEmgClient STATIC)
add_library(TrignoEmgClient::TrignoEmgClient ALIAS TrignoEmgClient)

target_sources(TrignoEmgClient PRIVATE
  include/TrignoEmgClient/TrignoEmgClient.h
  src/TrignoEmgClient.cpp
)

target_include_directories(TrignoEmgClient PUBLIC include)

target_link_libraries(TrignoEmgClient PUBLIC
  CONAN_PKG::boost
  CONAN_PKG::hdf5
)