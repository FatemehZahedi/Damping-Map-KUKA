cmake_minimum_required(VERSION 3.8)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

add_executable(DampingMap
  src/KukaDampingMap.cpp
)

target_link_libraries(DampingMap PUBLIC
  CONAN_PKG::eigen
  CONAN_PKG::boost
  UdpServer::UdpServer
  TrignoEmgClient::TrignoEmgClient
  PositionControlClient::PositionControlClient
)

