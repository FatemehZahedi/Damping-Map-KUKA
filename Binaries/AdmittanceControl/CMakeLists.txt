cmake_minimum_required(VERSION 3.8)

add_executable(AdmittanceControl
  src/AdmittanceControl.cpp
)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

target_include_directories(AdmittanceControl PUBLIC include)

target_link_libraries(AdmittanceControl PUBLIC
  FRI::Client
  UdpServer::UdpServer
  CONAN_PKG::eigen
  CONAN_PKG::boost
  CONAN_PKG::hdf5
  PositionControlClient::PositionControlClient
)