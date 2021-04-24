cmake_minimum_required(VERSION 3.8)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)


add_executable(VariableDampingControl
  include/VariableDampingControl/H5Functions.h
  include/VariableDampingControl/SubjectMetadataParser.h
  src/KukaVariableDamping.cpp
  src/SubjectMetadataParser.cpp
)

target_include_directories(VariableDampingControl PUBLIC include)

target_link_libraries(VariableDampingControl PUBLIC
  CONAN_PKG::hdf5
  CONAN_PKG::boost
  CONAN_PKG::eigen
  UdpServer::UdpServer
  PositionControlClient::PositionControlClient
  TrignoEmgClient::TrignoEmgClient
)