cmake_minimum_required(VERSION 3.8)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

# create executables
add_executable(PositionControlApp2
  src/PositionControlApp2.cpp
)
target_link_libraries(PositionControlApp2 PUBLIC
  CONAN_PKG::eigen
  PositionControlClient::PositionControlClient
)

# ------------------------------------------------------
add_executable(AdmittanceControl_test_mine
  src/AdmittanceControl_test_mine.cpp
)
target_link_libraries(AdmittanceControl_test_mine PUBLIC
  CONAN_PKG::eigen
  PositionControlClient::PositionControlClient
)

# ------------------------------------------------------
add_executable(Ad
  src/Ad.cpp
)
target_link_libraries(Ad PUBLIC
  CONAN_PKG::eigen
  PositionControlClient::PositionControlClient
)

# ------------------------------------------------------
add_executable(PositionControlApp_Ad
  src/PositionControlApp_Ad.cpp
)
target_link_libraries(PositionControlApp_Ad PUBLIC
  CONAN_PKG::eigen
  PositionControlClient::PositionControlClient
)

# ------------------------------------------------------
add_executable(AdmittanceControl_GUI
  src/AdmittanceControl_GUI.cpp
)
target_link_libraries(AdmittanceControl_GUI PUBLIC
  CONAN_PKG::eigen
  PositionControlClient::PositionControlClient
  UdpServer::UdpServer
)

# ------------------------------------------------------
add_executable(Hybrid
  src/Hybrid.cpp
)
target_link_libraries(Hybrid PUBLIC
  CONAN_PKG::eigen
  PositionControlClient::PositionControlClient
)

# ------------------------------------------------------
add_executable(run_udp_server
  src/run_udp_server.cpp
)
target_link_libraries(run_udp_server PUBLIC
  UdpServer::UdpServer
)

# ------------------------------------------------------
add_executable(Ad_pert_wholetrial_2
  src/Ad_pert_wholetrial_2.cpp
)
target_link_libraries(Ad_pert_wholetrial_2 PUBLIC
  CONAN_PKG::eigen
  PositionControlClient::PositionControlClient
)

# ------------------------------------------------------
add_executable(PositionControlApp_out
  src/PositionControlApp_out.cpp
)
target_link_libraries(PositionControlApp_out PUBLIC
  CONAN_PKG::eigen
  PositionControlClient::PositionControlClient
)

# ------------------------------------------------------
add_executable(PositionControlApp
  src/PositionControlApp.cpp
)
target_link_libraries(PositionControlApp PUBLIC
  CONAN_PKG::eigen
  PositionControlClient::PositionControlClient
  UdpServer::UdpServer
)

# ------------------------------------------------------
add_executable(PositionControlApp_ex
  src/PositionControlApp_ex.cpp
)
target_link_libraries(PositionControlApp_ex PUBLIC
  CONAN_PKG::eigen
  PositionControlClient::PositionControlClient
)

# ------------------------------------------------------
add_executable(Perturb
  src/Perturb.cpp
)
target_link_libraries(Perturb PUBLIC
  CONAN_PKG::eigen
  PositionControlClient::PositionControlClient
)

# ------------------------------------------------------
add_executable(Perturbation
  src/Perturbation.cpp
)
target_link_libraries(Perturbation PUBLIC
  CONAN_PKG::eigen
  PositionControlClient::PositionControlClient
)