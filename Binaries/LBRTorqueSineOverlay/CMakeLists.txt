cmake_minimum_required(VERSION 3.8)

add_executable(LBRTorqueSineOverlay
  include/LBRTorqueSineOverlay/LBRTorqueSineOverlayClient.h
  src/LBRTorqueSineOverlayApp.cpp
  src/LBRTorqueSineOverlayClient.cpp
)

target_include_directories(LBRTorqueSineOverlay PUBLIC include)

target_link_libraries(LBRTorqueSineOverlay PUBLIC
  FRI::Client
)