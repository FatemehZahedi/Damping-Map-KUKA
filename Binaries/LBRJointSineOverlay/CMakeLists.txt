cmake_minimum_required(VERSION 3.8)

add_executable(LBRJointSineOverlay
  include/LBRJointSineOverlay/LBRJointSineOverlayClient.h
  src/LBRJointSineOverlayApp.cpp
  src/LBRJointSineOverlayClient.cpp
)

target_include_directories(LBRJointSineOverlay PUBLIC include)

target_link_libraries(LBRJointSineOverlay PUBLIC
  FRI::Client
)

