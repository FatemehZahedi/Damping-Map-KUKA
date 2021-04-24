cmake_minimum_required(VERSION 3.8)

add_executable(LBRWrenchSineOverlay
  include/LBRWrenchSineOverlay/LBRWrenchSineOverlayClient.h
  src/LBRWrenchSineOverlayApp.cpp
  src/LBRWrenchSineOverlayClient.cpp
)

target_include_directories(LBRWrenchSineOverlay PUBLIC include)

target_link_libraries(LBRWrenchSineOverlay PUBLIC
  FRI::Client
)

