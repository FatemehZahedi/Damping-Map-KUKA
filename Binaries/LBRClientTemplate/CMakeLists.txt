cmake_minimum_required(VERSION 3.8)

add_executable(LBRClientTemplate
  include/LBRClientTemplate/MyLBRClient.h
  src/MyLBRApp.cpp
  src/MyLBRClient.cpp
)

target_include_directories(LBRClientTemplate PUBLIC include)

target_link_libraries(LBRClientTemplate PUBLIC
  FRI::Client
)

