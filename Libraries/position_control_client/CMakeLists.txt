cmake_minimum_required(VERSION 3.8)

add_library(PositionControlClient STATIC)
add_library(PositionControlClient::PositionControlClient ALIAS PositionControlClient)

target_sources(PositionControlClient PRIVATE
  include/PositionControlClient/PositionControlClient.h
  src/PositionControlClient.cpp
)

target_include_directories(PositionControlClient PUBLIC 
  include/PositionControlClient
)
target_link_libraries(PositionControlClient PUBLIC FRI::Client)

