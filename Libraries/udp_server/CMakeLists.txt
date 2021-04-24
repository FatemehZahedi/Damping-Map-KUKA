cmake_minimum_required(VERSION 3.8)

add_library(UdpServer STATIC)
add_library(UdpServer::UdpServer ALIAS UdpServer)

target_sources(UdpServer PRIVATE
  include/UdpServer/UdpServer.h
  src/UdpServer.cpp
)

target_include_directories(UdpServer PUBLIC include)