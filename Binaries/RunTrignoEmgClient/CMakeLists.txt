cmake_minimum_required(VERSION 3.8)

add_executable(RunTrignoEmgClient
  src/main.cpp
)

target_link_libraries(RunTrignoEmgClient PUBLIC
  TrignoEmgClient::TrignoEmgClient
)