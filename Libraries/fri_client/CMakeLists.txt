cmake_minimum_required(VERSION 3.8)

add_library(FRIClient STATIC
  # base
  include/base/friClientApplication.h
  include/base/friClientData.h
  src/base/friClientApplication.cpp

  # client_lbr
  include/client_lbr/friClientIf.h
  include/client_lbr/friLBRClient.h
  include/client_lbr/friLBRCommand.h
  include/client_lbr/friLBRState.h
  src/client_lbr/friLBRClient.cpp
  src/client_lbr/friLBRCommand.cpp
  src/client_lbr/friLBRState.cpp

  # connection
  include/connection/friConnectionIf.h
  include/connection/friUdpConnection.h
  src/connection/friUdpConnection.cpp

  # nanopb-0.2.8
  include/nanopb-0.2.8/pb_decode.h
  include/nanopb-0.2.8/pb_encode.h
  include/nanopb-0.2.8/pb_syshdr_win.h
  include/nanopb-0.2.8/pb_syshdr.h
  include/nanopb-0.2.8/pb.h
  src/nanopb-0.2.8/pb_decode.c
  src/nanopb-0.2.8/pb_encode.c

  # protobuf
  include/protobuf/friCommandMessageEncoder.h
  include/protobuf/friMonitoringMessageDecoder.h
  include/protobuf/pb_frimessages_callbacks.h
  src/protobuf/friCommandMessageEncoder.cpp
  src/protobuf/friMonitoringMessageDecoder.cpp
  src/protobuf/pb_frimessages_callbacks.c

  # protobuf_gen
  include/protobuf_gen/FRIMessages.pb.h
  src/protobuf_gen/FRIMessages.pb.c
)

target_include_directories(FRIClient PUBLIC
  include/base
  include/client_lbr
  include/connection
  include/nanopb-0.2.8
  include/protobuf
  include/protobuf_gen
)

target_compile_features(FRIClient PUBLIC cxx_std_11)

target_compile_options(FRIClient PRIVATE 
  -O2 
  -Wall
  -DHAVE_SOCKLEN_T
  -DPB_SYSTEM_HEADER=\"pb_syshdr.h\"
  -DHAVE_STDINT_H
  -DHAVE_STDDEF_H
  -DHAVE_STDBOOL_H
  -DHAVE_STDLIB_H
  -DHAVE_STRING_H
)
