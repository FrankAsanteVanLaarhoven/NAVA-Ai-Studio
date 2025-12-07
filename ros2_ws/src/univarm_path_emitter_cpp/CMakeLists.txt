cmake_minimum_required(VERSION 3.16)
project(univarm_path_emitter_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(univarm_msgs REQUIRED)
find_package(Protobuf REQUIRED)
# Require gRPC CMake config (provided by libgrpc++-dev)
find_package(gRPC CONFIG REQUIRED)

# Proto input
set(PROTO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../proto/univarm)
set(PROTO_FILE ${PROTO_DIR}/drive.proto)

# Generated sources will be in the build tree
set(GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${GEN_DIR})

# Invoke protoc for protobuf and gRPC
get_target_property(_PROTOBUF_PROTOC_EXECUTABLE protobuf::protoc IMPORTED_LOCATION)
get_target_property(_GRPC_CPP_PLUGIN_EXECUTABLE gRPC::grpc_cpp_plugin IMPORTED_LOCATION)

add_custom_command(
  OUTPUT ${GEN_DIR}/drive.pb.cc ${GEN_DIR}/drive.pb.h
  COMMAND ${_PROTOBUF_PROTOC_EXECUTABLE}
    --cpp_out=${GEN_DIR}
    -I ${PROTO_DIR}
    ${PROTO_FILE}
  DEPENDS ${PROTO_FILE}
  VERBATIM)

add_custom_command(
  OUTPUT ${GEN_DIR}/drive.grpc.pb.cc ${GEN_DIR}/drive.grpc.pb.h
  COMMAND ${_PROTOBUF_PROTOC_EXECUTABLE}
    --grpc_out=${GEN_DIR}
    --plugin=protoc-gen-grpc=${_GRPC_CPP_PLUGIN_EXECUTABLE}
    -I ${PROTO_DIR}
    ${PROTO_FILE}
  DEPENDS ${PROTO_FILE} ${GEN_DIR}/drive.pb.h
  VERBATIM)

add_library(univarm_drive_proto STATIC
  ${GEN_DIR}/drive.pb.cc
  ${GEN_DIR}/drive.grpc.pb.cc)
target_include_directories(univarm_drive_proto PUBLIC ${GEN_DIR})
target_link_libraries(univarm_drive_proto PUBLIC protobuf::libprotobuf gRPC::grpc++)

add_executable(path_emitter src/path_emitter.cpp)
target_include_directories(path_emitter PRIVATE ${GEN_DIR})
target_link_libraries(path_emitter PRIVATE
  univarm_drive_proto
  rclcpp::rclcpp
  nav_msgs::nav_msgs__rosidl_typesupport_cpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp)
ament_target_dependencies(path_emitter rclcpp nav_msgs geometry_msgs univarm_msgs)

install(TARGETS path_emitter
  DESTINATION lib/${PROJECT_NAME})

ament_package()
