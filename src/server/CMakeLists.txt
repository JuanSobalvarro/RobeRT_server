cmake_minimum_required(VERSION 3.8)
project(robert_server)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Protobuf REQUIRED)

# compile protos
# This looks for the file inside your new folder structure
# It creates variables for the generated .cc (SRCS) and .h (HDRS) files
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS proto/egm.proto)

add_executable(robert_server 
  src/main.cpp 
  ${PROTO_SRCS} 
  ${PROTO_HDRS}
)

message(STATUS "Proto headers will be in: ${CMAKE_CURRENT_BINARY_DIR}")

target_include_directories(robert_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${Protobuf_INCLUDE_DIRS}
  "${CMAKE_CURRENT_BINARY_DIR}"        # Root of build
  "${CMAKE_CURRENT_BINARY_DIR}/proto"  # Explicit path to generated headers
)

target_link_libraries(robert_server 
  ${PROTOBUF_LIBRARIES}
)
ament_target_dependencies(robert_server rclcpp)

install(TARGETS robert_server
  DESTINATION lib/${PROJECT_NAME})

ament_package()