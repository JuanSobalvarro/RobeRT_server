cmake_minimum_required(VERSION 3.10)
project(robert)

set(CMAKE_CXX_STANDARD 17)

# Find Google Protocol Buffers
find_package(Protobuf REQUIRED)

# Include the generated files
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Automatically generate .h and .cc files from egm.proto
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS proto/egm.proto)

# Create the executable
add_executable(egm_server src/main.cpp ${PROTO_SRCS})

# Link dependencies
target_link_libraries(egm_server ${Protobuf_LIBRARIES} pthread)