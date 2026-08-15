add_library(network_lib STATIC
    sock_comm.cpp
)

target_include_directories(network_lib PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

IF (WIN32)
    target_link_libraries(network_lib PUBLIC ws2_32 iphlpapi)
ENDIF()
