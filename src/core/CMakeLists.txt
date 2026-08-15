add_library(core_lib STATIC
    data.cpp
    conf_parser.cpp
)

target_include_directories(core_lib PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(core_lib PUBLIC
    yaml-cpp
)
