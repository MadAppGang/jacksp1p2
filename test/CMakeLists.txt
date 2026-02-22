cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../components/p1p2_bus"
    "${CMAKE_CURRENT_LIST_DIR}/../components/p1p2_protocol"
    "${CMAKE_CURRENT_LIST_DIR}/../components/p1p2_network"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(p1p2_test)
