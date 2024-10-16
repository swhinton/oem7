cmake_minimum_required(VERSION 3.29)
project(oem7)

set(CMAKE_CXX_STANDARD 20)

include(cmake/CPM.cmake)

CPMAddPackage("gh:swhinton/zpp_bits#albedospace")

add_library(oem7
        include/novatel/oem7.hpp
        src/oem7.cpp
        include/novatel/crc.hpp)
target_link_libraries(oem7 zpp_bits)