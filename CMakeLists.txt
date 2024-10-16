cmake_minimum_required(VERSION 3.24)

project(oem7
        VERSION 1.0.0
        DESCRIPTION "Novatel OEM7 binary command/telemetry interface library."
        LANGUAGES CXX C
)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

include(cmake/CPM.cmake)

CPMAddPackage("gh:swhinton/zpp_bits#albedospace")

if (CMAKE_CXX_STANDARD LESS 23)
    set(EXPECTED_BUILD_TESTS OFF)
    list(APPEND CMAKE_MESSAGE_CONTEXT " tl::expected")
    CPMAddPackage(
            NAME tl-expected
            GITHUB_REPOSITORY TartanLlama/expected
            GIT_TAG v1.1.0
    )
    list(POP_BACK CMAKE_MESSAGE_CONTEXT)
endif ()

add_library(${PROJECT_NAME}
        src/protocol.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE include)

list(APPEND LIB_DEPS zpp_bits)

if (CMAKE_CXX_STANDARD LESS 23)
    list(APPEND LIB_DEPS tl::expected)
endif ()

target_link_libraries(${PROJECT_NAME} PUBLIC ${LIB_DEPS})

target_include_directories(${PROJECT_NAME}
        INTERFACE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# =============================
# Test build
# =============================
if (${${PROJECT_NAME}_BUILD_TESTS})
    include(CTest)
    if (${${PROJECT_NAME}_TEST_COVERAGE})
        add_compile_options("$<$<CONFIG:Debug>:--coverage>")
        add_link_options("$<$<CONFIG:Debug>:--coverage>")
    endif ()
    enable_testing()
    add_subdirectory(test)
endif ()

# =============================
# Package export
# =============================
if (${PROJECT_IS_TOP_LEVEL})
    include(CMakePackageConfigHelpers)
    configure_package_config_file(
            "${PROJECT_SOURCE_DIR}/cmake/Config.cmake.in"
            "oem7Config.cmake"
            INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/oem7
            PATH_VARS
            CMAKE_INSTALL_LIBDIR
    )
    write_basic_package_version_file(
            ${CMAKE_CURRENT_BINARY_DIR}/oem7ConfigVersion.cmake
            VERSION ${CMAKE_PROJECT_VERSION}
            COMPATIBILITY SameMinorVersion
    )
    install(
            FILES "${CMAKE_CURRENT_BINARY_DIR}/oem7Config.cmake"
            "${CMAKE_CURRENT_BINARY_DIR}/oem7ConfigVersion.cmake"
            DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/oem7"
    )
    install(
            DIRECTORY ${PROJECT_SOURCE_DIR}/include
            DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )
endif ()
