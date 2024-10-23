cmake_minimum_required(VERSION 3.25)
set(CMAKE_CXX_STANDARD 20)

project(OEM7
        VERSION 1.0.0
        DESCRIPTION "Novatel OEM7 binary command/telemetry interface library."
        LANGUAGES CXX C
)
include(GNUInstallDirs)
include(GenerateExportHeader)
include(CMakePackageConfigHelpers)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

add_compile_options("$<$<CONFIG:Debug>:-fno-omit-frame-pointer>")
add_compile_options("$<$<CONFIG:Debug>:-ggdb3>")
add_compile_options("$<$<CONFIG:Debug>:-O0>")
add_compile_options("$<$<CONFIG:Debug>:-g3>")
set(CMAKE_DEBUG_POSTFIX -dbg)

option(OEM7_LIB_BUILD_TESTS "Build unit tests." ON)
option(OEM7_LIB_TEST_COVERAGE "Generate instrumented test coverage executable." OFF)

include(FindCPM)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

# Need to guard against this to avoid dependency conflicts with CPM.
if (${PROJECT_IS_TOP_LEVEL})
    include(OEM7LibDependencies)
endif ()

include_directories(${PROJECT_SOURCE_DIR}/include)
add_library(${PROJECT_NAME}
        src/protocol.cpp
)
target_include_directories(${PROJECT_NAME}
        INTERFACE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(${PROJECT_NAME}
        PUBLIC
        tl::expected
        zpp_bits::zpp_bits
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# =============================
# Test build
# =============================
if (${OEM7_LIB_BUILD_TESTS})
    include(CTest)
    if (${OEM7_LIB_TEST_COVERAGE})
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
            "OEM7Config.cmake"
            INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OEM7
            PATH_VARS
            CMAKE_INSTALL_LIBDIR
    )
    write_basic_package_version_file(
            ${CMAKE_CURRENT_BINARY_DIR}/OEM7ConfigVersion.cmake
            VERSION ${CMAKE_PROJECT_VERSION}
            COMPATIBILITY SameMinorVersion
    )
    install(
            FILES "${CMAKE_CURRENT_BINARY_DIR}/OEM7Config.cmake"
            "${CMAKE_CURRENT_BINARY_DIR}/OEM7ConfigVersion.cmake"
            DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/OEM7"
    )
    install(
            DIRECTORY ${PROJECT_SOURCE_DIR}/include
            DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )
endif ()
