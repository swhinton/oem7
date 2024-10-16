set(FMT_INSTALL ON)
CPMAddPackage("gh:fmtlib/fmt#11.0.2")

set(SPDLOG_FMT_EXTERNAL ON CACHE INTERNAL "Use external fmt library")
set(SPDLOG_INSTALL ON)
CPMAddPackage("gh:gabime/spdlog#v1.14.1")

CPMAddPackage(
        NAME magic_enum
        GITHUB_REPOSITORY Neargye/magic_enum
        GIT_TAG v0.9.5
)

CPMAddPackage(
        NAME googletest
        GITHUB_REPOSITORY google/googletest
        VERSION 1.14.0
        OPTIONS
        "INSTALL_GTEST OFF"
        "gtest_force_shared_crt ON"
)
add_library(gtest::gtest_main ALIAS gtest_main)
add_library(gmock::gmock_main ALIAS gmock_main)
enable_testing()
include(GoogleTest)
include(CTest)

set(TEST_NAME ${PROJECT_NAME}-unit-test)
add_executable(${TEST_NAME}
        test_oem7.cpp
)
target_include_directories(${TEST_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(${TEST_NAME}
        ${PROJECT_NAME}::${PROJECT_NAME}
        spdlog::spdlog
        magic_enum::magic_enum
        gtest::gtest_main
)
