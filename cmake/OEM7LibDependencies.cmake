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