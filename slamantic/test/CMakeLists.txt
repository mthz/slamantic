#
# copyright Matthias Schoerghuber (AIT)
#
include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        release-1.8.0
)

FetchContent_GetProperties(googletest)
if(NOT googletest_POPULATED)
  FetchContent_Populate(googletest)
  add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})
endif()

add_executable(slamantic_test test_slamantic.cpp src/dynamicsFactor.cpp src/patchLabel.cpp src/semanticImage.cpp src/semantic.cpp src/semanticMask.cpp)
target_link_libraries(slamantic_test slamantic gtest)
target_compile_definitions(slamantic_test PUBLIC SLAMANTIC_COMPILE_TEST_DATA="${CMAKE_CURRENT_SOURCE_DIR}")