cmake_minimum_required(VERSION 3.5)
project(calib_nextjs_test)
find_package(GTest REQUIRED)

enable_testing()

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(
  unitTests
  ${TEST_DIR}/gtest_main.cpp # Main entry for unit tests
  ${TEST_DIR}/intrinsic_test.cpp
)

# TODO : change Unittest wasm config
target_link_libraries(
  unitTests
  GTest::gtest
  opencv
  ${CERES_LIBRARIES}
)

set_target_properties(unitTests
  PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "../"
)

add_test(NAME unit COMMAND ${CMAKE_BINARY_DIR}/unitTests)
