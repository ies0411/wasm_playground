project(FLANN)

add_library(FLANN INTERFACE)

set(FLANN_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
set(FLANN_INCLUDE_PATH "${FLANN_PATH}/include")
set(FLANN_LIB_PATH "${OPENCV_PATH_PATH}/lib")

set(lib_FLANN
    ${FLANN_LIB_PATH}/libflann_cpp_s.a
    ${FLANN_LIB_PATH}/libflann_cpp.a)

target_link_libraries(FLANN INTERFACE ${lib_FLANN})
target_include_directories(FLANN INTERFACE ${FLANN_INCLUDE_PATH})
