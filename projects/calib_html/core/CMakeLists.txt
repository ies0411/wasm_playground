cmake_minimum_required(VERSION 3.5)
project(calibHTML)

set(PROJECT_NAME wasmCalib)
set(CMAKE_CXX_STANDARD 17)
set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)

set(OPENCV ../../../thirdparty/opencv)
set(BUILD_OPENCV_DIR ./build_opencv)

set(PCL ../../../thirdparty/pcl.js/core/pcl/build)
set(PCL_LIB ../../../thirdparty/pcl.js/core/pcl/build/lib)
set(PCL_INCLUDE ${PCL}/include/pcl-1.12)
set(BUILD_PCL_DIR ./build_pcl)

set(FLANN ../../../thirdparty/pcl.js/core/pcl/wasm/deps/flann/build)
set(FLANN_INCLUDE ${FLANN}/include)
set(FLANN_LIB ${FLANN}/lib)

# set(EIGEN ../../../thirdparty/pcl.js/core/pcl/wasm/deps/eigen)
set(BOOST ../../../thirdparty/pcl.js/core/pcl/wasm/deps/boost/build)
set(BOOST_INCLUDE ${BOOST}/include)
set(BOOST_LIB ${BOOST}/lib)
set(BUILD_BOOST_DIR ./build_boost)

set(EMSDK_FLAGS
    "-s WASM=1\
    -s USE_ES6_IMPORT_META=1\
    -s EXPORT_ES6=1\
    -O3\
    -s EXPORT_NAME='createModule'\
    -s EXPORTED_FUNCTIONS='[\"_malloc\", \"_free\"]'\
    -s EXTRA_EXPORTED_RUNTIME_METHODS='[\"ccall\",\"cwrap\"]'\
    -s EXPORTED_RUNTIME_METHODS='FS'\
    -s ENVIRONMENT='web,webview,worker'\
    -s ALLOW_MEMORY_GROWTH=1\
    -s FORCE_FILESYSTEM=1\
    ")

# -s ENVIRONMENT='web'\
set(CMAKE_CXX_FLAGS)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EMSDK_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EMSDK_FLAGS}")

# add_subdirectory(tflite)
find_package(Ceres REQUIRED)

# -s Release\
# MEMO : ceres lib cmake에서 find_package(thread) 삭제
add_subdirectory(${OPENCV} ${BUILD_OPENCV_DIR})
add_subdirectory(${PCL} ${BUILD_PCL_DIR})
add_subdirectory(${BOOST} ${BUILD_BOOST_DIR})

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CERES_INCLUDE_DIRS}
    ${BOOST_INCLUDE}
    ${PCL_INCLUDE}
    ${FLANN_INCLUDE}
    ${EIGEN_INCLUDE}
)
link_directories(
    ${PCL_LIB}
    ${FLANN_LIB}
    ${BOOST_LIB}
)

file(GLOB PCL_LINK_LIBS ${PCL_LIB}/*.a)
file(GLOB SRCS *.cpp)

add_executable(extrinsicCalib
    ${SRC_DIR}/cam_lidar_calib_node.cpp
)

add_executable(intrinsicCalib
    ${SRC_DIR}/cam_intrinsic.cpp
)

add_executable(stereoCalib
    ${SRC_DIR}/cam_stereo.cpp
)

# add_library()
target_link_libraries(extrinsicCalib PCL_INTERFACE ${PCL_LINK_LIBS} opencv ${CERES_LIBRARIES} flann_cpp_s boost_filesystem boost_regex boost_system boost_date_time boost_iostreams)
target_link_libraries(intrinsicCalib opencv ${CERES_LIBRARIES})
target_link_libraries(stereoCalib opencv ${CERES_LIBRARIES})
