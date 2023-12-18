cmake_minimum_required(VERSION 3.5)
project(calibBext)

set(PROJECT_NAME wasmCalib)
set(CMAKE_CXX_STANDARD 17)
set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)

set(OPENCV ../../../thirdparty/opencv)
set(BUILD_OPENCV_DIR ./build_opencv)

set(PCL ../../../thirdparty/pcl/build)
set(PCL_LIB ../../../thirdparty/pcl/build/lib)
set(PCL_INCLUDE ${PCL}/include/pcl-1.12)
set(BUILD_PCL_DIR ./build_pcl)

set(FLANN ../../../thirdparty/flann/build)
set(FLANN_INCLUDE ${FLANN}/include)
set(FLANN_LIB ${FLANN}/lib)

set(BOOST ../../../thirdparty/boost/build)
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

add_executable(calibration
    ${SRC_DIR}/calibration.cpp ${SRC_DIR}/camera_intrinsic.cpp ${SRC_DIR}/camera_lidar.cpp
)

# add_executable(intrinsicCalib
# ${SRC_DIR}/cam_intrinsic.cpp
# )

# add_executable(stereoCalib
# ${SRC_DIR}/cam_stereo.cpp
# )

# add_library()
target_link_libraries(calibration PCL_INTERFACE ${PCL_LINK_LIBS} opencv ${CERES_LIBRARIES} flann_cpp_s boost_filesystem boost_regex boost_system boost_date_time boost_iostreams)

# install(
# TARGETS calibration DESTINATION ${CMAKE_INSTALL_PREFIX}/intrinsic

# # TARGETS extrinsic DESTINATION ${CMAKE_INSTALL_PREFIX}/ext_camlidar

# # TARGETS extrinsic_stereo DESTINATION ${CMAKE_INSTALL_PREFIX}/stereo
# )
# install(
# FILES ${CORE_DIR}/calibration.wasm DESTINATION ${CMAKE_INSTALL_PREFIX}/intrinsic

# # FILES ${CORE_DIR}/extrinsic.wasm DESTINATION ${CMAKE_INSTALL_PREFIX}/ext_camlidar

# FILES ${CORE_DIR}/build/extrinsic_stereo.wasm DESTINATION ${CMAKE_INSTALL_PREFIX}/stereo
# )

# target_link_libraries(intrinsicCalib opencv ${CERES_LIBRARIES})
# target_link_libraries(stereoCalib opencv ${CERES_LIBRARIES})

# cmake_minimum_required(VERSION 3.4)

# # MEMO : docker is better about importing 3rd-party
# set(CMAKE_CXX_STANDARD 17)
# set(WASMPLAYGROUND_VERSION_MAJOR 0)
# set(WASMPLAYGROUND_VERSION_MINOR 1)
# set(WASMPLAYGROUND_VERSION_PATCH 1)
# set(WASMPLAYGROUND_VERSION_STRING "${WASMPLAYGROUND_VERSION_MAJOR}.${WASMPLAYGROUND_VERSION_MINOR}.${WASMPLAYGROUND_VERSION_PATCH}")

# if(CMAKE_SYSTEM_NAME STREQUAL Emscripten)
# set(BUILD_WASM ON)
# else()
# set(BUILD_WASM OFF)
# endif()

# # OPTIONS - thirdparty lib
# # Debug print option
# option(USE_SYSTEM_EIGEN3 "use eigen" ON)
# option(USE_SYSTEM_GTEST "use ceres" ON)
# option(USE_SYSTEM_GLOG "use glog" ON)
# option(USE_SYSTEM_CERES "use ceres" ON)

# if(USE_SYSTEM_CERES AND USE_SYSTEM_GLOG EQUAL OFF)
# option(USE_SYSTEM_GLOG "use glog" ON)
# endif()

# # project
# project(
# superb3d
# VERSION ${WASMPLAYGROUND_VERSION_STRING}
# DESCRIPTION "Superb wasm-playground"
# LANGUAGES C CXX
# )

# # #####
# message(STATUS "=====================================")
# message(STATUS " Superb WasmPlayground")
# message(STATUS " version ${WASMPLAYGROUND_VERSION_STRING}")

# # #####
# add_compile_options(-Wall)

# # wasm flag
# set(EMSDK_FLAGS
# "-s WASM=1\
# -s USE_ES6_IMPORT_META=1\
# -s EXPORT_ES6=1\
# -O3\
# -s EXPORT_NAME='createModule'\
# -s EXPORTED_FUNCTIONS='[\"_malloc\", \"_free\"]'\
# -s EXTRA_EXPORTED_RUNTIME_METHODS='[\"ccall\",\"cwrap\"]'\
# -s EXPORTED_RUNTIME_METHODS='FS'\
# -s ENVIRONMENT='web,webview,worker'\
# -s ALLOW_MEMORY_GROWTH=1\
# -s FORCE_FILESYSTEM=1\
# ")

# set(TEST_EMSDK_FLAGS
# "-O3")

# # cmake module
# list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake_modules)
# include(set_path)
# include(third_party)
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ")

# # third-party

# # INLUCDE
# include_directories(
# ${CMAKE_SOURCE_DIR}/include
# ${CERES_INCLUDE_DIRS}
# ${BOOST_INCLUDE}
# ${PCL_INCLUDE}
# ${FLANN_INCLUDE}
# ${EIGEN3_INCLUDE_PATH}
# )
# link_directories(
# ${PCL_LIB}
# ${FLANN_LIB}
# ${BOOST_LIB}
# )

# # message(STATUS ${EIGEN_INCLUDE_DIRS_REV})
# # message(${EIGEN3_INCLUDE_PATH})
# # include_directories(
# # ${EIGEN_INCLUDE_DIRS_REV}
# # )

# # add_executable(extrinsic_stereo
# # ${SRC_DIR}/cam_stereo.cpp
# # )
# # add_executable(intrinsic
# # ${SRC_DIR}/camera_intrinsic.cpp
# # )

# # add_executable(extrinsic
# # ${SRC_DIR}/camera_lidar.cpp
# # )
# add_executable(calibration
# ${SRC_DIR}/calibration.cpp ${SRC_DIR}/camera_intrinsic.cpp ${SRC_DIR}/camera_lidar.cpp
# )

# # target_link_libraries(extrinsic PCL_INTERFACE ${PCL_LIB} opencv ${CERES_LIBRARIES} ${FLANN_LIB} ${BOOST_LIB})
# # target_link_libraries(extrinsic PCL_INTERFACE ${PCL_LIB} opencv ${CERES_LIBRARIES} flann_cpp_s boost_filesystem boost_regex boost_system boost_date_time boost_iostreams)
# target_link_libraries(calibration PCL_INTERFACE ${PCL_LIB} opencv ${CERES_LIBRARIES} flann_cpp_s boost_filesystem boost_regex boost_system boost_date_time boost_iostreams)

# # set_target_properties(extrinsic PROPERTIES LINK_FLAGS ${EMSDK_FLAGS})

# # message(${EIGEN_INCLUDE_DIRS})

# # ${PCL_LINK_LIBS}
# # boost_filesystem boost_regex boost_system boost_date_time boost_iostreams flann_cpp_s
# # target_link_libraries(intrinsic PRIVATE opencv)

# # set_target_properties(intrinsic PROPERTIES LINK_FLAGS ${EMSDK_FLAGS})

# # target_link_libraries(extrinsic_stereo opencv ${CERES_LIBRARIES})

# # Gtest
# # message("Gtest..")
# # add_subdirectory(test)
# # set_target_properties(unitTests
# # PROPERTIES
# # RUNTIME_OUTPUT_DIRECTORY "./"
# # )

# # add_test(NAME unit COMMAND ${CMAKE_BINARY_DIR}/unitTests)

# # INSTALL
# # install(
# # TARGETS intrinsic DESTINATION ${CMAKE_INSTALL_PREFIX}/intrinsic

# # # TARGETS extrinsic DESTINATION ${CMAKE_INSTALL_PREFIX}/ext_camlidar

# # # TARGETS extrinsic_stereo DESTINATION ${CMAKE_INSTALL_PREFIX}/stereo
# # )
# # install(
# # FILES ${CORE_DIR}/intrinsic.wasm DESTINATION ${CMAKE_INSTALL_PREFIX}/intrinsic

# # # FILES ${CORE_DIR}/extrinsic.wasm DESTINATION ${CMAKE_INSTALL_PREFIX}/ext_camlidar

# # # FILES ${CORE_DIR}/build/extrinsic_stereo.wasm DESTINATION ${CMAKE_INSTALL_PREFIX}/stereo
# # )
