
message("run set path...")
set(CORE_DIR ${PROJECT_SOURCE_DIR}/../core/build)

set(SRC_DIR ${PROJECT_SOURCE_DIR}/src)

set(OPENCV ${PROJECT_SOURCE_DIR}/../../../thirdparty/opencv)
set(BUILD_OPENCV_DIR ./build_opencv)

set(PCL ${PROJECT_SOURCE_DIR}/../../../thirdparty/pcl/build)
set(PCL_LIB ${PROJECT_SOURCE_DIR}/../../../thirdparty/pcl/build/lib)
set(PCL_INCLUDE ${PCL}/include/pcl-1.12)
set(BUILD_PCL_DIR ./build_pcl)

set(FLANN ${PROJECT_SOURCE_DIR}/../../../thirdparty/flann/build)
set(FLANN_INCLUDE ${FLANN}/include)
set(FLANN_LIB ${FLANN}/lib)

set(EIGEN_BUILD_INCLUDE ${FLANN}/include)

set(BOOST ${PROJECT_SOURCE_DIR}/../../../thirdparty/boost/build)
set(BOOST_INCLUDE ${BOOST}/include)
set(BOOST_LIB ${BOOST}/lib)
set(BUILD_BOOST_DIR ./build_boost)

set(TEST_DIR ${PROJECT_SOURCE_DIR}/test)

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/../js/pages/)
