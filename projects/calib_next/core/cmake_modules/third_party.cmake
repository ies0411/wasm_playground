find_package(Ceres REQUIRED)

add_subdirectory(${OPENCV} ${BUILD_OPENCV_DIR})
add_subdirectory(${PCL} ${BUILD_PCL_DIR})
add_subdirectory(${BOOST} ${BUILD_BOOST_DIR})

# add_subdirectory(test)
link_directories(
  ${PCL_LIB}
  ${FLANN_LIB}
  ${BOOST_LIB}
)

message(STATUS "=====================================")
message(STATUS "# Gathering 3rd party libraries.")

# REQUIRED LIBRARIES ==========================
# Eigen3
include(FetchContent)
set(FETCH_PATH ${CMAKE_BINARY_DIR}/_deps)

if(USE_SYSTEM_EIGEN3)
  set(FETCH_EIGEN3_PATH ${FETCH_PATH}/eigen3-src/Eigen)
  message(STATUS "Finding eigen3")

  IF(NOT EXISTS ${FETCH_EIGEN3_PATH})
    message(STATUS "START EIGEN BUILD")
    FetchContent_Declare(
      Eigen3
      GIT_REPOSITORY "https://gitlab.com/libeigen/eigen.git"
      GIT_TAG "3.4"
    )
    FetchContent_MakeAvailable(Eigen3)
  else()
    message(STATUS "Already Exist")
  endif()

  set(EIGEN3_INCLUDE_PATH ${Eigen3_SOURCE_DIR})
endif()

if(USE_SYSTEM_GTEST)
  set(FETCH_GTEST_PATH ${FETCH_PATH}/googletest-src)
  message(STATUS "Finding gtest")

  IF(NOT EXISTS ${FETCH_GTEST_PATH})
    message(STATUS "START GTEST BUILD")
    FetchContent_Declare(
      googletest
      GIT_REPOSITORY "https://github.com/google/googletest.git"
      GIT_TAG "main"
    )
    FetchContent_MakeAvailable(googletest)
  else()
    message(STATUS "Already Exist")
  endif()

  # add_custom_command(${FETCH_EIGEN3_INCLUDE_PATH} COMMAND)
  # set(_INCLUDE_PATH ${Eigen3_SOURCE_DIR})
endif()

# if(USE_SYSTEM_GLOG)
# set(FETCH_GLOG_PATH ${FETCH_PATH}/glog-src)
# message(STATUS "Finding glog")

# IF(NOT EXISTS ${FETCH_GLOG_PATH})
# message(STATUS "START GLOG BUILD")
# FetchContent_Declare(
# glog
# GIT_REPOSITORY "https://github.com/google/glog.git"
# GIT_TAG "master"
# )
# FetchContent_MakeAvailable(glog)
# else()
# message(STATUS "Already Exist")
# endif()

# add_custom_command(${FETCH_EIGEN3_INCLUDE_PATH} COMMAND)
# set(_INCLUDE_PATH ${Eigen3_SOURCE_DIR})
# endif()
# if(USE_SYSTEM_CERES)
# set(FETCH_CERES_PATH ${FETCH_PATH}/ceres-src)
# message(STATUS "Finding ceres")

# IF(NOT EXISTS ${FETCH_CERES_PATH})
# message(STATUS "START CERES BUILD")
# FetchContent_Declare(
# ceres
# URL http://ceres-solver.org/ceres-solver-2.1.0.tar.gz
# BUILD_COMMAND "emake make"
# INSTALL_COMMAND "make install"
# )
# FetchContent_GetProperties(ceres)

# if(NOT ceres_POPULATED)
# FetchContent_Populate(ceres)
# add_subdirectory(${ceres_SOURCE_DIR} ${ceres_BINARY_DIR})
# endif()

# # FetchContent_MakeAvailable(ceres)
# else()
# message(STATUS "Already Exist")
# endif()

# # add_custom_command(${FETCH_EIGEN3_INCLUDE_PATH} COMMAND)
# # set(_INCLUDE_PATH ${Eigen3_SOURCE_DIR})
# endif()

# if(NOT USE_SYSTEM_EIGEN3)
# include(${3RDPARTY_DIR}/eigen/Eigen.cmake)
# superb3d_import_3rdparty_library(Eigen3
# PUBLIC
# INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS}
# INCLUDE_ALL
# ) # TODO: find a way to compile with EIGEN_MPL2_ONLY
# endif()

# append_item_to_parent_scope_list(SUPERB3D_3RDPARTY_PUBLIC_LIBRARIES ${PROJECT_NAME}::${3RDPARTY_PREFIX}_Eigen3)

# # DEPENDANT LIBRARIES =========================
# # pcl - Eigen / FLANN
# if(USE_SYSTEM_PCL)
# superb3d_find_3rdparty_package(PCL
# PUBLIC
# QUIET
# TARGETS
# pcl_filters # also includes

# # pcl_common
# # sample_consensus
# # search
# # kdtree
# pcl_io # also includes

# # boost::system
# # boost::filesystem
# # boost::date_time
# # boost::iostreams
# # boost::serialization
# # pcl_common
# # pcl_octree
# pcl_geometry # pcl_common
# )

# if(NOT PCL_FOUND)
# set(USE_SYSTEM_PCL OFF)
# endif()
# endif()

# if(NOT USE_SYSTEM_PCL)
# if(BUILD_WASM)
# include(${3RDPARTY_DIR}/pcl/PclWasmBuilt.cmake)
# superb3d_import_3rdparty_library(PCL
# PUBLIC
# INCLUDE_DIRS ${PCL_INCLUDE_DIRS}
# LIB_DIR ${PCL_LIB_DIR}
# LIBRARIES ${PCL_LIBRARIES}
# )
# add_dependencies(_PCL pcl_wasm_build)
# else()
# message(WARNING "importing pcl not supported yet! use system pcl instead")
# superb3d_find_3rdparty_package(PCL
# PUBLIC
# QUIET
# TARGETS
# pcl_filters # also includes

# # pcl_common
# # sample_consensus
# # search
# # kdtree
# pcl_io # also includes

# # boost::system
# # boost::filesystem
# # boost::date_time
# # boost::iostreams
# # boost::serialization
# # pcl_common
# # pcl_octree
# pcl_geometry # pcl_common
# )
# endif()

# # superb3d_import_3rdparty_library(PCL
# # INCLUDE_DIRS ${PCL_INCLUDE_DIRS}
# # LIB_DIR ${PCL_LIB_DIR}
# # LIBRARIES ${PCL_LIBRARIES}
# # )
# endif()

# append_item_to_parent_scope_list(SUPERB3D_3RDPARTY_PUBLIC_LIBRARIES ${PROJECT_NAME}::${3RDPARTY_PREFIX}_PCL)

# # CONDITIONAL LIBRARIES =======================
# # spdlog
# if(USE_SPDLOG)
# if(USE_SYSTEM_SPDLOG)
# superb3d_find_3rdparty_package(spdlog
# TARGETS spdlog::spdlog
# )

# if(NOT spdlog_FOUND)
# set(USE_SYSTEM_SPDLOG OFF)
# endif()
# endif()

# if(NOT USE_SYSTEM_SPDLOG)
# include(${3RDPARTY_DIR}/spdlog/Spdlog.cmake)
# superb3d_import_3rdparty_library(spdlog
# INCLUDE_DIRS ${SPDLOG_INCLUDE_DIRS}
# LIB_DIR ${SPDLOG_LIB_DIR}
# LIBRARIES ${SPDLOG_LIBRARIES}
# )
# endif()

# append_item_to_parent_scope_list(SUPERB3D_3RDPARTY_PRIVATE_LIBRARIES ${PROJECT_NAME}::${3RDPARTY_PREFIX}_spdlog)
# endif()

# # gtest
# if(BUILD_TESTS)
# if(USE_SYSTEM_GTEST)
# superb3d_find_3rdparty_package(GTest
# TARGETS GTest::gmock GTest::gtest_main
# )

# if(NOT GTest_FOUND)
# set(USE_SYSTEM_GTEST OFF)
# endif()
# endif()

# if(NOT USE_SYSTEM_GTEST)
# include(${3RDPARTY_DIR}/googletest/GoogleTest.cmake)
# superb3d_import_3rdparty_library(GTest
# PUBLIC
# INCLUDE_DIRS ${GTEST_INCLUDE_DIRS}
# LIB_DIR ${GTEST_LIB_DIR}
# LIBRARIES ${GTEST_LIBRARIES}
# )
# endif()
# endif()

# list(REMOVE_DUPLICATES SUPERB3D_3RDPARTY_EXTERNAL_MODULES)

# print_list(SUPERB3D_3RDPARTY_EXTERNAL_MODULES)
# print_list(SUPERB3D_3RDPARTY_PUBLIC_LIBRARIES)
# print_list(SUPERB3D_3RDPARTY_PRIVATE_LIBRARIES)

# set(SUPERB3D_3RDPARTY_EXTERNAL_MODULES ${SUPERB3D_3RDPARTY_EXTERNAL_MODULES} PARENT_SCOPE)
# set(SUPERB3D_3RDPARTY_PUBLIC_LIBRARIES ${SUPERB3D_3RDPARTY_PUBLIC_LIBRARIES} PARENT_SCOPE)
# set(SUPERB3D_3RDPARTY_PRIVATE_LIBRARIES ${SUPERB3D_3RDPARTY_PRIVATE_LIBRARIES} PARENT_SCOPE)