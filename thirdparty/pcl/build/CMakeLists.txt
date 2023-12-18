project(PCL_INTERFACE)

add_library(PCL_INTERFACE INTERFACE)

set(PCL_INTERFACE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
set(PCL_INTERFACE_INCLUDE_PATH "${PCL_INTERFACE_PATH}/include")
set(PCL_INTERFACE_LIB_PATH "${PCL_INTERFACE_PATH}/lib")

set(lib_PCL_INTERFACE
    ${PCL_INTERFACE_LIB_PATH}/libpcl_common.a
    ${PCL_INTERFACE_LIB_PATH}/libpcl_features.a
    ${PCL_INTERFACE_LIB_PATH}/libpcl_filters.a

    ${PCL_INTERFACE_LIB_PATH}/libpcl_io_ply.a
    ${PCL_INTERFACE_LIB_PATH}/libpcl_io.a
    ${PCL_INTERFACE_LIB_PATH}/libpcl_kdtree.a
    ${PCL_INTERFACE_LIB_PATH}/libpcl_keypoints.a
    ${PCL_INTERFACE_LIB_PATH}/libpcl_ml.a
    ${PCL_INTERFACE_LIB_PATH}/libpcl_octree.a

    ${PCL_INTERFACE_LIB_PATH}/libpcl_recognition.a
    ${PCL_INTERFACE_LIB_PATH}/libpcl_registration.a
    ${PCL_INTERFACE_LIB_PATH}/libpcl_sample_consensus.a
    ${PCL_INTERFACE_LIB_PATH}/libpcl_search.a
    ${PCL_INTERFACE_LIB_PATH}/libpcl_segmentation.a
    ${PCL_INTERFACE_LIB_PATH}/libpcl_stereo.a

    ${PCL_INTERFACE_LIB_PATH}/libpcl_surface.a
    ${PCL_INTERFACE_LIB_PATH}/libpcl_tracking.a)

target_link_libraries(PCL_INTERFACE INTERFACE ${lib_PCL_INTERFACE})
target_include_directories(PCL_INTERFACE INTERFACE ${PCL_INTERFACE_INCLUDE_PATH})
