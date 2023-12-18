project(BOOST)

add_library(BOOST INTERFACE)

set(BOOST_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
set(BOOST_INCLUDE_PATH "${BOOST_PATH}/include/boost")
set(BOOST_LIB_PATH "${OPENCV_PATH_PATH}/lib")

set(lib_BOOST
    ${BOOST_LIB_PATH}/libboost_date_time.a
    ${BOOST_LIB_PATH}/libboost_filesystem.a
    ${BOOST_LIB_PATH}/libboost_iostreams.a
    ${BOOST_LIB_PATH}/libboost_regex.a
    ${BOOST_LIB_PATH}/libboost_system.a)

target_link_libraries(BOOST INTERFACE ${lib_BOOST})
target_include_directories(BOOST INTERFACE ${BOOST_INCLUDE_PATH})
