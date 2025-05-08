project(takephoto)
cmake_minimum_required(VERSION 3.0.0)

set (CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

include_directories(
    ${LIBCAMERA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include/camera
)

add_executable(
    test_binarymask
    ${CMAKE_SOURCE_DIR}/test/test_binarymask.cpp
)

target_link_libraries(test_binarymask -llccv ${OpenCV_LIBS})
