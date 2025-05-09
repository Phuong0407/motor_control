project(takephoto)
cmake_minimum_required(VERSION 3.0.0)

set (CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

include_directories(
    ${LIBCAMERA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include/vision
)

add_executable(
    testcontour
    ${CMAKE_SOURCE_DIR}/test/testcontour.cpp
)

target_link_libraries(testcontour -llccv ${OpenCV_LIBS})
