project(takephoto)
cmake_minimum_required(VERSION 3.0.0)

set (CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

include_directories(
    ${LIBCAMERA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include/
)

add_executable(
    main
    ${CMAKE_SOURCE_DIR}/main.cpp
)

add_executable(
    ball_detection
    ${CMAKE_SOURCE_DIR}/include/kasilas/ball_detection.cpp
)

target_link_libraries(
    main
    -llccv
    ${OpenCV_LIBS}
    ${LIBCAMERA_LIBRARIES}
    -lwiringPi
)

target_link_libraries(
    ball_detection
    -llccv
    ${OpenCV_LIBS}
    ${LIBCAMERA_LIBRARIES}
)