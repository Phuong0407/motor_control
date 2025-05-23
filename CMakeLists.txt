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

add_executable(
    pixel_to_centimeter
    ${CMAKE_SOURCE_DIR}/include/kasilas/pixel_to_centimeter_scale_calibration.cpp
)

add_executable(
    ball_calibration
    ${CMAKE_SOURCE_DIR}/include/kasilas/ball_calibration.cpp
)

add_executable(
    casillas
    ${CMAKE_SOURCE_DIR}/casillas.cpp
)

add_executable(
    casillas_2
    ${CMAKE_SOURCE_DIR}/casillas_2.cpp
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

target_link_libraries(
    pixel_to_centimeter
    -llccv
    ${OpenCV_LIBS}
    ${LIBCAMERA_LIBRARIES}
)

target_link_libraries(
    ball_calibration
    -llccv
    ${OpenCV_LIBS}
    ${LIBCAMERA_LIBRARIES}
)

target_link_libraries(
    casillas
    -llccv
    ${OpenCV_LIBS}
    ${LIBCAMERA_LIBRARIES}
    -lwiringPi
)

target_link_libraries(
    casillas_2
    -llccv
    ${OpenCV_LIBS}
    ${LIBCAMERA_LIBRARIES}
    -lwiringPi
)