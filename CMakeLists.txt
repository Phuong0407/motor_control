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
    line_follower
    ${CMAKE_SOURCE_DIR}/line_follower.cpp
)

add_executable(
    casillas
    ${CMAKE_SOURCE_DIR}/casillas.cpp
)

add_executable(
    casillas_2
    ${CMAKE_SOURCE_DIR}/casillas_2.cpp
)


add_executable(
    goal_keeper
    ${CMAKE_SOURCE_DIR}/goal_keeper.cpp
)

# target_link_libraries(
#     line_follower
#     -llccv
#     ${OpenCV_LIBS}
#     ${LIBCAMERA_LIBRARIES}
#     -lwiringPi
# )

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


# target_link_libraries(
#     goal_keeper
#     -llccv
#     ${OpenCV_LIBS}
#     ${LIBCAMERA_LIBRARIES}
#     -lwiringPi
# )