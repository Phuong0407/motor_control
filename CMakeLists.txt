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
    ${CMAKE_SOURCE_DIR}/include/motor
    ${CMAKE_SOURCE_DIR}/include/navigation
)

# add_executable(
#     test_vision
#     ${CMAKE_SOURCE_DIR}/test/test_vision.cpp
# )

# add_executable(
    # test_motor
    # ${CMAKE_SOURCE_DIR}/test/test_motor.cpp
# )

add_executable(
    main
    ${CMAKE_SOURCE_DIR}/main.cpp
)

# target_link_libraries(test_vision -llccv ${OpenCV_LIBS})
# target_link_libraries(test_motor -lwiringPi)
target_link_libraries(main -llccv ${OpenCV_LIBS} -lwiringPi)
