project(takephoto)
cmake_minimum_required(VERSION 3.0.0)

set (CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(OpenMP REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0 -Wall -Wextra -Wpedantic")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -O0 -Wall -Wextra -Wpedantic")

include_directories(
    ${LIBCAMERA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include/
)

# add_executable(
#     test_vision
#     ${CMAKE_SOURCE_DIR}/test/test_vision.cpp
# )

# add_executable(
#     test_motor
#     ${CMAKE_SOURCE_DIR}/test/test_motor.cpp
# )

# add_executable(
#     main
#     ${CMAKE_SOURCE_DIR}/main.cpp
# )

# add_executable(
#     test_encoder
#     ${CMAKE_SOURCE_DIR}/test/test_encoder.cpp
# )

# add_executable(
#     test_motorcontroller
#     ${CMAKE_SOURCE_DIR}/test/test_motorcontroller.cpp
# )

# add_executable(
#     test_multimotorcontroller
#     ${CMAKE_SOURCE_DIR}/test/test_multimotorcontroller.cpp
# )

add_executable(
    test_imageprocessor
    ${CMAKE_SOURCE_DIR}/test/test_imageprocessor.cpp
)

# target_link_libraries(test_vision -llccv ${OpenCV_LIBS} ${LIBCAMERA_LIBRARIES})
# target_link_libraries(main -llccv ${OpenCV_LIBS} ${LIBCAMERA_LIBRARIES} -lwiringPi)
# target_link_libraries(test_motorcontroller -lwiringPi)
# target_link_libraries(test_encoder -lwiringPi)
# target_link_libraries(test_multimotorcontroller -lwiringPi)
# target_link_libraries(test_motor -lwiringPi)

target_link_libraries(
    test_imageprocessor
    -llccv
    ${OpenCV_LIBS}
    ${LIBCAMERA_LIBRARIES}
    OpenMP::OpenMP_CXX
)