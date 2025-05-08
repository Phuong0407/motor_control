project(takephoto)
cmake_minimum_required(VERSION 3.0.0)

set (CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

include_directories(${LIBCAMERA_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include/camera)

add_executable(takephoto takephoto.cpp)
add_executable(takevideo takevideo.cpp)
add_executable(test_camera_controller test_camera_controller.cpp)
add_executable(test_mask test_mask.cpp)

target_link_libraries(takephoto -llccv ${OpenCV_LIBS})
target_link_libraries(takevideo -llccv ${OpenCV_LIBS})
target_link_libraries(test_camera_controller -llccv ${OpenCV_LIBS})
target_link_libraries(tes_mask -llccv ${OpenCV_LIBS})