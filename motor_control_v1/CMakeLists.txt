cmake_minimum_required(VERSION 3.0.0)

# Define your project
project(RedLineBot CXX)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include LCCV and libcamera headers
include_directories(/usr/local/include)
include_directories(/usr/include/libcamera)  # Add this if span.h is located here

# Find OpenCV
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

# Find WiringPi and LCCV
find_library(WIRINGPI_LIB wiringPi)
find_library(LCCV_LIB lccv)

# Source file
add_executable(red_line_bot red_line_bot.cpp)

# Link libraries
target_link_libraries(red_line_bot
    -llccv
    ${OpenCV_LIBS}
    ${WIRINGPI_LIB}
    ${LIBCAMERA_LIBRARIES}
    -lwiringPi
)

add_executable(motor_test motor_test.cpp)
target_link_libraries(motor_test ${WIRINGPI_LIB})
