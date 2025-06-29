cmake_minimum_required(VERSION 2.8.3)
project(ipm_generation)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 定义
add_definitions(-DIMAGE_DIR="${PROJECT_SOURCE_DIR}/images/")

find_package(OpenCV REQUIRED)

include_directories(${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS})

add_executable(ipm_generation
        src/ipm_main.cc
        src/ipm.cc
)
target_link_libraries(ipm_generation ${OpenCV_LIBS})