cmake_minimum_required(VERSION 2.8.3)
project(vehicle_state_estimation)

set(CMAKE_CXX_FLAGS "-std=c++11 -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs nav_msgs tf)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(vehicle_state_estimation
        src/vehicle_state_estimation_node.cpp
)

target_link_libraries(vehicle_state_estimation ${catkin_LIBRARIES} )