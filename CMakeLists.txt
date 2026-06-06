cmake_minimum_required(VERSION 3.10)
project(NurbsBenchmark)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OCCT
find_package(OpenCASCADE REQUIRED)

add_executable(nurbs_benchmark nurbs_benchmark.cpp)

target_link_libraries(nurbs_benchmark
    ${OpenCASCADE_LIBRARIES}
)

target_include_directories(nurbs_benchmark PRIVATE
    ${OpenCASCADE_INCLUDE_DIR}
)
