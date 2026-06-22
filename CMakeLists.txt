cmake_minimum_required(VERSION 3.10)
project(NurbsBenchmark)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OCCT
find_package(OpenCASCADE REQUIRED)

add_executable(nurbs_benchmark src/nurbs_benchmark.cpp)

target_link_libraries(nurbs_benchmark
    ${OpenCASCADE_LIBRARIES}
)

target_include_directories(nurbs_benchmark PRIVATE
    ${OpenCASCADE_INCLUDE_DIR}
)

# On Windows, enerate the bat file to run tests
file(GENERATE OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/nurbs_benchmark_grid.bat CONTENT
":: Set path to OCCT and launch test with specified options;
:: see file testgrid.bat in source folder for arguments
call \"${OpenCASCADE_DIR}/../env.bat\"
set \"BUILD_PATH=%~dp0\"
${CMAKE_SOURCE_DIR}/testgrid.bat %*
"
)
