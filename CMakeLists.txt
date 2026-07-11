cmake_minimum_required(VERSION 3.10)
project(NurbsBenchmark)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OCCT
find_package(OpenCASCADE REQUIRED)

# Find SISL
find_package(SISL OPTIONAL)

add_executable(nurbs_benchmark src/nurbs_benchmark.cpp)

# enable AVX2 instructions for speed
target_compile_options(nurbs_benchmark PRIVATE $<$<BOOL:${MSVC}>:/arch:AVX2>)

target_link_libraries(nurbs_benchmark
    ${OpenCASCADE_LIBRARIES}
    ${sisl_LIBRARIES}
)

target_include_directories(nurbs_benchmark PRIVATE
    ${OpenCASCADE_INCLUDE_DIR}
)

target_include_directories(nurbs_benchmark PRIVATE
    ${CMAKE_SOURCE_DIR}/submodules/tinynurbs/include
    ${CMAKE_SOURCE_DIR}/submodules/glm
)

target_include_directories(nurbs_benchmark PRIVATE
    ${sisl_INCLUDE_DIRS}
)

set_target_properties(nurbs_benchmark PROPERTIES VS_DEBUGGER_ENVIRONMENT "PATH=%PATH%;${OpenCASCADE_DIR}/../win64/vc14/bin")

# On Windows, enerate the bat file to run tests
file(GENERATE OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/nurbs_benchmark_grid.bat CONTENT
":: Set path to OCCT and launch test with specified options;
:: see file testgrid.bat in source folder for arguments
call \"${OpenCASCADE_DIR}/../env.bat\"
set \"BUILD_PATH=%~dp0\"
${CMAKE_SOURCE_DIR}/testgrid.bat %*
"
)
