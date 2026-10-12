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

# ---------------------------------------------------------------------------
# Optional: ACIS kernel (Spatial / Dassault Systemes)
#
# The C++ code turns the ACIS backend on automatically when <bs3surf.hpp> (or the
# legacy <bs3surf.hxx>) is visible via __has_include, so CMake only has to add the
# ACIS headers to the include path and link the ACIS libraries.
#
# Set ACIS_ROOT to the root of your ACIS installation (it defaults to the
# ACIS_ROOT environment variable). Override ACIS_INCLUDE_DIR / ACIS_LIBRARY_DIR if
# your layout differs; when ACIS_LIBRARY_DIR is empty it is auto-detected under
# ACIS_ROOT (lib, lib64, bin, or the first sub-folder of lib). All libraries found
# in that folder are linked.
# ---------------------------------------------------------------------------
set(ACIS_ROOT "" CACHE PATH "Root of the ACIS installation (headers + libraries); defaults to $ENV{ACIS_ROOT}")
if(NOT ACIS_ROOT AND DEFINED ENV{ACIS_ROOT})
  set(ACIS_ROOT "$ENV{ACIS_ROOT}")
endif()

if(ACIS_ROOT)
  set(ACIS_INCLUDE_DIR "${ACIS_ROOT}/include" CACHE PATH "ACIS headers directory")
  set(ACIS_LIBRARY_DIR "" CACHE PATH "ACIS libraries directory (auto-detected when empty)")

  if(NOT ACIS_LIBRARY_DIR)
    foreach(_acis_cand "${ACIS_ROOT}/lib" "${ACIS_ROOT}/lib64" "${ACIS_ROOT}/bin")
      if(IS_DIRECTORY "${_acis_cand}")
        set(ACIS_LIBRARY_DIR "${_acis_cand}")
        break()
      endif()
    endforeach()
  endif()
  if(NOT ACIS_LIBRARY_DIR)
    file(GLOB _acis_subdirs "${ACIS_ROOT}/lib/*")
    foreach(_acis_d ${_acis_subdirs})
      if(IS_DIRECTORY "${_acis_d}")
        set(ACIS_LIBRARY_DIR "${_acis_d}")
        break()
      endif()
    endforeach()
  endif()

  target_include_directories(nurbs_benchmark PRIVATE "${ACIS_INCLUDE_DIR}")
  if(ACIS_LIBRARY_DIR AND IS_DIRECTORY "${ACIS_LIBRARY_DIR}")
    file(GLOB ACIS_LIBRARIES "${ACIS_LIBRARY_DIR}/*.lib" "${ACIS_LIBRARY_DIR}/*.so" "${ACIS_LIBRARY_DIR}/*.a")
    target_link_libraries(nurbs_benchmark ${ACIS_LIBRARIES})
    message(STATUS "ACIS backend: include '${ACIS_INCLUDE_DIR}', libraries '${ACIS_LIBRARY_DIR}'")
  else()
    message(WARNING "ACIS_ROOT is set but no library directory was found; set ACIS_LIBRARY_DIR")
  endif()
endif()

# ---------------------------------------------------------------------------
# Optional: Parasolid kernel (Siemens)
#
# Enabled in C++ automatically when <parasolid.h> is visible via __has_include.
# Set PARASOLID_ROOT to the root of the Parasolid development kit (it defaults to
# the standard PARASOLID_BASE environment variable). The kernel library
# (pskernel_archive on Windows, pskernel otherwise) is located automatically.
# A valid Parasolid licence/schema must be available at run time.
# ---------------------------------------------------------------------------
set(PARASOLID_ROOT "" CACHE PATH "Root of the Parasolid kit (headers + libraries); defaults to $ENV{PARASOLID_BASE}")
if(NOT PARASOLID_ROOT AND DEFINED ENV{PARASOLID_BASE})
  set(PARASOLID_ROOT "$ENV{PARASOLID_BASE}")
endif()

if(PARASOLID_ROOT)
  set(PARASOLID_INCLUDE_DIR "${PARASOLID_ROOT}" CACHE PATH "Parasolid headers directory")
  set(PARASOLID_LIBRARY "" CACHE FILEPATH "Parasolid kernel library (auto-detected when empty)")

  target_include_directories(nurbs_benchmark PRIVATE "${PARASOLID_INCLUDE_DIR}")

  if(NOT PARASOLID_LIBRARY)
    find_library(PARASOLID_LIBRARY
      NAMES pskernel_archive pskernel
      HINTS "${PARASOLID_ROOT}"
      PATH_SUFFIXES lib lib64 bin ""
    )
  endif()

  if(PARASOLID_LIBRARY)
    target_link_libraries(nurbs_benchmark "${PARASOLID_LIBRARY}")
    message(STATUS "Parasolid backend: include '${PARASOLID_INCLUDE_DIR}', library '${PARASOLID_LIBRARY}'")
  else()
    message(WARNING "PARASOLID_ROOT is set but the Parasolid kernel library was not found; set PARASOLID_LIBRARY")
  endif()
endif()

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
