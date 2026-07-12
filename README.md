NURBS Benchmark
=======================

This is a small program aimed to measure performance of evaluation of points
on NURBS (BSpline) curves and surfaces in different versions of Open
CASCADE Technology (OCCT) and other libraries providing this functionality.

Build
-----

Use CMake.

Define variable `CMAKE_PREFIX_PATH` pointing to directory where Open CASCADE
is installed for CMake to be able to find it.

For building with TinyNurbs library (https://github.com/pradeep-pyro/tinynurbs),
it should be sufficient to update Git with all submodules (TinyNurbs and glm are
included as submodules in this repo).

For building with SISL (https://github.com/SINTEF-Geometry/SISL), add path to
SISL installation folder to `CMAKE_PREFIX_PATH` variable.
You might also need to modify CMake files in SISL installation folder to help
CMake to find it; in my case I needed to move sub-folder `cmake` from `lib\sisl`
to root folder, and edit file `sislConfig.cmake` to have variable `PACKAGE_PREFIX_DIR`
pointing to the SISL installation folder.

By the moment, the only really used build environment is VS 2022.

Use
---
Synopsis:

    Usage: nurbs_benchmark.exe <file> <curves|surfaces> <indices> <numPoints> <numRepeats> <random|sequential> <derivDegree(0-2)> <perf|record>
    Parameters:
    <file>        - Path to .brep or .step file
    <curves|surfaces> - Choose what to evaluate - either curves or surfaces
    <indices>     - Comma-separated list of edge (for curves) or face (for surfaces) indices (1-based),
                    can include ranges like 3-7
    <numPoints>   - Number of points to evaluate
    <numRepeats>  - Number of times to repeat evaluation (for greater, more stable, time measurements)
    <random|sequential> - Parameter distribution mode
    <derivDegree> - Derivative degree (0, 1, or 2)
    <perf|record> - Output mode: "perf" for performance measurement, "record" for additional output of all 
                    calculated values (use only with reasonably small number of points) 
    lib           - Library to be tested: either of occt (default), tinynurbs, sisl, lnlib
                    (each available only if executable is built with corresponding library)

Example:

    nurbs_benchmark.exe path_to_occt/data/occ/terrain.brep surfaces 1 1000000 500 sequential 1 perf 2>>runs.log

Remarks
-------

For each surface evaluated the program outputs its type and, in case of BSpline, basic properties.

Each run outputs one-line digest of the run containing program arguments and times to `cerr`,
this can be used to collect a log of all runs for consequent analysis.

Use the helper script with name `nurbs_benchmark_grid.bat` generated in the build folder to either
run the test on the specified model (with arguments as above) or run complete testgrid on the
included sample models with pedefined parameters.
Note that this script redirects `cerr` to local file `run.log`.

The points for evaluation are calculated in the parametric range of the face, not natural range of the surface.
Some faces can go (usually slightly) outside of the natural range of the surface.

If executed with `numPoints` = 4 in sequential mode, for the NURBS surfaces which are clamped to their
poles at some corners, the distance from the points evaluated at such corners and the pole is output.
This is an indirect test for the algorithm used for evaluation: we expect that `de Boor` algorithm shall
provide exact value while polinomial evaluation might give some error.
In any case the deviation is a measure of accuracy of evaluation against ground truth.

OCCT is used to load data and prepare UV points for evaluation regardless of the selected library.
The selected library is used only for evaluation, thus the NURBS parameters and UV points are
guaranteed to be the same.

Performance measurements
------------------------

To have performance measurements stable, it is recommended to:

* Disable CPU Turbo Boost (e.g. use ThrottleStop utility by TechPowerUp
  https://www.techpowerup.com/download/techpowerup-throttlestop/, run it and check "Disable turbo")
* Run with affinity to one (performant) CPU core (done in script `nurbs_benchmark_grid.bat`)
* Set high priority to the process (done in script `nurbs_benchmark_grid.bat`)
* Set high performance profile (in OS settings) and disable computer going to sleep on user inactivity

