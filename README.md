NURBS Benchmark
=======================

This is a small program aimed to measure performance of evaluation of points
on NURBS (BSpline) curves and surfaces in different versions of Open
CASCADE Technology (OCCT) or (after relevant adaptation) other similar libraries. 

Build
-----

Use CMake. Define variable `CMAKE_PREFIX_PATH` pointing to directory where
Open CASCADE is installed for CMake to be able to find it.

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

Example:

    nurbs_benchmark.exe path_to_occt/data/occ/terrain.brep surfaces 1 1000000 500 sequential 1 perf 2>>runs.log

Note that each run outputs one-line digest of the run containing program arguments and times to `cerr`, this can be used to collect a log of all runs for consequent analysis.

Hints
-----

To have performance measurements stable, it is recommended to:

* Run with affinity to one (performant) CPU core (done programmatically)
* Set high priority to the process (done programmatically)
* Set high performance profile (in OS settings)
* Disable CPU Turbo Boost (e.g. use ThrottleStop utility by TechPowerUp, run it and check "Disable turbo")

