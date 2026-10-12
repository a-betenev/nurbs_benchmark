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

For building with the ACIS kernel (Spatial / Dassault Systemes), set the CMake
variable `ACIS_ROOT` (or the environment variable of the same name) to the root of
your ACIS installation so that its `include` folder and libraries are found.
Override `ACIS_INCLUDE_DIR` / `ACIS_LIBRARY_DIR` if your layout differs. The ACIS
backend is used automatically once its headers (`bs3surf.hpp` / `bs3surf.hxx`) are
visible to the compiler; no extra macro is needed.

For building with the Parasolid kernel (Siemens), set the CMake variable
`PARASOLID_ROOT` (it defaults to the standard `PARASOLID_BASE` environment
variable) to the root of the Parasolid development kit. The backend is used
automatically once `parasolid.h` is visible to the compiler; the kernel library
(`pskernel_archive` / `pskernel`) is located and linked automatically. A valid
Parasolid licence/schema must be available at run time.

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
    lib           - Library to be tested: either of occt (default), tinynurbs, sisl, acis, parasolid
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

Evaluation variants (ACIS / Parasolid / OCCT)
----------------------------------------------

Both kernels evaluate a B-spline entity directly (ACIS `bs3_curve_eval` /
`bs3_surface_evaluate`; Parasolid `PK_CURVE_eval` / `PK_SURF_eval`). Neither
exposes a separate user-facing "adaptor + explicit cache" object like OCCT's
`BRepAdaptor_Surface` backed by `BSplSLib_Cache`; instead the B-spline classes
cache the active span internally, so these direct evaluators are the closest
analogue of OCCT's cached evaluation and are what this benchmark uses.

Other variants that exist, but are intentionally not used here:

* ACIS: the virtual `curve::eval()` / `surface::eval()` member functions;
  `bs3_curve_evaluate()` / `bs3_surface_evaluate()` requesting an arbitrary number
  of derivatives in one call; and evaluation through topology (`api_*`, FACE /
  EDGE), which additionally applies transforms and sense.
* Parasolid: evaluation through a FACE (`PK_EVAL_face_*` / `PK_FACE_*`), which
  applies the face transform and sense on top of the surface; and the batched
  `PK_SURF_eval_grid`, which evaluates a whole rectangular grid of parameters in a
  single call (useful for tessellation, less so for single-point timing).
* OCCT: the alternative to the cached `BRepAdaptor_*` evaluation used here is
  direct evaluation through the geometric class (`Geom_BSplineCurve::D0/D1/D2`,
  `Geom_BSplineSurface::D0/D1/D2`), which recomputes the span on every call.

Performance measurements
------------------------

To have performance measurements stable, it is recommended to:

* Disable CPU Turbo Boost (e.g. use ThrottleStop utility by TechPowerUp
  https://www.techpowerup.com/download/techpowerup-throttlestop/, run it and check "Disable turbo")
* Run with affinity to one (performant) CPU core (done in script `nurbs_benchmark_grid.bat`)
* Set high priority to the process (done in script `nurbs_benchmark_grid.bat`)
* Set high performance profile (in OS settings) and disable computer going to sleep on user inactivity

