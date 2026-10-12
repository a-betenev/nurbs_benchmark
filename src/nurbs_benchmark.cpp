/**
 * NURBS Benchmark Tool for Open CASCADE Technology (OCCT)
 *
 * This application benchmarks the calculation of points on NURBS curves and surfaces.
 * It measures performance and optionally records evaluation results for verification.
 *
 * Usage: nurbs_benchmark <file> <curves|surfaces> <indices> <numPoints> <random|sequential> <derivDegree(0-2)> <perf|record>
 * Example: nurbs_benchmark model.step curves 1,2,3 10000 random 1 perf
 */

#include <array>
#include <iostream>
#include <string>
#include <vector>
#include <random>
#include <chrono>
#include <iomanip>
#include <filesystem>
#include <cmath>
#include <ctime>
#include <variant>

#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <STEPControl_Reader.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TColStd_Array2OfReal.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

// If TinyNurbs library is available (in submodules), use it
#if __has_include(<tinynurbs/tinynurbs.h>)
#define HAS_TINYNURBS
#endif
#ifdef HAS_TINYNURBS
#include <tinynurbs/tinynurbs.h>
#endif

// If SISL library is available, use it
#if __has_include(<sisl.h>)
#define HAS_SISL
#endif
#ifdef HAS_SISL
#include <sisl.h>
#endif

// If the ACIS kernel (Spatial/Dassault) is available, use it.
// ACIS ships its headers either with the modern .hpp or the legacy .hxx
// extension; detect whichever is present and remember it so the includes below
// resolve consistently. bs3surf/bs3curve transitively pull in SPAposition,
// SPAvector and SPApar_pos; kernapi provides api_initialize_acis().
#if defined(__has_include)
#  if __has_include(<bs3surf.hpp>)
#    define HAS_ACIS 1
#    define ACIS_HEADER(f) <f.hpp>
#  elif __has_include(<bs3surf.hxx>)
#    define HAS_ACIS 1
#    define ACIS_HEADER(f) <f.hxx>
#  endif
#endif
#ifdef HAS_ACIS
#  include ACIS_HEADER(kernapi)
#  include ACIS_HEADER(bs3curve)
#  include ACIS_HEADER(bs3surf)
#endif

// If the Parasolid kernel (Siemens) is available, use it. The whole C API is
// exposed through the single umbrella header <parasolid.h>.
#if defined(__has_include)
#  if __has_include(<parasolid.h>)
#    define HAS_PARASOLID 1
#  endif
#endif
#ifdef HAS_PARASOLID
#  include <parasolid.h>
#endif

#ifdef _WIN32
#include <windows.h>
#include <powerbase.h>
#pragma comment(lib, "PowrProf.lib")

enum class Library {
  OCCT,
  ACIS,
  Parasolid,
  Poisson,
  TinyNurbs,
  SISL,
  LNLib
};

 //! Query maximal processor frequency, 0. if failed
static double GetCPUFrequency()
{
  // Note: this structure is missing in WinNT.h, so copied here according to MS recommendation
  struct PROCESSOR_POWER_INFORMATION
  {
    ULONG Number;
    ULONG MaxMhz;
    ULONG CurrentMhz;
    ULONG MhzLimit;
    ULONG MaxIdleState;
    ULONG CurrentIdleState;
  };

  SYSTEM_INFO aSysInfo;
  GetSystemInfo(&aSysInfo);

  std::vector<PROCESSOR_POWER_INFORMATION> aProcInfoVec(aSysInfo.dwNumberOfProcessors, PROCESSOR_POWER_INFORMATION{});
  LONG status = CallNtPowerInformation(ProcessorInformation, NULL, 0, &aProcInfoVec[0],
                                       aSysInfo.dwNumberOfProcessors * sizeof(PROCESSOR_POWER_INFORMATION));
  if (status != 0)
  {
    std::cerr << "Warning: CallNtPowerInformation() failed with status = " << status << std::endl;
    return 0.;
  }
  double aFrequency = 0.0;
  for (const PROCESSOR_POWER_INFORMATION& aProcInfo : aProcInfoVec)
  {
    aFrequency = std::max(aFrequency, 1.e6 * aProcInfo.MaxMhz);
  }
  return aFrequency;
}

double GetProcessCPU()
{
  static const HANDLE hProcess = GetCurrentProcess();
  static const double aFrequency = GetCPUFrequency();

  ULONG64 cycleCount = 0;
  if (aFrequency > 0. && QueryProcessCycleTime(hProcess, &cycleCount))
  {
    return cycleCount / aFrequency;
  }
  return 0.; // stub
}

#endif

 /**
  * Print usage information
  */
void printUsage(const char* progName)
{
  std::cout << "Usage: " << progName << " <file> <curves|surfaces> <indices> <numPoints> <numRepeats> <random|sequential> <derivDegree(0-2)> <perf|record|probe> [lib]\n";
  std::cout << "  <file>        - Path to .brep or .step file\n";
  std::cout << "  <curves|surfaces> - Choose to evaluate curves or surfaces\n";
  std::cout << "  <indices>     - Comma-separated list of edge (for curves) or face (for surfaces) indices (1-based),\n";
  std::cout << "                  can include ranges like 3-7, or '*' for all subshapes (probe mode only)\n";
  std::cout << "  <numPoints>   - Number of points to evaluate\n";
  std::cout << "  <numRepeats>  - Number of times to repeat evaluation (for greater times)\n";
  std::cout << "  <random|sequential> - Parameter distribution mode\n";
  std::cout << "  <derivDegree> - Derivative degree (0, 1, or 2)\n";
  std::cout << "  <perf|record|probe> - Output mode: performance measurement, result recording,\n";
  std::cout << "                  or geometry probe (report type/BSpline properties of every subshape, no evaluation)\n";
  std::cout << "  lib           - Library to be used: occt (default), tinynurbs, sisl, acis, parasolid\n";
  std::cout << "\nExample: " << progName << " model.step curves 1,2,3 10000 10 random 1 perf\n";
}

/**
 * Create a deterministic random number generator with fixed seed
 * This ensures reproducible results across runs
 */
static std::mt19937 createRNG()
{
  return std::mt19937(12345);  // Fixed seed for reproducibility
}

/**
 * Generate evenly spaced sequential parameters
 */
static std::vector<double> generateSequentialParams(int n, double tmin, double tmax)
{
  std::vector<double> params(n);
  if (n <= 1) {
    params[0] = (tmin + tmax) / 2.0;
  }
  else {
    double step = (tmax - tmin) / (n - 1);
    for (int i = 0; i < n; ++i) {
      params[i] = tmin + i * step;
    }
  }
  return params;
}

/**
 * Generate uniformly distributed random parameters
 */
static std::vector<double> generateRandomParams(int n, double tmin, double tmax)
{
  std::vector<double> params(n);
  auto rng = createRNG();
  std::uniform_real_distribution<double> dist(tmin, tmax);
  for (int i = 0; i < n; ++i) {
    params[i] = dist(rng);
  }
  return params;
}

/**
 * Generate evenly spaced sequential UV parameters for surfaces
 * Creates a grid of points and flattens to a vector
 */
static std::vector<std::pair<double, double>> generateSequentialUVParams(int n, double umin, double umax, double vmin, double vmax)
{
  std::vector<std::pair<double, double>> uvPoints;
  uvPoints.reserve(n);

  // Calculate number of points in the grid in each direction, but not less than 2
  int n_side = std::max(2, static_cast<int>(std::sqrt(n)));

  // Create a grid that roughly covers the desired number of points
  double ustep = (umax - umin) / (n_side - 1);
  double vstep = (vmax - vmin) / (n_side - 1);

  for (int i = 0; i < n_side && (int)uvPoints.size() < n; ++i) {
    for (int j = 0; j < n_side && (int)uvPoints.size() < n; ++j) {
      double u = umin + i * ustep;
      double v = vmin + j * vstep;
      uvPoints.emplace_back(u, v);
    }
  }

  // Fill remaining points if grid was too small
  while ((int)uvPoints.size() < n) {
    uvPoints.push_back(uvPoints.back());
  }

  return uvPoints;
}

/**
 * Generate uniformly distributed random UV parameters
 */
static std::vector<std::pair<double, double>> generateRandomUVParams(int n, double umin, double umax, double vmin, double vmax)
{
  std::vector<std::pair<double, double>> uvPoints(n);
  auto rng = createRNG();
  std::uniform_real_distribution<double> distU(umin, umax);
  std::uniform_real_distribution<double> distV(vmin, vmax);
  for (int i = 0; i < n; ++i) {
    uvPoints[i] = { distU(rng), distV(rng) };
  }
  return uvPoints;
}

/**
 * Performance measurement results
 */
struct PerfResult
{
  size_t nbPoints = 0;
  double elapsedTime = 0;
  double cpuTime = 0;
  double cpuTime2 = 0;
  unsigned __int64 totalTsc = 0;
};

/**
 * Measure both wall clock time and CPU time of a function execution
 * Returns pair<wall_time_seconds, cpu_time_seconds>
 */
template <typename Func>
PerfResult measureTime(Func&& func, int numRepeats)
{
  auto start_cpu = std::clock();
  auto start_wall = std::chrono::high_resolution_clock::now();
  double start_cpu2 = GetProcessCPU();
  unsigned __int64 start_tsc = __rdtsc();

  for (int i = 0; i < numRepeats; i++) {
    func();
  }

  unsigned __int64 total_tsc = __rdtsc() - start_tsc;
  double total_cpu2 = GetProcessCPU() - start_cpu2;

  auto end_wall = std::chrono::high_resolution_clock::now();
  auto end_cpu = std::clock();

  double wall_time = std::chrono::duration<double>(end_wall - start_wall).count();
  double cpu_time = static_cast<double>(end_cpu - start_cpu) / CLOCKS_PER_SEC;

  return { 0, wall_time, cpu_time, total_cpu2, total_tsc };
}

/**
 * Load a TopoDS_Shape from either BREP or STEP file
 */
TopoDS_Shape loadShape(const std::string& filePath)
{
  TopoDS_Shape shape;
  std::string ext = std::filesystem::path(filePath).extension().string();

  // Convert extension to lowercase for comparison
  for (char& c : ext) c = std::tolower(c);

  if (ext == ".brep") {
    BRep_Builder B;
    if (!BRepTools::Read(shape, filePath.c_str(), B)) {
      throw std::runtime_error("Failed to read BREP file: " + filePath);
    }
  }
  else if (ext == ".step" || ext == ".stp") {
    STEPControl_Reader reader;
    IFSelect_ReturnStatus stat = reader.ReadFile(filePath.c_str());
    if (stat != IFSelect_RetDone) {
      throw std::runtime_error("Failed to read STEP file: " + filePath);
    }
    reader.TransferRoots();
    shape = reader.OneShape();
  }
  else {
    throw std::runtime_error("Unsupported file format. Use .brep, .step, or .stp");
  }

  if (shape.IsNull()) {
    throw std::runtime_error("Loaded shape is null");
  }

  return shape;
}

/**
 * Parse comma-separated list of indices
 */
std::vector<int> parseIndices(const std::string& input)
{
  std::vector<int> result;

  if (input.empty()) {
    return result;
  }

  std::stringstream ss(input);
  std::string token;

  while (std::getline(ss, token, ',')) {
    // Trim whitespace
    size_t start = token.find_first_not_of(" \t");
    size_t end = token.find_last_not_of(" \t");

    if (start == std::string::npos) {
      continue; // Skip empty tokens
    }

    token = token.substr(start, end - start + 1);

    // Check if it's a range (contains '-')
    size_t dashPos = token.find('-');

    if (dashPos != std::string::npos) {
      // Parse range: "start-end"
      std::string startStr = token.substr(0, dashPos);
      std::string endStr = token.substr(dashPos + 1);

      // Trim whitespace from start and end strings
      startStr.erase(0, startStr.find_first_not_of(" \t"));
      startStr.erase(startStr.find_last_not_of(" \t") + 1);
      endStr.erase(0, endStr.find_first_not_of(" \t"));
      endStr.erase(endStr.find_last_not_of(" \t") + 1);

      if (startStr.empty() || endStr.empty()) {
        throw std::invalid_argument("Invalid range format: " + token);
      }

      // Parse integers
      int startVal = std::stoi(startStr);
      int endVal = std::stoi(endStr);

      if (startVal > endVal) {
        throw std::invalid_argument("Invalid range: start > end in " + token);
      }

      // Add all integers in the range
      for (int i = startVal; i <= endVal; ++i) {
        result.push_back(i);
      }
    }
    else {
      // Parse single integer
      result.push_back(std::stoi(token));
    }
  }

  return result;
}

// ==================== CURVE EVALUATION FUNCTIONS ====================

struct CurveResult
{
  gp_Pnt P;
  gp_Vec D1;
  gp_Vec D2;
};

/**
 * Evaluate points on curves with derivative degree 0 (just points)
 */
void evaluateCurvesDegree0(const std::vector<BRepAdaptor_Curve>& curves,
                           const std::vector<std::vector<double>>& allParams,
                           std::vector<std::vector<CurveResult>>& allPoints)
{
  for (size_t cidx = 0; cidx < curves.size(); ++cidx) {
    const auto& curve = curves[cidx];
    const auto& params = allParams[cidx];
    auto& points = allPoints[cidx];

    int i = 0;
    for (double t : params) {
      points[i++].P = curve.Value(t);
    }
  }
}

/**
 * Evaluate points on curves with derivative degree 1 (point and first derivative)
 */
void evaluateCurvesDegree1(const std::vector<BRepAdaptor_Curve>& curves,
                           const std::vector<std::vector<double>>& allParams,
                           std::vector<std::vector<CurveResult>>& allPoints)
{
  for (size_t cidx = 0; cidx < curves.size(); ++cidx) {
    const auto& curve = curves[cidx];
    const auto& params = allParams[cidx];
    auto& points = allPoints[cidx];

    int i = 0;
    for (double t : params) {
      curve.D1(t, points[i].P, points[i].D1);
      i++;
    }
  }
}

/**
 * Evaluate points on curves with derivative degree 2 (point, first and second derivatives)
 */
void evaluateCurvesDegree2(const std::vector<BRepAdaptor_Curve>& curves,
                           const std::vector<std::vector<double>>& allParams,
                           std::vector<std::vector<CurveResult>>& allPoints)
{
  for (size_t cidx = 0; cidx < curves.size(); ++cidx) {
    const auto& curve = curves[cidx];
    const auto& params = allParams[cidx];
    auto& points = allPoints[cidx];

    int i = 0;
    for (double t : params) {
      curve.D2(t, points[i].P, points[i].D1, points[i].D2);
      i++;
    }
  }
}

/**
 * Dump curve evaluation results
 */
void dumpCurveResults(const std::vector<BRepAdaptor_Curve>& curves,
                      const std::vector<std::vector<double>>& allParams,
                      const std::vector<std::vector<CurveResult>>& allPoints, int derivDegree)
{
  std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);
  for (size_t cidx = 0; cidx < curves.size(); ++cidx) {
    const auto& curve = curves[cidx];
    const auto& params = allParams[cidx];
    const auto& points = allPoints[cidx];

    if (curve.GetType() == GeomAbs_BSplineCurve)
    {
      const auto& spline = curve.BSpline();
      std::cout << "Curve " << (cidx + 1) << " degree " << spline->Degree();
      std::cout << " nbknots " << spline->NbKnots() << " rational " << spline->IsRational() << std::endl;
    }
    else
    {
      static const std::array<const char*, 9> curveTypeNames = {
        "Line", "Circle", "Ellipse", "Hyperbola", "Parabola", "BezierCurve", "BSplineCurve", "OffsetCurve", "OtherCurve"
      };
      std::cout << "Curve " << (cidx + 1) << " is " << curveTypeNames[curve.GetType() - GeomAbs_Line] << std::endl;
    }

    if (derivDegree < 0) {
      continue;
    }

    int i = 0;
    for (double t : params) {
      std::cout << " t " << t;

      const gp_Pnt& P = points[i].P;
      const gp_Vec& D1 = points[i].D1, D2 = points[i].D2;

      std::cout << " P (" << P.X() << "," << P.Y() << "," << P.Z() << ")";
      if (derivDegree > 0)
      {
        std::cout << " D1 (" << D1.X() << "," << D1.Y() << "," << D1.Z() << ")";
      }
      if (derivDegree > 1)
      {
        std::cout << " D2 (" << D2.X() << "," << D2.Y() << "," << D2.Z() << ")";
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }
}

// ==================== SURFACE EVALUATION FUNCTIONS (NO OUTPUT) ====================

struct SurfaceResult
{
  gp_Pnt P;
  gp_Vec D1u, D1v;
  gp_Vec D2uu, D2vv, D2uv;

  static inline gp_XYZ nanXYZ()
  {
    return { NAN, NAN, NAN };
  }

  SurfaceResult() : P(nanXYZ()), D1u(nanXYZ()), D1v(nanXYZ()), D2uu(nanXYZ()), D2vv(nanXYZ()), D2uv(nanXYZ()) {}
};

/**
 * Evaluate points on surfaces with derivative degree 0 (just points)
 */
void evaluateSurfacesDegree0(const std::vector<BRepAdaptor_Surface>& surfaces,
                             const std::vector<std::vector<std::pair<double, double>>>& allUVPoints,
                             std::vector<std::vector<SurfaceResult>>& allPoints)
{
  for (size_t sidx = 0; sidx < surfaces.size(); ++sidx) {
    const auto& surf = surfaces[sidx];
    const auto& uvPoints = allUVPoints[sidx];
    auto& points = allPoints[sidx];

    int i = 0;
    for (auto [u, v] : uvPoints) {
      points[i++].P = surf.Value(u, v);
    }
  }
}

/**
 * Evaluate points on surfaces with derivative degree 1 (point and first derivatives)
 */
void evaluateSurfacesDegree1(const std::vector<BRepAdaptor_Surface>& surfaces,
                             const std::vector<std::vector<std::pair<double, double>>>& allUVPoints,
                             std::vector<std::vector<SurfaceResult>>& allPoints)
{
  for (size_t sidx = 0; sidx < surfaces.size(); ++sidx) {
    const auto& surf = surfaces[sidx];
    const auto& uvPoints = allUVPoints[sidx];
    auto& points = allPoints[sidx];

    int i = 0;
    for (auto [u, v] : uvPoints) {
      SurfaceResult& r = points[i++];
      surf.D1(u, v, r.P, r.D1u, r.D1v);
    }
  }
}

/**
 * Evaluate points on surfaces with derivative degree 2 (point, first and second derivatives)
 */
void evaluateSurfacesDegree2(const std::vector<BRepAdaptor_Surface>& surfaces,
                             const std::vector<std::vector<std::pair<double, double>>>& allUVPoints,
                             std::vector<std::vector<SurfaceResult>>& allPoints)
{
  for (size_t sidx = 0; sidx < surfaces.size(); ++sidx) {
    const auto& surf = surfaces[sidx];
    const auto& uvPoints = allUVPoints[sidx];
    auto& points = allPoints[sidx];

    int i = 0;
    for (auto [u, v] : uvPoints) {
      SurfaceResult& r = points[i++];
      surf.D2(u, v, r.P, r.D1u, r.D1v, r.D2uu, r.D2vv, r.D2uv);
    }
  }
}

#ifdef HAS_TINYNURBS

/**
 * Convert an Open CASCADE B-spline surface to a TinyNURBS surface.
 *
 * @param surf OCCT handle to Geom_BSplineSurface.
 * @return A TinyNURBS Surface (non-rational) or RationalSurface (if rational).
 *         Use std::variant or overloads to handle both cases.
 *
 * Note: TinyNURBS uses glm::vec3 for points and array2<T> for 2D structures.
 */
//std::variant<tinynurbs::Surface<double>, tinynurbs::RationalSurface<double>>
tinynurbs::Surface<double> ConvertToTinyNurbs(const Handle(Geom_BSplineSurface)& surf)
{
  if (surf.IsNull()) {
    throw std::runtime_error("Surface is not a BSpline, not supported by TinyNurbs library");
  }
  if (surf->IsURational() || surf->IsVRational()) {
    throw std::runtime_error("Evaluation of rational surfaces in TinyNurbs is extremely slow, not supported");
  }

  unsigned int degU = surf->UDegree();
  unsigned int degV = surf->VDegree();

  const TColStd_Array1OfReal& uKnots = surf->UKnotSequence();
  const TColStd_Array1OfReal& vKnots = surf->VKnotSequence();

  std::vector<double> knotsU, knotsV;
  for (int i = uKnots.Lower(); i <= uKnots.Upper(); ++i) {
    knotsU.push_back(uKnots(i));
  }
  for (int i = vKnots.Lower(); i <= vKnots.Upper(); ++i) {
    knotsV.push_back(vKnots(i));
  }

  int nbPolesU = surf->NbUPoles();
  int nbPolesV = surf->NbVPoles();

  const TColgp_Array2OfPnt& poles = surf->Poles();
  const TColStd_Array2OfReal*  weights = surf->Weights();

  tinynurbs::array2<glm::vec<3, double>> ctrlPts(nbPolesU, nbPolesV);
//  tinynurbs::array2<double> wVec(nbPolesU, nbPolesV);

  for (int i = 1; i <= nbPolesU; ++i) {
    for (int j = 1; j <= nbPolesV; ++j) {
      const gp_Pnt& p = poles(i, j);
      ctrlPts(i - 1, j - 1) = glm::vec3(p.X(), p.Y(), p.Z());
//      wVec(i - 1, j - 1) = weights->Value(i, j);
    }
  }

  // RationalSurface constructor:
  // RationalSurface(deg_u, deg_v, knots_u, knots_v, control_points, weights)
//  if (surf->IsURational() || surf->IsVRational()) {
//    return tinynurbs::RationalSurface<double>(degU, degV, knotsU, knotsV, ctrlPts, wVec);
  //}
  //else {
    return tinynurbs::Surface<double>(degU, degV, knotsU, knotsV, ctrlPts);
  //}
}

/**
 * Evaluate points on surfaces with derivative degree 0 (just points)
 */
void evaluateSurfacesDegree0(const std::vector<tinynurbs::Surface<double>>& surfaces,
                             const std::vector<std::vector<std::pair<double, double>>>& allUVPoints,
                             std::vector<std::vector<SurfaceResult>>& allPoints)
{
  for (size_t sidx = 0; sidx < surfaces.size(); ++sidx) {
    const auto& surf = surfaces[sidx];
    const auto& uvPoints = allUVPoints[sidx];
    auto& points = allPoints[sidx];

    int i = 0;
    for (auto& [u, v] : uvPoints) {
      auto pnt = tinynurbs::surfacePoint(surf, u, v);
      points[i++].P.SetCoord(pnt.x, pnt.y, pnt.z);
    }
  }
}

/**
 * Evaluate points on surfaces with derivative degree 1
 */
void evaluateSurfacesDegree1(const std::vector<tinynurbs::Surface<double>>& surfaces,
                             const std::vector<std::vector<std::pair<double, double>>>& allUVPoints,
                             std::vector<std::vector<SurfaceResult>>& allPoints)
{
  for (size_t sidx = 0; sidx < surfaces.size(); ++sidx) {
    const auto& surf = surfaces[sidx];
    const auto& uvPoints = allUVPoints[sidx];
    auto& points = allPoints[sidx];

    int i = 0;
    for (auto& [u, v] : uvPoints) {
      auto pnt = tinynurbs::surfacePoint(surf, u, v);
      auto d1 = tinynurbs::surfaceDerivatives(surf, 1, u, v);
      SurfaceResult& r = points[i++];
      r.P.SetCoord(d1(0, 0).x, d1(0, 0).y, d1(0, 0).z);
      r.D1u.SetCoord(d1(1, 0).x, d1(1, 0).y, d1(1, 0).z);
      r.D1v.SetCoord(d1(0, 1).x, d1(0, 1).y, d1(0, 1).z);
    }
  }
}

/**
 * Evaluate points on surfaces with derivative degree 2
 */
void evaluateSurfacesDegree2(const std::vector<tinynurbs::Surface<double>>& surfaces,
                             const std::vector<std::vector<std::pair<double, double>>>& allUVPoints,
                             std::vector<std::vector<SurfaceResult>>& allPoints)
{
  for (size_t sidx = 0; sidx < surfaces.size(); ++sidx) {
    const auto& surf = surfaces[sidx];
    const auto& uvPoints = allUVPoints[sidx];
    auto& points = allPoints[sidx];

    int i = 0;
    for (auto& [u, v] : uvPoints) {
      auto pnt = tinynurbs::surfacePoint(surf, u, v);
      auto d2 = tinynurbs::surfaceDerivatives(surf, 2, u, v);
      SurfaceResult& r = points[i++];
      r.P.SetCoord(d2(0, 0).x, d2(0, 0).y, d2(0, 0).z);
      r.D1u.SetCoord(d2(1, 0).x, d2(1, 0).y, d2(1, 0).z);
      r.D1v.SetCoord(d2(0, 1).x, d2(0, 1).y, d2(0, 1).z);
      r.D2uu.SetCoord(d2(2, 0).x, d2(2, 0).y, d2(2, 0).z);
      r.D2vv.SetCoord(d2(0, 2).x, d2(0, 2).y, d2(0, 2).z);
      r.D2uv.SetCoord(d2(1, 1).x, d2(1, 1).y, d2(1, 1).z);
    }
  }
}

#endif /* HAS_TINYNURBS */

#ifdef HAS_SISL

struct SISLSurfDeleter
{
  void operator()(SISLSurf* surf) const
  {
    if (surf) {
      freeSurf(surf);
    }
  }
};

// Type alias for convenience
using UniqueSISLSurf = std::unique_ptr<SISLSurf, SISLSurfDeleter>;

/**
 * Convert an Open CASCADE B-spline surface to a SISL SISLSurf structure.
 *
 * @param surf  OCCT surface handle (Geom_BSplineSurface)
 * @return      SISL surface pointer, or null on failure
 */
UniqueSISLSurf ConvertToSISL(const Handle(Geom_BSplineSurface)& surf)
{
  if (surf.IsNull())
    return nullptr;

  // 1. Degrees and orders
  int degU = surf->UDegree();
  int degV = surf->VDegree();
  int orderU = degU + 1;
  int orderV = degV + 1;

  int nbPolesU = surf->NbUPoles();
  int nbPolesV = surf->NbVPoles();

  bool isRational = surf->IsURational() || surf->IsVRational();
  const int dim = 3;

  // 2. Knot vectors - directly from the pre-expanded sequences
  const TColStd_Array1OfReal& seqU = surf->UKnotSequence();
  const TColStd_Array1OfReal& seqV = surf->VKnotSequence();

  // Copy to std::vector for SISL
  std::vector<double> knotU(seqU.Length());
  std::vector<double> knotV(seqV.Length());

  for (int i = seqU.Lower(); i <= seqU.Upper(); ++i)
    knotU[i - seqU.Lower()] = seqU(i);
  for (int i = seqV.Lower(); i <= seqV.Upper(); ++i)
    knotV[i - seqV.Lower()] = seqV(i);

  // 3. Control points and weights
  const TColgp_Array2OfPnt& poles = surf->Poles();
  const TColStd_Array2OfReal* weights = surf->Weights();

  // 4. Build coefficient array
  int coefStride = isRational ? (dim + 1) : dim;
  int totalCoef = nbPolesU * nbPolesV * coefStride;
  std::vector<double> coef(totalCoef);

  for (int i = 1; i <= nbPolesU; ++i) {
    for (int j = 1; j <= nbPolesV; ++j) {
      int idx = ((j - 1) * nbPolesU + (i - 1)) * coefStride;

      const gp_Pnt& p = poles(i, j);
      coef[idx + 0] = p.X();
      coef[idx + 1] = p.Y();
      coef[idx + 2] = p.Z();

      if (isRational) {
        coef[idx + 3] = (weights ? weights->Value(i, j) : 1.);
      }
    }
  }

  int kind = isRational ? 2 : 1;

  // 6. Create SISL surface
  UniqueSISLSurf result(newSurf(nbPolesU, nbPolesV, orderU, orderV,
                                knotU.data(), knotV.data(), coef.data(), kind, dim, true));

  return result;
}

/**
 * Evaluate points on surfaces with derivative degree 0 (just points)
 */
void evaluateSurfacesDegree0(const std::vector<UniqueSISLSurf>& surfaces,
                             const std::vector<std::vector<std::pair<double, double>>>& allUVPoints,
                             std::vector<std::vector<SurfaceResult>>& allPoints)
{
  for (size_t sidx = 0; sidx < surfaces.size(); ++sidx) {
    const auto& surf = surfaces[sidx];
    const auto& uvPoints = allUVPoints[sidx];
    auto& points = allPoints[sidx];

    int i = 0, jstat = 0;
    int ileft1 = 0, ileft2 = 0; // index of interval cached?
    for (auto& uvpair : uvPoints) {
      double uv[2] = { uvpair.first, uvpair.second };
      double eder[3];
      s1425(surf.get(), 0, 0, 0, 0, uv, &ileft1, &ileft2, eder, &jstat);
      if (jstat) {
        std::cerr << "Point evaluation failed with error code: " << jstat << std::endl;
      }
      points[i++].P.SetCoord(eder[0], eder[1], eder[2]);
    }
  }
}

/**
 * Evaluate points on surfaces with derivative degree 1
 */
void evaluateSurfacesDegree1(const std::vector<UniqueSISLSurf>& surfaces,
                             const std::vector<std::vector<std::pair<double, double>>>& allUVPoints,
                             std::vector<std::vector<SurfaceResult>>& allPoints)
{
  for (size_t sidx = 0; sidx < surfaces.size(); ++sidx) {
    const auto& surf = surfaces[sidx];
    const auto& uvPoints = allUVPoints[sidx];
    auto& points = allPoints[sidx];

    int i = 0, jstat = 0;
    int ileft1 = 0, ileft2 = 0; // index of interval cached?
    for (auto& uvpair : uvPoints) {
      double uv[2] = { uvpair.first, uvpair.second };
      double eder[12];
      s1425(surf.get(), 1, 1, 0, 0, uv, &ileft1, &ileft2, eder, &jstat);
      if (jstat) {
        std::cerr << "Point evaluation failed with error code: " << jstat << std::endl;
      }
      SurfaceResult& r = points[i++];
      r.P.SetCoord(eder[0], eder[1], eder[2]);
      r.D1u.SetCoord(eder[3], eder[4], eder[5]);
      r.D1v.SetCoord(eder[6], eder[7], eder[8]);
    }
  }
}

/**
 * Evaluate points on surfaces with derivative degree 2
 */
void evaluateSurfacesDegree2(const std::vector<UniqueSISLSurf>& surfaces,
                             const std::vector<std::vector<std::pair<double, double>>>& allUVPoints,
                             std::vector<std::vector<SurfaceResult>>& allPoints)
{
  for (size_t sidx = 0; sidx < surfaces.size(); ++sidx) {
    const auto& surf = surfaces[sidx];
    const auto& uvPoints = allUVPoints[sidx];
    auto& points = allPoints[sidx];

    int i = 0, jstat = 0;
    int ileft1 = 0, ileft2 = 0; // index of interval cached?
    for (auto& uvpair : uvPoints) {
      double uv[2] = { uvpair.first, uvpair.second };
      double eder[27];
      s1425(surf.get(), 2, 2, 0, 0, uv, &ileft1, &ileft2, eder, &jstat);
      if (jstat) {
        std::cerr << "Point evaluation failed with error code: " << jstat << std::endl;
      }
      SurfaceResult& r = points[i++];
      r.P.SetCoord(eder[0], eder[1], eder[2]);
      r.D1u.SetCoord(eder[3], eder[4], eder[5]);
      r.D2uu.SetCoord(eder[6], eder[7], eder[8]);
      r.D1v.SetCoord(eder[9], eder[10], eder[11]);
      r.D2uv.SetCoord(eder[12], eder[13], eder[14]);
      // Duuv not used
      r.D2vv.SetCoord(eder[18], eder[19], eder[20]);
      // Duvv not used
      // Duuvv not used
    }
  }
}

#endif /* HAS_SISL */

#ifdef HAS_ACIS

// ==================== ACIS BACKEND ====================
//
// Evaluation strategy: ACIS has no user-facing "adaptor + explicit cache" object
// like OCCT's BRepAdaptor_Surface / BSplSLib_Cache. Its B-spline classes
// (bs3_curve, bs3_surface) cache the active span internally, so the
// spline-interface evaluators used below -- bs3_curve_eval() and
// bs3_surface_evaluate() -- are the direct, lowest-overhead entry points and the
// closest analogue to OCCT's cached evaluation of a Geom_BSpline*.
// Alternatives (see README): the virtual curve::eval()/surface::eval() member
// overloads, bs3_curve_evaluate()/bs3_surface_evaluate() with an arbitrary
// derivative count, and the api_* / evaluation-through-FACE/EDGE routes.
//
// NOTE: bs3_curve / bs3_surface constructor signatures vary slightly between
// ACIS releases. Verify the two `new bs3_curve(...)` / `new bs3_surface(...)`
// calls below against the bs3curve / bs3surf headers of your ACIS version.

static void AcisInitOnce()
{
  static bool s_initialized = false;
  if (!s_initialized) {
    api_initialize_acis();
    s_initialized = true;
  }
}

struct ACISCurveDeleter { void operator()(bs3_curve* c) const { delete c; } };
struct ACISSurfDeleter  { void operator()(bs3_surface* s) const { delete s; } };
using UniqueACISCurve = std::unique_ptr<bs3_curve, ACISCurveDeleter>;
using UniqueACISSurf  = std::unique_ptr<bs3_surface, ACISSurfDeleter>;

/**
 * Build an ACIS bs3_curve from an OCCT B-spline curve.
 */
UniqueACISCurve ConvertToACIS(const Handle(Geom_BSplineCurve)& curve)
{
  AcisInitOnce();
  if (curve.IsNull())
    throw std::runtime_error("Curve is not a BSpline, not supported by ACIS backend");

  const int order  = curve->Degree() + 1;
  const int nPoles = curve->NbPoles();

  // ACIS expects the fully expanded knot sequence (with multiplicities).
  const TColStd_Array1OfReal& knots = curve->KnotSequence();
  const int nKnots = knots.Length();
  std::vector<double> knotVec(nKnots);
  for (int i = knots.Lower(); i <= knots.Upper(); ++i)
    knotVec[i - knots.Lower()] = knots(i);

  const TColgp_Array1OfPnt& poles = curve->Poles();
  std::vector<SPAposition> poleVec(nPoles);
  for (int i = 1; i <= nPoles; ++i) {
    const gp_Pnt& p = poles(i);
    poleVec[i - 1] = SPAposition(p.X(), p.Y(), p.Z());
  }

  std::vector<double> weightVec;
  double* weights = nullptr;
  if (curve->IsRational()) {
    const TColStd_Array1OfReal* w = curve->Weights();
    weightVec.resize(nPoles);
    for (int i = 1; i <= nPoles; ++i)
      weightVec[i - 1] = w->Value(i);
    weights = weightVec.data();
  }

  // bs3_curve(order, n_knots, knots, n_poles, poles, weights)
  return UniqueACISCurve(new bs3_curve(order, nKnots, knotVec.data(),
                                       nPoles, poleVec.data(), weights));
}

/**
 * Build an ACIS bs3_surface from an OCCT B-spline surface.
 */
UniqueACISSurf ConvertToACIS(const Handle(Geom_BSplineSurface)& surf)
{
  AcisInitOnce();
  if (surf.IsNull())
    throw std::runtime_error("Surface is not a BSpline, not supported by ACIS backend");

  const int uOrder = surf->UDegree() + 1;
  const int vOrder = surf->VDegree() + 1;
  const int nU = surf->NbUPoles();
  const int nV = surf->NbVPoles();

  const TColStd_Array1OfReal& uk = surf->UKnotSequence();
  const TColStd_Array1OfReal& vk = surf->VKnotSequence();
  std::vector<double> uKnots(uk.Length()), vKnots(vk.Length());
  for (int i = uk.Lower(); i <= uk.Upper(); ++i) uKnots[i - uk.Lower()] = uk(i);
  for (int i = vk.Lower(); i <= vk.Upper(); ++i) vKnots[i - vk.Lower()] = vk(i);

  const TColgp_Array2OfPnt& poles = surf->Poles();
  const bool rational = surf->IsURational() || surf->IsVRational();
  const TColStd_Array2OfReal* weights = surf->Weights();

  // ACIS stores the control net u-major: flat index = iU * nV + iV.
  std::vector<SPAposition> poleVec(static_cast<size_t>(nU) * nV);
  std::vector<double> weightVec;
  if (rational) weightVec.resize(static_cast<size_t>(nU) * nV);
  for (int i = 1; i <= nU; ++i) {
    for (int j = 1; j <= nV; ++j) {
      const size_t k = static_cast<size_t>(i - 1) * nV + (j - 1);
      const gp_Pnt& p = poles(i, j);
      poleVec[k] = SPAposition(p.X(), p.Y(), p.Z());
      if (rational) weightVec[k] = weights->Value(i, j);
    }
  }

  // bs3_surface(nU, nV, uOrder, vOrder, uKnots, vKnots, poles, weights)
  return UniqueACISSurf(new bs3_surface(nU, nV, uOrder, vOrder,
                                        uKnots.data(), vKnots.data(),
                                        poleVec.data(),
                                        rational ? weightVec.data() : nullptr));
}

// ---- curve evaluation ----
void evaluateCurvesDegree0(const std::vector<UniqueACISCurve>& curves,
                           const std::vector<std::vector<double>>& allParams,
                           std::vector<std::vector<CurveResult>>& allPoints)
{
  for (size_t cidx = 0; cidx < curves.size(); ++cidx) {
    const bs3_curve& c = *curves[cidx];
    const auto& params = allParams[cidx];
    auto& points = allPoints[cidx];
    int i = 0;
    for (double t : params) {
      SPAposition pos;
      bs3_curve_eval(t, c, pos);
      points[i++].P.SetCoord(pos.x(), pos.y(), pos.z());
    }
  }
}

void evaluateCurvesDegree1(const std::vector<UniqueACISCurve>& curves,
                           const std::vector<std::vector<double>>& allParams,
                           std::vector<std::vector<CurveResult>>& allPoints)
{
  for (size_t cidx = 0; cidx < curves.size(); ++cidx) {
    const bs3_curve& c = *curves[cidx];
    const auto& params = allParams[cidx];
    auto& points = allPoints[cidx];
    int i = 0;
    for (double t : params) {
      SPAposition pos; SPAvector d1;
      bs3_curve_eval(t, c, pos, d1);
      CurveResult& r = points[i++];
      r.P.SetCoord(pos.x(), pos.y(), pos.z());
      r.D1.SetCoord(d1.x(), d1.y(), d1.z());
    }
  }
}

void evaluateCurvesDegree2(const std::vector<UniqueACISCurve>& curves,
                           const std::vector<std::vector<double>>& allParams,
                           std::vector<std::vector<CurveResult>>& allPoints)
{
  for (size_t cidx = 0; cidx < curves.size(); ++cidx) {
    const bs3_curve& c = *curves[cidx];
    const auto& params = allParams[cidx];
    auto& points = allPoints[cidx];
    int i = 0;
    for (double t : params) {
      SPAposition pos; SPAvector d1, d2;
      bs3_curve_eval(t, c, pos, d1, d2);
      CurveResult& r = points[i++];
      r.P.SetCoord(pos.x(), pos.y(), pos.z());
      r.D1.SetCoord(d1.x(), d1.y(), d1.z());
      r.D2.SetCoord(d2.x(), d2.y(), d2.z());
    }
  }
}

// ---- surface evaluation ----
// deriv[k] points to the (k+1)-th order derivative array, holding (k+2) vectors:
//   deriv[0] -> [du, dv];  deriv[1] -> [duu, duv, dvv].
void evaluateSurfacesDegree0(const std::vector<UniqueACISSurf>& surfaces,
                             const std::vector<std::vector<std::pair<double, double>>>& allUVPoints,
                             std::vector<std::vector<SurfaceResult>>& allPoints)
{
  for (size_t sidx = 0; sidx < surfaces.size(); ++sidx) {
    const bs3_surface& s = *surfaces[sidx];
    const auto& uvPoints = allUVPoints[sidx];
    auto& points = allPoints[sidx];
    int i = 0;
    for (auto& [u, v] : uvPoints) {
      SPApar_pos uv(u, v);
      SPAposition pos;
      bs3_surface_evaluate(uv, s, pos);
      points[i++].P.SetCoord(pos.x(), pos.y(), pos.z());
    }
  }
}

void evaluateSurfacesDegree1(const std::vector<UniqueACISSurf>& surfaces,
                             const std::vector<std::vector<std::pair<double, double>>>& allUVPoints,
                             std::vector<std::vector<SurfaceResult>>& allPoints)
{
  for (size_t sidx = 0; sidx < surfaces.size(); ++sidx) {
    const bs3_surface& s = *surfaces[sidx];
    const auto& uvPoints = allUVPoints[sidx];
    auto& points = allPoints[sidx];
    int i = 0;
    for (auto& [u, v] : uvPoints) {
      SPApar_pos uv(u, v);
      SPAposition pos;
      SPAvector d1[2];                 // [du, dv]
      SPAvector* deriv[1] = { d1 };
      bs3_surface_evaluate(uv, s, pos, deriv, 1);
      SurfaceResult& r = points[i++];
      r.P.SetCoord(pos.x(), pos.y(), pos.z());
      r.D1u.SetCoord(d1[0].x(), d1[0].y(), d1[0].z());
      r.D1v.SetCoord(d1[1].x(), d1[1].y(), d1[1].z());
    }
  }
}

void evaluateSurfacesDegree2(const std::vector<UniqueACISSurf>& surfaces,
                             const std::vector<std::vector<std::pair<double, double>>>& allUVPoints,
                             std::vector<std::vector<SurfaceResult>>& allPoints)
{
  for (size_t sidx = 0; sidx < surfaces.size(); ++sidx) {
    const bs3_surface& s = *surfaces[sidx];
    const auto& uvPoints = allUVPoints[sidx];
    auto& points = allPoints[sidx];
    int i = 0;
    for (auto& [u, v] : uvPoints) {
      SPApar_pos uv(u, v);
      SPAposition pos;
      SPAvector d1[2];                 // [du, dv]
      SPAvector d2[3];                 // [duu, duv, dvv]
      SPAvector* deriv[2] = { d1, d2 };
      bs3_surface_evaluate(uv, s, pos, deriv, 2);
      SurfaceResult& r = points[i++];
      r.P.SetCoord(pos.x(), pos.y(), pos.z());
      r.D1u.SetCoord(d1[0].x(), d1[0].y(), d1[0].z());
      r.D1v.SetCoord(d1[1].x(), d1[1].y(), d1[1].z());
      r.D2uu.SetCoord(d2[0].x(), d2[0].y(), d2[0].z());
      r.D2uv.SetCoord(d2[1].x(), d2[1].y(), d2[1].z());
      r.D2vv.SetCoord(d2[2].x(), d2[2].y(), d2[2].z());
    }
  }
}

#endif /* HAS_ACIS */

#ifdef HAS_PARASOLID

// ==================== PARASOLID BACKEND ====================
//
// Evaluation strategy: Parasolid evaluates a geometry entity directly through
// PK_SURF_eval / PK_CURVE_eval; B-surfaces and B-curves cache their active span
// internally, so this is the direct analogue of OCCT's cached geometric
// evaluation. Alternatives (see README): evaluating through a FACE
// (PK_EVAL_face_* / PK_FACE_*), which adds the face transform and sense on top
// of the surface, and the batched PK_SURF_eval_grid, which evaluates a whole
// rectangular grid of parameters in one call.
//
// A Parasolid session needs a "frustrum" (memory + I/O callbacks). Only memory
// and start/stop are exercised here; the file callbacks are inert stubs because
// this benchmark builds entities in memory and never reads or writes Parasolid
// files. Signatures follow the standard Parasolid frustrum. Requires a valid
// Parasolid licence/schema at run time.

static int g_parasolidStarts = 0;

static void PS_FSTART(int* ifail) { ++g_parasolidStarts; *ifail = 0; }
static void PS_FSTOP(int* ifail)  { if (g_parasolidStarts > 0) --g_parasolidStarts; *ifail = 0; }
static void PS_FMALLO(int* nbytes, char** memory, int* ifail)
{
  *memory = static_cast<char*>(malloc(*nbytes));
  *ifail = (*memory != nullptr) ? 0 : 1;   // 0 == FR_no_errors
}
static void PS_FMFREE(int* /*nbytes*/, char** memory, int* ifail) { free(*memory); *ifail = 0; }
static void PS_FFOPRD(const int*, const int*, const char*, const int*, const int*, int* strid, int* ifail) { *strid = -1; *ifail = 1; }
static void PS_FFOPWR(const int*, const int*, const char*, const int*, const char*, const int*, int* strid, int* ifail) { *strid = -1; *ifail = 1; }
static void PS_FFWRIT(const int*, const int*, const int*, const char*, int* ifail) { *ifail = 1; }
static void PS_FFREAD(const int*, const int*, const int*, char*, int* nactual, int* ifail) { *nactual = 0; *ifail = 1; }
static void PS_FFCLOS(const int*, const int*, const int*, int* ifail) { *ifail = 1; }

static void ParasolidInitOnce()
{
  static bool s_started = false;
  if (s_started) return;

  PK_SESSION_frustrum_t fru;
  PK_SESSION_frustrum_o_m(fru);
  fru.fstart = PS_FSTART;
  fru.fstop  = PS_FSTOP;
  fru.fmallo = PS_FMALLO;
  fru.fmfree = PS_FMFREE;
  fru.ffoprd = PS_FFOPRD;
  fru.ffopwr = PS_FFOPWR;
  fru.ffclos = PS_FFCLOS;
  fru.ffread = PS_FFREAD;
  fru.ffwrit = PS_FFWRIT;
  PK_SESSION_register_frustrum(&fru);

  PK_SESSION_start_o_t opts;
  PK_SESSION_start_o_m(opts);
  PK_ERROR_code_t err = PK_SESSION_start(&opts);
  if (err != PK_ERROR_no_errors) {
    throw std::runtime_error("Failed to start Parasolid session (error " +
                             std::to_string(static_cast<int>(err)) +
                             "); a valid Parasolid licence/schema is required");
  }
  s_started = true;
}

/**
 * Build a Parasolid b-curve (evaluated as a PK_CURVE_t) from an OCCT B-spline.
 */
PK_CURVE_t ConvertToParasolid(const Handle(Geom_BSplineCurve)& curve)
{
  ParasolidInitOnce();
  if (curve.IsNull())
    throw std::runtime_error("Curve is not a BSpline, not supported by Parasolid backend");

  const int deg = curve->Degree();
  const int nV  = curve->NbPoles();
  const bool rational = curve->IsRational();

  // Distinct knots + multiplicities map directly onto the standard form.
  const int nK = curve->NbKnots();
  std::vector<double> knot(nK);
  std::vector<int> mult(nK);
  for (int i = 1; i <= nK; ++i) { knot[i - 1] = curve->Knot(i); mult[i - 1] = curve->Multiplicity(i); }

  // Rational vertices are homogeneous: (x*w, y*w, z*w, w).
  const int vdim = rational ? 4 : 3;
  const TColgp_Array1OfPnt& poles = curve->Poles();
  const TColStd_Array1OfReal* weights = curve->Weights();
  std::vector<double> vertex(static_cast<size_t>(nV) * vdim);
  for (int i = 1; i <= nV; ++i) {
    const size_t k = static_cast<size_t>(i - 1) * vdim;
    const gp_Pnt& p = poles(i);
    const double w = rational ? weights->Value(i) : 1.0;
    vertex[k + 0] = p.X() * w;
    vertex[k + 1] = p.Y() * w;
    vertex[k + 2] = p.Z() * w;
    if (rational) vertex[k + 3] = w;
  }

  PK_BCURVE_sf_t sf{};   // advisory enums (form/knot_type/...) left at their unset default
  sf.degree      = deg;
  sf.n_vertices  = nV;
  sf.vertex_dim  = vdim;
  sf.is_rational = rational ? PK_LOGICAL_true : PK_LOGICAL_false;
  sf.vertex      = vertex.data();
  sf.n_knots     = nK;
  sf.knot_mult   = mult.data();
  sf.knot        = knot.data();

  PK_BCURVE_t bcurve{};
  PK_ERROR_code_t err = PK_BCURVE_create(&sf, &bcurve);
  if (err != PK_ERROR_no_errors)
    throw std::runtime_error("PK_BCURVE_create failed (error " + std::to_string(static_cast<int>(err)) + ")");

  return static_cast<PK_CURVE_t>(bcurve);   // a b-curve is a curve entity
}

/**
 * Build a Parasolid b-surface (evaluated as a PK_SURF_t) from an OCCT B-spline.
 */
PK_SURF_t ConvertToParasolid(const Handle(Geom_BSplineSurface)& surf)
{
  ParasolidInitOnce();
  if (surf.IsNull())
    throw std::runtime_error("Surface is not a BSpline, not supported by Parasolid backend");

  const int uDeg = surf->UDegree();
  const int vDeg = surf->VDegree();
  const int nU = surf->NbUPoles();
  const int nV = surf->NbVPoles();
  const bool rational = surf->IsURational() || surf->IsVRational();

  const int nUK = surf->NbUKnots();
  const int nVK = surf->NbVKnots();
  std::vector<double> uKnot(nUK), vKnot(nVK);
  std::vector<int> uMult(nUK), vMult(nVK);
  for (int i = 1; i <= nUK; ++i) { uKnot[i - 1] = surf->UKnot(i); uMult[i - 1] = surf->UMultiplicity(i); }
  for (int i = 1; i <= nVK; ++i) { vKnot[i - 1] = surf->VKnot(i); vMult[i - 1] = surf->VMultiplicity(i); }

  // Vertices vary most quickly with v, then u; rational vertices are homogeneous.
  const int vdim = rational ? 4 : 3;
  const TColgp_Array2OfPnt& poles = surf->Poles();
  const TColStd_Array2OfReal* weights = surf->Weights();
  std::vector<double> vertex(static_cast<size_t>(nU) * nV * vdim);
  for (int i = 1; i <= nU; ++i) {
    for (int j = 1; j <= nV; ++j) {
      const size_t k = (static_cast<size_t>(i - 1) * nV + (j - 1)) * vdim;
      const gp_Pnt& p = poles(i, j);
      const double w = rational ? weights->Value(i, j) : 1.0;
      vertex[k + 0] = p.X() * w;
      vertex[k + 1] = p.Y() * w;
      vertex[k + 2] = p.Z() * w;
      if (rational) vertex[k + 3] = w;
    }
  }

  PK_BSURF_sf_t sf{};    // advisory enums (form/knot_type/...) left at their unset default
  sf.u_degree     = uDeg;
  sf.v_degree     = vDeg;
  sf.n_u_vertices = nU;
  sf.n_v_vertices = nV;
  sf.vertex_dim   = vdim;
  sf.is_rational  = rational ? PK_LOGICAL_true : PK_LOGICAL_false;
  sf.vertex       = vertex.data();
  sf.n_u_knots    = nUK;
  sf.n_v_knots    = nVK;
  sf.u_knot_mult  = uMult.data();
  sf.v_knot_mult  = vMult.data();
  sf.u_knot       = uKnot.data();
  sf.v_knot       = vKnot.data();

  PK_BSURF_t bsurf{};
  PK_ERROR_code_t err = PK_BSURF_create(&sf, &bsurf);
  if (err != PK_ERROR_no_errors)
    throw std::runtime_error("PK_BSURF_create failed (error " + std::to_string(static_cast<int>(err)) + ")");

  return static_cast<PK_SURF_t>(bsurf);     // a b-surface is a surface entity
}

// ---- curve evaluation: p[0]=point, p[1]=1st deriv, p[2]=2nd deriv ----
void evaluateCurvesDegree0(const std::vector<PK_CURVE_t>& curves,
                           const std::vector<std::vector<double>>& allParams,
                           std::vector<std::vector<CurveResult>>& allPoints)
{
  for (size_t cidx = 0; cidx < curves.size(); ++cidx) {
    PK_CURVE_t c = curves[cidx];
    const auto& params = allParams[cidx];
    auto& points = allPoints[cidx];
    int i = 0;
    for (double t : params) {
      PK_VECTOR_t p[1];
      PK_CURVE_eval(c, t, 0, p);
      points[i++].P.SetCoord(p[0].coord[0], p[0].coord[1], p[0].coord[2]);
    }
  }
}

void evaluateCurvesDegree1(const std::vector<PK_CURVE_t>& curves,
                           const std::vector<std::vector<double>>& allParams,
                           std::vector<std::vector<CurveResult>>& allPoints)
{
  for (size_t cidx = 0; cidx < curves.size(); ++cidx) {
    PK_CURVE_t c = curves[cidx];
    const auto& params = allParams[cidx];
    auto& points = allPoints[cidx];
    int i = 0;
    for (double t : params) {
      PK_VECTOR_t p[2];
      PK_CURVE_eval(c, t, 1, p);
      CurveResult& r = points[i++];
      r.P.SetCoord(p[0].coord[0], p[0].coord[1], p[0].coord[2]);
      r.D1.SetCoord(p[1].coord[0], p[1].coord[1], p[1].coord[2]);
    }
  }
}

void evaluateCurvesDegree2(const std::vector<PK_CURVE_t>& curves,
                           const std::vector<std::vector<double>>& allParams,
                           std::vector<std::vector<CurveResult>>& allPoints)
{
  for (size_t cidx = 0; cidx < curves.size(); ++cidx) {
    PK_CURVE_t c = curves[cidx];
    const auto& params = allParams[cidx];
    auto& points = allPoints[cidx];
    int i = 0;
    for (double t : params) {
      PK_VECTOR_t p[3];
      PK_CURVE_eval(c, t, 2, p);
      CurveResult& r = points[i++];
      r.P.SetCoord(p[0].coord[0], p[0].coord[1], p[0].coord[2]);
      r.D1.SetCoord(p[1].coord[0], p[1].coord[1], p[1].coord[2]);
      r.D2.SetCoord(p[2].coord[0], p[2].coord[1], p[2].coord[2]);
    }
  }
}

// ---- surface evaluation ----
// Rectangular layout: index of (i-th u-deriv, j-th v-deriv) = i + (n_u_derivs+1)*j.
void evaluateSurfacesDegree0(const std::vector<PK_SURF_t>& surfaces,
                             const std::vector<std::vector<std::pair<double, double>>>& allUVPoints,
                             std::vector<std::vector<SurfaceResult>>& allPoints)
{
  for (size_t sidx = 0; sidx < surfaces.size(); ++sidx) {
    PK_SURF_t s = surfaces[sidx];
    const auto& uvPoints = allUVPoints[sidx];
    auto& points = allPoints[sidx];
    int i = 0;
    for (auto& [u, v] : uvPoints) {
      PK_UV_t uv; uv.param[0] = u; uv.param[1] = v;
      PK_VECTOR_t p[1];
      PK_SURF_eval(s, uv, 0, 0, PK_LOGICAL_false, p);
      points[i++].P.SetCoord(p[0].coord[0], p[0].coord[1], p[0].coord[2]);
    }
  }
}

void evaluateSurfacesDegree1(const std::vector<PK_SURF_t>& surfaces,
                             const std::vector<std::vector<std::pair<double, double>>>& allUVPoints,
                             std::vector<std::vector<SurfaceResult>>& allPoints)
{
  for (size_t sidx = 0; sidx < surfaces.size(); ++sidx) {
    PK_SURF_t s = surfaces[sidx];
    const auto& uvPoints = allUVPoints[sidx];
    auto& points = allPoints[sidx];
    int i = 0;
    for (auto& [u, v] : uvPoints) {
      PK_UV_t uv; uv.param[0] = u; uv.param[1] = v;
      PK_VECTOR_t p[4];   // (1+1)*(1+1)
      PK_SURF_eval(s, uv, 1, 1, PK_LOGICAL_false, p);
      SurfaceResult& r = points[i++];
      r.P.SetCoord(p[0].coord[0], p[0].coord[1], p[0].coord[2]);
      r.D1u.SetCoord(p[1].coord[0], p[1].coord[1], p[1].coord[2]);   // i=1,j=0
      r.D1v.SetCoord(p[2].coord[0], p[2].coord[1], p[2].coord[2]);   // i=0,j=1
    }
  }
}

void evaluateSurfacesDegree2(const std::vector<PK_SURF_t>& surfaces,
                             const std::vector<std::vector<std::pair<double, double>>>& allUVPoints,
                             std::vector<std::vector<SurfaceResult>>& allPoints)
{
  for (size_t sidx = 0; sidx < surfaces.size(); ++sidx) {
    PK_SURF_t s = surfaces[sidx];
    const auto& uvPoints = allUVPoints[sidx];
    auto& points = allPoints[sidx];
    int i = 0;
    for (auto& [u, v] : uvPoints) {
      PK_UV_t uv; uv.param[0] = u; uv.param[1] = v;
      PK_VECTOR_t p[9];   // (2+1)*(2+1)
      PK_SURF_eval(s, uv, 2, 2, PK_LOGICAL_false, p);
      SurfaceResult& r = points[i++];
      r.P.SetCoord(p[0].coord[0], p[0].coord[1], p[0].coord[2]);     // point
      r.D1u.SetCoord(p[1].coord[0], p[1].coord[1], p[1].coord[2]);   // i=1,j=0
      r.D2uu.SetCoord(p[2].coord[0], p[2].coord[1], p[2].coord[2]);  // i=2,j=0
      r.D1v.SetCoord(p[3].coord[0], p[3].coord[1], p[3].coord[2]);   // i=0,j=1
      r.D2uv.SetCoord(p[4].coord[0], p[4].coord[1], p[4].coord[2]);  // i=1,j=1
      r.D2vv.SetCoord(p[6].coord[0], p[6].coord[1], p[6].coord[2]);  // i=0,j=2
    }
  }
}

#endif /* HAS_PARASOLID */

/**
 * Dump surface evaluation results
 */
void dumpSurfaceResults(const std::vector<BRepAdaptor_Surface>& surfaces,
                        const std::vector<std::vector<std::pair<double, double>>>& allUVPoints,
                        const std::vector<std::vector<SurfaceResult>>& allPoints, int derivDegree)
{
  std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);
  for (size_t sidx = 0; sidx < surfaces.size(); ++sidx) {
    const auto& surf = surfaces[sidx];
    const auto& uvPoints = allUVPoints[sidx];
    const auto& points = allPoints[sidx];

    if (surf.GetType() == GeomAbs_BSplineSurface)
    {
      const auto& spline = surf.BSpline();
      std::cout << "Surface " << (sidx + 1) << " degreeU " << spline->UDegree() << " degreeV " << spline->VDegree();
      std::cout << " nbknotsU " << spline->NbUKnots() << " nbknotsV " << spline->NbVKnots();
      std::cout << " rationalU " << spline->IsURational() << " rationalV " << spline->IsVRational() << std::endl;
    }
    else
    {
      static const std::array<const char*, 11> surfTypeNames = {
        "Plane", "Cylinder", "Cone", "Sphere", "Torus", "BezierSurface", "BSplineSurface", "SurfaceOfRevolution", "SurfaceOfExtrusion", "OffsetSurface", "OtherSurface"
      };
      std::cout << "Surface " << (sidx + 1) << " is " << surfTypeNames[surf.GetType() - GeomAbs_Plane] << std::endl;
    }

    if (derivDegree < 0) {
      continue;
    }

    int i = 0;
    for (auto [u, v] : uvPoints) {
      std::cout << " u " << u << " v " << v;

      const SurfaceResult& r = points[i++];
      const gp_Pnt& P = r.P;
      const gp_Vec& D1u = r.D1u, &D1v = r.D1v;
      const gp_Vec& D2uu = r.D2uu, &D2vv = r.D2vv, &D2uv = r.D2uv;

      std::cout << " P (" << P.X() << "," << P.Y() << "," << P.Z() << ")";
      if (derivDegree > 0) {
        std::cout << " D1u (" << D1u.X() << "," << D1u.Y() << "," << D1u.Z() << ")";
        std::cout << " D1v (" << D1v.X() << "," << D1v.Y() << "," << D1v.Z() << ")";
      }
      if (derivDegree > 1) {
        std::cout << " D2uu (" << D2uu.X() << "," << D2uu.Y() << "," << D2uu.Z() << ")";
        std::cout << " D2vv (" << D2vv.X() << "," << D2vv.Y() << "," << D2vv.Z() << ")";
        std::cout << " D2uv (" << D2uv.X() << "," << D2uv.Y() << "," << D2uv.Z() << ")";
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }
}

/**
 * Benchmark NURBS curves
 */
PerfResult benchmarkCurves(const TopoDS_Shape& shape,
                           const std::vector<int>& edgeIndices,
                           int numPoints,
                           int numRepeats,
                           bool randomMode,
                           int derivDegree,
                           bool recordOutput,
                           Library lib)
{
  // Collect all edges from the shape
  std::vector<TopoDS_Edge> edges;
  TopExp_Explorer exp(shape, TopAbs_EDGE);
  for (; exp.More(); exp.Next()) {
    edges.push_back(TopoDS::Edge(exp.Current()));
  }

  if (edges.empty()) {
    throw std::runtime_error("No edges found in shape");
  }

  // Extract adaptors for selected edges
  std::vector<BRepAdaptor_Curve> curves;
  for (int idx : edgeIndices) {
    if (idx <= 0 || idx > (int)edges.size()) {
      throw std::runtime_error("Edge index " + std::to_string(idx) +
                               " out of range (1-" + std::to_string(edges.size()) + ")");
    }
    curves.emplace_back(edges[idx - 1]);
  }

  // Pre-generate parameters and prepare buffer for results for all curves (outside timing)
  std::vector<std::vector<double>> allParams;
  allParams.reserve(curves.size());
  std::vector<std::vector<CurveResult>> allPoints;
  allPoints.reserve(curves.size());

  for (const auto& curve : curves) {
    double tmin = curve.FirstParameter();
    double tmax = curve.LastParameter();

    if (randomMode) {
      allParams.push_back(generateRandomParams(numPoints, tmin, tmax));
    }
    else {
      allParams.push_back(generateSequentialParams(numPoints, tmin, tmax));
    }

    allPoints.emplace_back(allParams.back().size());
  }

  // Select and execute the appropriate evaluation function based on selected library, mode, and degree
  PerfResult result{};
  if (lib == Library::OCCT)
  {
    switch (derivDegree) {
    case 0:
      result = measureTime([&]() {evaluateCurvesDegree0(curves, allParams, allPoints); }, numRepeats);
      break;
    case 1:
      result = measureTime([&]() {evaluateCurvesDegree1(curves, allParams, allPoints); }, numRepeats);
      break;
    case 2:
      result = measureTime([&]() {evaluateCurvesDegree2(curves, allParams, allPoints); }, numRepeats);
      break;
    }
  }
  else if (lib == Library::ACIS)
  {
#ifdef HAS_ACIS
    std::vector<UniqueACISCurve> curvesAcis;
    curvesAcis.reserve(curves.size());
    for (size_t i = 0; i < curves.size(); ++i) {
      curvesAcis.emplace_back(std::move(ConvertToACIS(curves[i].BSpline())));
    }

    switch (derivDegree) {
    case 0:
      result = measureTime([&]() {evaluateCurvesDegree0(curvesAcis, allParams, allPoints); }, numRepeats);
      break;
    case 1:
      result = measureTime([&]() {evaluateCurvesDegree1(curvesAcis, allParams, allPoints); }, numRepeats);
      break;
    case 2:
      result = measureTime([&]() {evaluateCurvesDegree2(curvesAcis, allParams, allPoints); }, numRepeats);
      break;
    }
#else
    throw std::runtime_error("The executable is built without ACIS support");
#endif
  }
  else if (lib == Library::Parasolid)
  {
#ifdef HAS_PARASOLID
    std::vector<PK_CURVE_t> curvesPs;
    curvesPs.reserve(curves.size());
    for (size_t i = 0; i < curves.size(); ++i) {
      curvesPs.push_back(ConvertToParasolid(curves[i].BSpline()));
    }

    switch (derivDegree) {
    case 0:
      result = measureTime([&]() {evaluateCurvesDegree0(curvesPs, allParams, allPoints); }, numRepeats);
      break;
    case 1:
      result = measureTime([&]() {evaluateCurvesDegree1(curvesPs, allParams, allPoints); }, numRepeats);
      break;
    case 2:
      result = measureTime([&]() {evaluateCurvesDegree2(curvesPs, allParams, allPoints); }, numRepeats);
      break;
    }
#else
    throw std::runtime_error("The executable is built without Parasolid support");
#endif
  }
  else
  {
    throw std::runtime_error("The selected library does not support curve evaluation");
  }

  // Output either summary of curve propeties or all points evaluated
  dumpCurveResults(curves, allParams, allPoints, recordOutput ? derivDegree : -1);

  // Return counters
  result.nbPoints = curves.size() * numPoints * numRepeats;
  return result;
}

/**
 * Check distance between corner point of the BSpline surface and corresponding pole,
 * in the case if both U and V multiplicities at that corner are equal to degree + 1.
 * This allows to check accuracy of evaluation against ground truth.
 */
void checkCorners(const BRepAdaptor_Surface& surf,
                  const std::vector<std::pair<double, double>>& uvParams,
                  const std::vector<SurfaceResult>& results)
{
  Handle(Geom_BSplineSurface) bspl = surf.BSpline();
  if (!bspl)
  {
    return; // nothing to check
  }

  // iterate by corners
  for (bool takeMinU : {true, false}) {
    for (bool takeMinV : {true, false}) {
      if (bspl->UMultiplicity(takeMinU ? 1 : bspl->NbUKnots()) != bspl->UDegree() + 1 ||
          bspl->VMultiplicity(takeMinV ? 1 : bspl->NbVKnots()) != bspl->VDegree() + 1)
      {
        continue; // surface is not clamped at that corner (does not pass through the pole)
      }

      double uCorner = bspl->UKnot(takeMinU ? 1 : bspl->NbUKnots());
      double vCorner = bspl->VKnot(takeMinV ? 1 : bspl->NbVKnots());

      for (size_t i = 0; i < uvParams.size(); ++i) {
        double u = uvParams[i].first;
        double v = uvParams[i].second;

        // note: here we check for exact equality because this code is intended to check
        // the points evaluated exactly at corners
        if (u == uCorner && v == vCorner) {
          gp_Pnt pole = bspl->Pole(takeMinU ? 1 : bspl->NbUPoles(), takeMinV ? 1 : bspl->NbVPoles());
          Standard_Real dist = pole.Distance(results[i].P);
          std::cout << "Corner [" << (takeMinU ? 0 : 1) << ", " << (takeMinV ? 0 : 1) <<
            "] distance to pole = " << dist << std::endl;
        }
      }
    }
  }
}

/**
 * Benchmark NURBS surfaces
 */
PerfResult benchmarkSurfaces(const TopoDS_Shape& shape,
                             const std::vector<int>& faceIndices,
                             int numPoints,
                             int numRepeats,
                             bool randomMode,
                             int derivDegree,
                             bool recordOutput,
                             Library lib)
{
  // Collect all faces from the shape
  std::vector<TopoDS_Face> faces;
  TopExp_Explorer exp(shape, TopAbs_FACE);
  for (; exp.More(); exp.Next()) {
    faces.push_back(TopoDS::Face(exp.Current()));
  }

  if (faces.empty()) {
    throw std::runtime_error("No faces found in shape");
  }

  // Extract adaptors for selected faces
  std::vector<BRepAdaptor_Surface> surfaces;
  for (int idx : faceIndices) {
    if (idx <= 0 || idx > (int)faces.size()) {
      throw std::runtime_error("Face index " + std::to_string(idx) +
                               " out of range (1-" + std::to_string(faces.size()) + ")");
    }
    surfaces.emplace_back(faces[idx - 1], false);
  }

  // Pre-generate UV parameters and prepare buffer for results for all surfaces (outside timing)
  std::vector<std::vector<std::pair<double, double>>> allUVPoints;
  allUVPoints.reserve(surfaces.size());
  std::vector<std::vector<SurfaceResult>> allPoints;
  allPoints.reserve(surfaces.size());

  for (const auto& surf : surfaces) {
    double umin = surf.FirstUParameter();
    double umax = surf.LastUParameter();
    double vmin = surf.FirstVParameter();
    double vmax = surf.LastVParameter();

    if (randomMode) {
      allUVPoints.push_back(generateRandomUVParams(numPoints, umin, umax, vmin, vmax));
    }
    else {
      allUVPoints.push_back(generateSequentialUVParams(numPoints, umin, umax, vmin, vmax));
    }

    allPoints.emplace_back(allUVPoints.back().size());
  }

  // Select and execute the appropriate evaluation function based on selected library, mode, and degree
  PerfResult result{};
  if (lib == Library::OCCT)
  {
    switch (derivDegree) {
    case 0:
      result = measureTime([&]() {evaluateSurfacesDegree0(surfaces, allUVPoints, allPoints); }, numRepeats);
      break;
    case 1:
      result = measureTime([&]() {evaluateSurfacesDegree1(surfaces, allUVPoints, allPoints); }, numRepeats);
      break;
    case 2:
      result = measureTime([&]() {evaluateSurfacesDegree2(surfaces, allUVPoints, allPoints); }, numRepeats);
      break;
    }
  }
  else if (lib == Library::TinyNurbs)
  {
#ifdef HAS_TINYNURBS
    std::vector<tinynurbs::Surface<double>> surfacesTn;
    for (int i = 0; i < surfaces.size(); ++i) {
      surfacesTn.push_back(ConvertToTinyNurbs(surfaces[i].BSpline()));
    }

    switch (derivDegree) {
    case 0:
      result = measureTime([&]() {evaluateSurfacesDegree0(surfacesTn, allUVPoints, allPoints); }, numRepeats);
      break;
    case 1:
      result = measureTime([&]() {evaluateSurfacesDegree1(surfacesTn, allUVPoints, allPoints); }, numRepeats);
      break;
    case 2:
      result = measureTime([&]() {evaluateSurfacesDegree2(surfacesTn, allUVPoints, allPoints); }, numRepeats);
      break;
    }
#else
    throw std::runtime_error("The executable is built without TinyNurbs support");
#endif
  }
  else if (lib == Library::SISL)
  {
#ifdef HAS_SISL
    std::vector<UniqueSISLSurf> surfacesSISL;
    for (int i = 0; i < surfaces.size(); ++i) {
      surfacesSISL.emplace_back(std::move(ConvertToSISL(surfaces[i].BSpline())));
    }

    switch (derivDegree) {
    case 0:
      result = measureTime([&]() {evaluateSurfacesDegree0(surfacesSISL, allUVPoints, allPoints); }, numRepeats);
      break;
    case 1:
      result = measureTime([&]() {evaluateSurfacesDegree1(surfacesSISL, allUVPoints, allPoints); }, numRepeats);
      break;
    case 2:
      result = measureTime([&]() {evaluateSurfacesDegree2(surfacesSISL, allUVPoints, allPoints); }, numRepeats);
      break;
    }
#else
    throw std::runtime_error("The executable is built without SISL support");
#endif
  }
  else if (lib == Library::ACIS)
  {
#ifdef HAS_ACIS
    std::vector<UniqueACISSurf> surfacesAcis;
    surfacesAcis.reserve(surfaces.size());
    for (size_t i = 0; i < surfaces.size(); ++i) {
      surfacesAcis.emplace_back(std::move(ConvertToACIS(surfaces[i].BSpline())));
    }

    switch (derivDegree) {
    case 0:
      result = measureTime([&]() {evaluateSurfacesDegree0(surfacesAcis, allUVPoints, allPoints); }, numRepeats);
      break;
    case 1:
      result = measureTime([&]() {evaluateSurfacesDegree1(surfacesAcis, allUVPoints, allPoints); }, numRepeats);
      break;
    case 2:
      result = measureTime([&]() {evaluateSurfacesDegree2(surfacesAcis, allUVPoints, allPoints); }, numRepeats);
      break;
    }
#else
    throw std::runtime_error("The executable is built without ACIS support");
#endif
  }
  else if (lib == Library::Parasolid)
  {
#ifdef HAS_PARASOLID
    std::vector<PK_SURF_t> surfacesPs;
    surfacesPs.reserve(surfaces.size());
    for (size_t i = 0; i < surfaces.size(); ++i) {
      surfacesPs.push_back(ConvertToParasolid(surfaces[i].BSpline()));
    }

    switch (derivDegree) {
    case 0:
      result = measureTime([&]() {evaluateSurfacesDegree0(surfacesPs, allUVPoints, allPoints); }, numRepeats);
      break;
    case 1:
      result = measureTime([&]() {evaluateSurfacesDegree1(surfacesPs, allUVPoints, allPoints); }, numRepeats);
      break;
    case 2:
      result = measureTime([&]() {evaluateSurfacesDegree2(surfacesPs, allUVPoints, allPoints); }, numRepeats);
      break;
    }
#else
    throw std::runtime_error("The executable is built without Parasolid support");
#endif
  }

  // For special case of 4 sequential points, which may be located exactly at
  // the corners of the surface (if face is built on natural range of the surface),
  // check them against ground-truth (corner poles), only for clamped surfaces
  if (numPoints == 4 && !randomMode)
  {
    for (size_t i = 0; i < surfaces.size(); ++i) {
      checkCorners(surfaces[i], allUVPoints[i], allPoints[i]);
    }
  }

  // Output either summary of surface propeties or all points evaluated
  dumpSurfaceResults(surfaces, allUVPoints, allPoints, recordOutput ? derivDegree : -1);

  // Return performance counters
  result.nbPoints = surfaces.size() * numPoints * numRepeats;
  return result;
}

/**
 * Tune system to get more stable measurements
 */
void StabilizeForMinMeasurement()
{
  // 1. Force CPU to maximum frequency (prevents throttling giving false lows)
  // Run a busy loop first to exit low-power states
  auto start = std::chrono::steady_clock::now();
  while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < 0.1) {
    _mm_pause();
  }


#ifdef _WIN32

  // 2. Align to core 0 for most consistent frequency scaling
//  SetThreadAffinityMask(GetCurrentThread(), 1);

  // 3. Prevent Windows from moving our thread while measuring
//  SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);

  // 4. Disable Turbo Boost? (Optional - more consistent but slower)
  // Use powercfg to set max frequency to 99% disabling turbo

  // Disable Core Parking (run once before measurements)
//  system("powercfg -setactive 8c5e7fda-e8bf-4a96-9a85-a6e23a8c635c"); // High performance

  // Force Windows to complete background tasks
  Sleep(1000);

  // Clear working set to avoid page faults during measurement
  SetProcessWorkingSetSize(GetCurrentProcess(), -1, -1);

  // Disable Windows Update, Defender, etc. (manual steps recommended)

#endif
}

/**
 * Helper function generating string representing current time in ISO format
 */
static std::string GetCurrentTimeString()
{
  auto now = std::chrono::system_clock::now();
  std::time_t time = std::chrono::system_clock::to_time_t(now);

  std::stringstream ss;
  ss << std::put_time(std::localtime(&time), "%FT%T");
  return ss.str();
};

/**
 * Probe mode: report the geometry of every subshape of the requested kind,
 * without performing any evaluation.
 *
 * Subshapes are deduplicated (orientation-normalized) so that a shared edge or
 * face is reported once per model. For BSpline geometry the intrinsic
 * properties are printed; for other geometry only the type name is printed.
 * Output lines are prefixed with "PROBE" for easy machine parsing:
 *   PROBE C <index> BSpline <degree> <rational 0/1> <nbKnots>
 *   PROBE C <index> <CurveTypeName>
 *   PROBE S <index> BSpline <uDeg> <vDeg> <ratU 0/1> <ratV 0/1> <nbUKnots> <nbVKnots>
 *   PROBE S <index> <SurfaceTypeName>
 */
static void probeShape(const TopoDS_Shape& shape, bool curves)
{
  TopTools_IndexedMapOfShape subshapes;
  const TopAbs_ShapeEnum kind = curves ? TopAbs_EDGE : TopAbs_FACE;
  for (TopExp_Explorer exp(shape, kind); exp.More(); exp.Next()) {
    TopoDS_Shape s = exp.Current();
    s.Orientation(TopAbs_FORWARD); // normalize so shared subshapes are counted once
    subshapes.Add(s);
  }

  static const std::array<const char*, 9> curveTypeNames = {
    "Line", "Circle", "Ellipse", "Hyperbola", "Parabola", "BezierCurve", "BSplineCurve", "OffsetCurve", "OtherCurve"
  };
  static const std::array<const char*, 11> surfTypeNames = {
    "Plane", "Cylinder", "Cone", "Sphere", "Torus", "BezierSurface", "BSplineSurface", "SurfaceOfRevolution", "SurfaceOfExtrusion", "OffsetSurface", "OtherSurface"
  };

  for (int i = 1; i <= subshapes.Extent(); ++i) {
    const TopoDS_Shape& s = subshapes(i);
    if (curves) {
      BRepAdaptor_Curve c(TopoDS::Edge(s));
      const GeomAbs_CurveType t = c.GetType();
      if (t == GeomAbs_BSplineCurve) {
        const Handle(Geom_BSplineCurve) b = c.BSpline();
        std::cout << "PROBE C " << i << " BSpline " << b->Degree() << " "
                  << (b->IsRational() ? 1 : 0) << " " << b->NbKnots() << "\n";
      }
      else {
        std::cout << "PROBE C " << i << " " << curveTypeNames[t - GeomAbs_Line] << "\n";
      }
    }
    else {
      BRepAdaptor_Surface sf(TopoDS::Face(s), false);
      const GeomAbs_SurfaceType t = sf.GetType();
      if (t == GeomAbs_BSplineSurface) {
        const Handle(Geom_BSplineSurface) b = sf.BSpline();
        std::cout << "PROBE S " << i << " BSpline " << b->UDegree() << " " << b->VDegree() << " "
                  << (b->IsURational() ? 1 : 0) << " " << (b->IsVRational() ? 1 : 0) << " "
                  << b->NbUKnots() << " " << b->NbVKnots() << "\n";
      }
      else {
        std::cout << "PROBE S " << i << " " << surfTypeNames[t - GeomAbs_Plane] << "\n";
      }
    }
  }
}

/**
 * Main entry point
 */
int main(int argc, char* argv[])
{
  // Parse command line arguments
  if (argc != 9 && argc != 10) {
    printUsage(argv[0]);
    return 1;
  }

  StabilizeForMinMeasurement();

  try {
    std::string filePath = argv[1];
    std::string typeStr = argv[2];
    std::string indicesStr = argv[3];
    int numPoints = std::stoi(argv[4]);
    int numRepeats = std::stoi(argv[5]);
    std::string modeStr = argv[6];
    int derivDegree = std::stoi(argv[7]);
    std::string outputStr = argv[8];

    std::string libStr = (argc > 9 ? argv[9] : "occt");
    Library lib = Library::OCCT;
    if (argc > 9) {
      if (libStr == "tinynurbs") {
        lib = Library::TinyNurbs;
      }
      else if (libStr == "sisl") {
        lib = Library::SISL;
      }
      else if (libStr == "acis") {
        lib = Library::ACIS;
      }
      else if (libStr == "parasolid") {
        lib = Library::Parasolid;
      }
      else if (libStr != "occt") {
        throw std::runtime_error("Unsupported library type: '" + libStr + "'");
      }
    }

    // Validate arguments
    if (numPoints < 0) {
      throw std::runtime_error("Number of points must be non-negative");
    }

    if (derivDegree < 0 || derivDegree > 2) {
      throw std::runtime_error("Derivative degree must be 0, 1, or 2");
    }

    bool isCurves = false;
    if (typeStr == "curves") {
      isCurves = true;
    }
    else if (typeStr == "surfaces") {
      isCurves = false;
    }
    else {
      throw std::runtime_error("Type must be 'curves' or 'surfaces'");
    }

    bool isRandom = false;
    if (modeStr == "random") {
      isRandom = true;
    }
    else if (modeStr == "sequential") {
      isRandom = false;
    }
    else {
      throw std::runtime_error("Mode must be 'random' or 'sequential'");
    }

    bool recordOutput = false;
    bool probeMode = false;
    if (outputStr == "record") {
      recordOutput = true;
    }
    else if (outputStr == "perf") {
    }
    else if (outputStr == "probe") {
      probeMode = true;
    }
    else {
      throw std::runtime_error("Output mode must be 'perf', 'record' or 'probe'");
    }

    // Parse indices (probe mode scans all subshapes, so indices are ignored there)
    std::vector<int> indices;
    const bool allShapes = (indicesStr == "*");
    if (!probeMode) {
      if (allShapes) {
        throw std::runtime_error("Index '*' (all subshapes) is only supported in 'probe' output mode");
      }
      indices = parseIndices(indicesStr);
      if (indices.empty()) {
        throw std::runtime_error("No indices provided");
      }
    }

    // Print timestamp
    std::cout << GetCurrentTimeString() << std::endl;

    // Load shape
    std::cout << "Loading shape from: " << filePath << std::endl;
    TopoDS_Shape shape = loadShape(filePath);
    std::cout << "Shape loaded successfully" << std::endl;

    // Probe mode: report geometry of every subshape and exit (no evaluation)
    if (probeMode) {
      probeShape(shape, isCurves);
      return 0;
    }

    // Run benchmark
    PerfResult result;
    if (isCurves) {
      std::cout << "Benchmarking curves (edges: ";
      for (size_t i = 0; i < indices.size(); ++i) {
        if (i > 0) std::cout << ",";
        std::cout << indices[i];
      }
      std::cout << ")" << std::endl << std::endl;
      result = benchmarkCurves(shape, indices, numPoints, numRepeats, isRandom, derivDegree, recordOutput, lib);
    }
    else {
      std::cout << "Benchmarking surfaces (faces: ";
      for (size_t i = 0; i < indices.size(); ++i) {
        if (i > 0) std::cout << ",";
        std::cout << indices[i];
      }
      std::cout << ")" << std::endl << std::endl;
      result = benchmarkSurfaces(shape, indices, numPoints, numRepeats, isRandom, derivDegree, recordOutput, lib);
    }

    // Output performance summary
    if (numPoints > 0)
    {
      std::cout << "\n=== Performance Summary ===\n";
      std::cout << "Total wall clock time:  " << result.elapsedTime << " seconds\n";
      std::cout << "Total CPU time:         " << result.cpuTime << " seconds\n";
      std::cout << "QueryProcessCycleTime:  " << result.cpuTime2 << " sec\n";
      std::cout << "TSC:                    " << result.totalTsc / 1000000 << " M\n";
      std::cout << "Points evaluated:       " << result.nbPoints << "\n";
      if (result.elapsedTime > 0) {
        std::cout << "Evaluations per second: " << (size_t)(result.nbPoints / result.elapsedTime) << "\n";
      }

      // Output one-line information on a call and results to cerr for statistics
      std::cerr << GetCurrentTimeString() << " ";
      for (int i = 1; i < argc; i++) {
        std::cerr << argv[i] << " ";
      }
      std::cerr << "NbPoints " << result.nbPoints 
        << " Elapsed " << result.elapsedTime << " CPU " << result.cpuTime
        << " QueryProcessCycleTime " << result.cpuTime2 << " TSC " << result.totalTsc / 1000000
        << std::endl;
    }
  }
  catch (const std::exception& e) {
    std::cerr << GetCurrentTimeString() << " ";
    for (int i = 1; i < argc; i++) {
      std::cerr << argv[i] << " ";
    }
    std::cerr << "Error: " << e.what() << std::endl;
    if (std::cout.rdbuf() != std::cerr.rdbuf()) {
      std::cout << "Error: " << e.what() << std::endl;
    }
    return 1;
  }

  return 0;
}
