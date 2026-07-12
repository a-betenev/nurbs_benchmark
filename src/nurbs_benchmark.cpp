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
#include <TColgp_Array2OfPnt.hxx>
#include <TColStd_Array2OfReal.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>

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
  std::cout << "Usage: " << progName << " <file> <curves|surfaces> <indices> <numPoints> <numRepeats> <random|sequential> <derivDegree(0-2)> <perf|record> [lib]\n";
  std::cout << "  <file>        - Path to .brep or .step file\n";
  std::cout << "  <curves|surfaces> - Choose to evaluate curves or surfaces\n";
  std::cout << "  <indices>     - Comma-separated list of edge (for curves) or face (for surfaces) indices (1-based),\n";
  std::cout << "                  can include ranges like 3-7\n";
  std::cout << "  <numPoints>   - Number of points to evaluate\n";
  std::cout << "  <numRepeats>  - Number of times to repeat evaluation (for greater times)\n";
  std::cout << "  <random|sequential> - Parameter distribution mode\n";
  std::cout << "  <derivDegree> - Derivative degree (0, 1, or 2)\n";
  std::cout << "  <perf|record> - Output mode: performance measurement or result recording\n";
  std::cout << "  lib           - Librray to be used: occt (default) or tinynurbs\n";
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
  double total_cpu2 = GetProcessCPU();

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
  std::cout << std::setprecision(15);
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

/**
 * Dump surface evaluation results
 */
void dumpSurfaceResults(const std::vector<BRepAdaptor_Surface>& surfaces,
                        const std::vector<std::vector<std::pair<double, double>>>& allUVPoints,
                        const std::vector<std::vector<SurfaceResult>>& allPoints, int derivDegree)
{
  std::cout << std::setprecision(15);
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
                           bool recordOutput)
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

  // Select and execute the appropriate evaluation function based on mode and degree
  PerfResult result{};
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
    if (outputStr == "record") {
      recordOutput = true;
    }
    else if (outputStr == "perf") {
      recordOutput = false;
    }
    else {
      throw std::runtime_error("Output mode must be 'perf' or 'record'");
    }

    // Parse indices
    std::vector<int> indices = parseIndices(indicesStr);
    if (indices.empty()) {
      throw std::runtime_error("No indices provided");
    }

    // Print timestamp
    std::cout << GetCurrentTimeString() << std::endl;

    // Load shape
    std::cout << "Loading shape from: " << filePath << std::endl;
    TopoDS_Shape shape = loadShape(filePath);
    std::cout << "Shape loaded successfully" << std::endl;

    // Run benchmark
    PerfResult result;
    if (isCurves) {
      std::cout << "Benchmarking curves (edges: ";
      for (size_t i = 0; i < indices.size(); ++i) {
        if (i > 0) std::cout << ",";
        std::cout << indices[i];
      }
      std::cout << ")" << std::endl << std::endl;
      result = benchmarkCurves(shape, indices, numPoints, numRepeats, isRandom, derivDegree, recordOutput);
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
