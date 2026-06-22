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

#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <STEPControl_Reader.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>

#ifdef _WIN32
#include <windows.h>
#endif

 /**
  * Print usage information
  */
void printUsage(const char* progName)
{
  std::cout << "Usage: " << progName << " <file> <curves|surfaces> <indices> <numPoints> <numRepeats> <random|sequential> <derivDegree(0-2)> <perf|record>\n";
  std::cout << "  <file>        - Path to .brep or .step file\n";
  std::cout << "  <curves|surfaces> - Choose to evaluate curves or surfaces\n";
  std::cout << "  <indices>     - Comma-separated list of edge (for curves) or face (for surfaces) indices (1-based),\n";
  std::cout << "                  can include ranges like 3-7\n";
  std::cout << "  <numPoints>   - Number of points to evaluate\n";
  std::cout << "  <numRepeats>  - Number of times to repeat evaluation (for greater times)\n";
  std::cout << "  <random|sequential> - Parameter distribution mode\n";
  std::cout << "  <derivDegree> - Derivative degree (0, 1, or 2)\n";
  std::cout << "  <perf|record> - Output mode: performance measurement or result recording\n";
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
 * Measure both wall clock time and CPU time of a function execution
 * Returns pair<wall_time_seconds, cpu_time_seconds>
 */
template <typename Func>
auto measureTime(Func&& func, int numRepeats)
{
  auto start_cpu = std::clock();
  auto start_wall = std::chrono::high_resolution_clock::now();

  for (int i = 0; i < numRepeats; i++) {
    func();
  }

  auto end_wall = std::chrono::high_resolution_clock::now();
  auto end_cpu = std::clock();

  double wall_time = std::chrono::duration<double>(end_wall - start_wall).count();
  double cpu_time = static_cast<double>(end_cpu - start_cpu) / CLOCKS_PER_SEC;

  return std::make_pair(wall_time, cpu_time);
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
      const gp_Pnt& P = points[i].P;
      const gp_Vec& D1u = points[i].D1u, D1v = points[i].D1v;
      const gp_Vec& D2uu = points[i].D2uu, D2vv = points[i].D2vv, D2uv = points[i].D2uv;

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
 * Performance measurement results
 */
struct PerfResult
{
  size_t nbPoints = 0;
  double elapsedTime = 0;
  double cpuTime = 0;
};

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
  std::pair<double, double> times = {};
  switch (derivDegree) {
  case 0:
    times = measureTime([&]() {evaluateCurvesDegree0(curves, allParams, allPoints); }, numRepeats);
    break;
  case 1:
    times = measureTime([&]() {evaluateCurvesDegree1(curves, allParams, allPoints); }, numRepeats);
    break;
  case 2:
    times = measureTime([&]() {evaluateCurvesDegree2(curves, allParams, allPoints); }, numRepeats);
    break;
  }
  double total_wall = times.first;
  double total_cpu = times.second;

  // Output either summary of curve propeties or all points evaluated
  dumpCurveResults(curves, allParams, allPoints, recordOutput ? derivDegree : -1);

  // Return performance counters
  return { curves.size() * numPoints * numRepeats, times.first, times.second };
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
                             bool recordOutput)
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

  // Select and execute the appropriate evaluation function based on mode and degree
  std::pair<double, double> times = {};
  switch (derivDegree) {
  case 0:
    times = measureTime([&]() {evaluateSurfacesDegree0(surfaces, allUVPoints, allPoints); }, numRepeats);
    break;
  case 1:
    times = measureTime([&]() {evaluateSurfacesDegree1(surfaces, allUVPoints, allPoints); }, numRepeats);
    break;
  case 2:
    times = measureTime([&]() {evaluateSurfacesDegree2(surfaces, allUVPoints, allPoints); }, numRepeats);
    break;
  }
  double total_wall = times.first;
  double total_cpu = times.second;

  // Output either summary of surface propeties or all points evaluated
  dumpSurfaceResults(surfaces, allUVPoints, allPoints, recordOutput ? derivDegree : -1);

  // Return performance counters
  return { surfaces.size() * numPoints * numRepeats, times.first, times.second };
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
  SetThreadAffinityMask(GetCurrentThread(), 1);

  // 3. Prevent Windows from moving our thread while measuring
  SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);

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
 * Main entry point
 */
int main(int argc, char* argv[])
{
  // Parse command line arguments
  if (argc != 9) {
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

    // Validate arguments
    if (numPoints <= 0) {
      throw std::runtime_error("Number of points must be positive");
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
      result = benchmarkSurfaces(shape, indices, numPoints, numRepeats, isRandom, derivDegree, recordOutput);
    }

    // Output performance summary
    std::cout << "\n=== Performance Summary ===\n";
    std::cout << "Total wall clock time: " << result.elapsedTime << " seconds\n";
    std::cout << "Total CPU time:         " << result.cpuTime << " seconds\n";
    std::cout << "Points evaluated:       " << result.nbPoints << "\n";
    if (result.elapsedTime > 0) {
      std::cout << "Evaluations per second: " << (size_t)(result.nbPoints / result.elapsedTime) << "\n";
    }

    // Output one-line information on a call and results to cerr for statistics
    for (int i = 0; i < argc; i++) {
      std::cerr << argv[i] << " ";
    }
    std::cerr << "NbPoints " << result.nbPoints << " Elapsed " << result.elapsedTime << " CPU " << result.cpuTime << std::endl;
  }
  catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
