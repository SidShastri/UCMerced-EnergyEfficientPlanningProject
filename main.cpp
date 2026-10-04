// =============================================================================
//  OMPL RRT-family parameter sweep — corrected harness
//
//  Fixes relative to the previous version (each marked [FIX n] inline):
//   1. Collision checking used point-to-triangle-CENTROID distance, which
//      models every triangle as a 0.5-radius sphere at its centre. Large wall
//      triangles were effectively invisible and the planner passed through
//      them. Replaced with exact point-to-triangle distance.
//   2. isValid() scanned every triangle linearly on every check. Added a
//      uniform-grid broadphase.
//   3. RAPL wraparound was silently logged as 0.0 J. Now handled using
//      max_energy_range_uj, and summed across all packages.
//   4. No idle-power baseline was subtracted, so energy was ~a linear
//      function of runtime (r = 0.98). Baseline is now measured and reported.
//   5. exactSolnPlannerTerminationCondition() is a no-op for RRT*/Informed
//      RRT* (OMPL issue #708 — planners only register solutions with the
//      ProblemDefinition at the END of solve()). Optimizing planners
//      therefore ran with NO time bound at all. Added an explicit timeout and
//      logging of what actually caused termination.
//   6. `if (solved)` conflated exact and approximate solutions. Status is now
//      logged explicitly, along with the achieved solution cost.
//   7. RRTConnect silently ignored goal bias, so 1/7 of the goal-bias axis was
//      replicate noise. Planners now declare whether they consume it, and
//      non-consumers run the goal-bias loop only once.
//   8. The RNG was never seeded — runs were not reproducible. Seeded and
//      logged.
//   9. The path simplifier was toggled by commenting code out, producing two
//      incomparable CSVs. Both raw and simplified metrics are now logged from
//      the same run.
//  10. Only the multiplier was logged, not the absolute range/resolution.
//      Both are now recorded.
//  11. si->setup() was called inside the resolution loop, which warns on
//      repeat calls. Only the state space is re-set-up now.
//  12. Support for multiple start/goal queries per environment, so an
//      obstructed query can be swept alongside the line-of-sight one.
//  13. Per-run RAPL sampling read 0.0 J on 28% of runs: the counter updates
//      about once per millisecond and most runs finish faster than that.
//      The zeros were not uniform -- they hit the FASTEST planners hardest
//      (RRTConnect 67%, RRTstar 16%), biasing exactly the comparison the
//      sweep exists to make. Energy is now sampled once around a whole
//      batch of `trials` runs and attributed to individual trials in
//      proportion to their runtime. The raw batch measurement is logged
//      alongside, so the derivation stays auditable.
//  14. The sweep only ever planned from a query's ORIGINAL start state, so
//      "remaining path length" -- the input the runtime policy is supposed to
//      condition on -- had exactly one value per query. A reference path is
//      now planned once per query, sampled at N fractions along its length,
//      and each sample becomes a start state. Remaining along-path distance
//      and remaining straight-line distance are logged per row.
//  15. The cost threshold was a fixed per-query constant. Held fixed while the
//      start state slides toward the goal, it becomes trivially satisfiable:
//      an optimizing planner terminates on its first sample and looks free
//      near the goal for reasons that are pure artifact. The threshold is now
//      a per-query MULTIPLE of the current start-to-goal straight-line
//      distance, so difficulty stays comparable across start points.
//  16. [FIX 13] attributed whole-BATCH energy across trials by runtime. That
//      batch window also contained planner construction, setup(), path
//      simplification, getPlannerData() and CSV I/O -- for RRTConnect only
//      ~37% of batch wall time was inside solve(), so ~63% of unrelated work
//      was being billed to planning, reinstating the very bias FIX 13 removed.
//      RAPL is now bracketed around solve() itself and around simplification
//      separately. Per-trial direct reads (which still quantize to 0 below the
//      ~1 ms counter tick) are logged raw next to the batch-summed value that
//      analysis should use.
//  17. Simplification was an unconditional simplifyMax(), which flattened every
//      planner to ~3.8 waypoints and hid its own cost inside the batch. It is
//      now a swept parameter (none / budget / max) and separately timed. All
//      three modes are applied to COPIES of the same solved path, so the
//      comparison is paired and the solve is not repeated three times.
//  18. pRRT ran at Thread_Count=1 in every row ever collected. Thread count is
//      the only swept parameter that changes package POWER rather than just
//      duration, and is therefore the only one for which ranking by joules can
//      differ from ranking by seconds. It is now swept.
//  19. Added a collision-check counter to the validity checker. It measures
//      planner work in a hardware-independent way, so results transfer off
//      this desktop, and it cross-checks the RAPL numbers.
//  20. The resolution sweep spanned at most ~1.7x in time and ~0 in path
//      quality, while costing 3x the run budget. Pinned to the finest setting;
//      the budget is spent on start points instead.
//  21. Added a CLI so one planner can be run end-to-end for timing before the
//      full sweep is committed to, plus --dry-run cost estimation and a
//      running ETA.
//
//  Usage:
//    ./benchmark --dry-run                       # print the plan, run nothing
//    ./benchmark --planner RRTConnect            # one planner, everything else
//    ./benchmark --planner RRTstar --starts 4 --trials 3   # quick timing probe
//    ./benchmark                                 # full sweep
//
//  Build (adjust paths as needed):
//    g++ -O2 -std=c++17 main_fixed.cpp -o benchmark \
//        $(pkg-config --cflags --libs ompl assimp)
// =============================================================================

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <string>
#include <array>
#include <cmath>
#include <chrono>
#include <thread>
#include <limits>
#include <algorithm>
#include <functional>
#include <unordered_map>
#include <atomic>
#include <memory>
#include <stdexcept>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/util/Console.h>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// =============================================================================
//  [FIX 3] RAPL energy reader with wraparound handling and multi-package sum
// =============================================================================
class RaplReader {
public:
    RaplReader() {
        for (int pkg = 0; pkg < 8; ++pkg) {
            std::string base = "/sys/class/powercap/intel-rapl/intel-rapl:" +
                               std::to_string(pkg);
            std::ifstream probe(base + "/energy_uj");
            if (!probe.is_open()) continue;
            long long max_range = 0;
            std::ifstream mr(base + "/max_energy_range_uj");
            if (mr.is_open()) mr >> max_range;
            // If the kernel does not expose the range, fall back to the common
            // 32-bit microjoule counter width rather than assuming no wrap.
            if (max_range <= 0) max_range = 4294967296LL;
            domains_.push_back({base + "/energy_uj", max_range});
        }
        if (domains_.empty())
            std::cerr << "Warning: no RAPL domains readable. Energy will be 0.\n"
                      << "         (Try: sudo chmod -R a+r /sys/class/powercap/intel-rapl)\n";
    }

    bool available() const { return !domains_.empty(); }

    // Per-domain raw counter values, in microjoules.
    std::vector<long long> sample() const {
        std::vector<long long> out;
        out.reserve(domains_.size());
        for (const auto &d : domains_) {
            long long uj = 0;
            std::ifstream f(d.path);
            if (f.is_open()) f >> uj;
            out.push_back(uj);
        }
        return out;
    }

    // Joules consumed between two samples, correcting counter wraparound.
    double delta_joules(const std::vector<long long> &a,
                        const std::vector<long long> &b) const {
        double total_uj = 0.0;
        for (size_t i = 0; i < domains_.size() && i < a.size() && i < b.size(); ++i) {
            long long d = b[i] - a[i];
            if (d < 0) d += domains_[i].max_range;   // <-- the actual fix
            total_uj += static_cast<double>(d);
        }
        return total_uj / 1e6;
    }

    // [FIX 4] Idle package power, so runtime-proportional draw can be removed.
    double measure_idle_watts(double seconds = 2.0) const {
        if (!available()) return 0.0;
        auto s0 = sample();
        auto t0 = std::chrono::steady_clock::now();
        std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
        auto s1 = sample();
        auto t1 = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(t1 - t0).count();
        return dt > 0.0 ? delta_joules(s0, s1) / dt : 0.0;
    }

private:
    struct Domain { std::string path; long long max_range; };
    std::vector<Domain> domains_;
};

// =============================================================================
//  [FIX 1 + 2] Mesh collision checker: exact point-triangle distance,
//              accelerated by a uniform grid.
// =============================================================================
struct Triangle {
    std::array<double, 3> v0, v1, v2;
};

namespace {

inline std::array<double, 3> sub(const std::array<double, 3> &a,
                                 const std::array<double, 3> &b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
inline double dot(const std::array<double, 3> &a, const std::array<double, 3> &b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
inline std::array<double, 3> add_scaled(const std::array<double, 3> &p,
                                        const std::array<double, 3> &d, double t) {
    return {p[0] + d[0] * t, p[1] + d[1] * t, p[2] + d[2] * t};
}

// Closest point on triangle ABC to point P.
// Ericson, "Real-Time Collision Detection", §5.1.5.
std::array<double, 3> closest_point_on_triangle(const std::array<double, 3> &p,
                                                const std::array<double, 3> &a,
                                                const std::array<double, 3> &b,
                                                const std::array<double, 3> &c) {
    const auto ab = sub(b, a), ac = sub(c, a), ap = sub(p, a);
    const double d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0.0 && d2 <= 0.0) return a;                       // vertex A

    const auto bp = sub(p, b);
    const double d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0.0 && d4 <= d3) return b;                        // vertex B

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)                    // edge AB
        return add_scaled(a, ab, d1 / (d1 - d3));

    const auto cp = sub(p, c);
    const double d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0.0 && d5 <= d6) return c;                        // vertex C

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)                    // edge AC
        return add_scaled(a, ac, d2 / (d2 - d6));

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {    // edge BC
        const double t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return add_scaled(b, sub(c, b), t);
    }

    const double denom = 1.0 / (va + vb + vc);                  // face interior
    const double v = vb * denom, w = vc * denom;
    return {a[0] + ab[0] * v + ac[0] * w,
            a[1] + ab[1] * v + ac[1] * w,
            a[2] + ab[2] * v + ac[2] * w};
}

}  // namespace

class MeshValidityChecker : public ob::StateValidityChecker {
public:
    MeshValidityChecker(const ob::SpaceInformationPtr &si,
                        const std::string &mesh_path,
                        double margin = 0.5)
        : ob::StateValidityChecker(si), margin_(margin), margin_sq_(margin * margin) {

        Assimp::Importer importer;
        const aiScene *scene = importer.ReadFile(
            mesh_path, aiProcess_Triangulate | aiProcess_PreTransformVertices);
        if (!scene || !scene->mRootNode) {
            std::cerr << "ERROR: could not load mesh " << mesh_path << "\n";
            throw std::runtime_error("mesh load failed");
        }

        // NOTE: aiProcess_PreTransformVertices bakes the node hierarchy into
        // vertex coordinates. Without it, sub-mesh transforms are ignored and
        // geometry lands in the wrong place — a silent correctness bug.
        for (unsigned m = 0; m < scene->mNumMeshes; ++m) {
            const aiMesh *mesh = scene->mMeshes[m];
            for (unsigned f = 0; f < mesh->mNumFaces; ++f) {
                const aiFace &face = mesh->mFaces[f];
                if (face.mNumIndices != 3) continue;
                const aiVector3D &p0 = mesh->mVertices[face.mIndices[0]];
                const aiVector3D &p1 = mesh->mVertices[face.mIndices[1]];
                const aiVector3D &p2 = mesh->mVertices[face.mIndices[2]];
                triangles_.push_back({{p0.x, p0.y, p0.z},
                                      {p1.x, p1.y, p1.z},
                                      {p2.x, p2.y, p2.z}});
            }
        }
        if (triangles_.empty())
            std::cerr << "Warning: mesh contained no triangles.\n";
        build_grid();
        std::cout << "Loaded " << triangles_.size() << " triangles into "
                  << grid_.size() << " occupied grid cells (cell = "
                  << cell_ << ")\n";
    }

    bool isValid(const ob::State *state) const override {
        // [FIX 19] Hardware-independent work counter. Relaxed ordering: we only
        // ever read it between planner calls, and pRRT calls isValid from
        // several threads at once, so a fetch_add is required for correctness
        // but no synchronisation with other memory is needed.
        checks_.fetch_add(1, std::memory_order_relaxed);

        const auto *pos = state->as<ob::RealVectorStateSpace::StateType>();
        const std::array<double, 3> p{pos->values[0], pos->values[1], pos->values[2]};

        if (!si_->satisfiesBounds(state)) return false;

        auto it = grid_.find(cell_key(p));
        if (it == grid_.end()) return true;          // no geometry nearby

        for (int idx : it->second) {
            const Triangle &t = triangles_[idx];
            const auto q = closest_point_on_triangle(p, t.v0, t.v1, t.v2);
            const auto d = sub(p, q);
            if (dot(d, d) < margin_sq_) return false;
        }
        return true;
    }

    size_t triangle_count() const { return triangles_.size(); }

    // [FIX 19]
    long long collision_checks() const {
        return checks_.load(std::memory_order_relaxed);
    }

    // [FIX 22] Clearance query: distance from a point to the NEAREST triangle,
    // searched over a small window of cells around the point. This is not the
    // collision check (which only looks in the point's own cell within margin);
    // it deliberately scans neighbouring cells so it can report how close the
    // path comes to obstacles even when it is not colliding. The window is
    // capped: a point farther than `cap` from all geometry is simply "safe"
    // and we return the cap. That keeps the query O(window) instead of scanning
    // the whole mesh, and the exact value only matters when it is small.
    double clearance(const std::array<double, 3> &p, double cap = 5.0) const {
        const long r = std::max(1L, static_cast<long>(std::ceil(cap / cell_)));
        const long cx = cell_index(p[0]), cy = cell_index(p[1]), cz = cell_index(p[2]);
        double best_sq = cap * cap;
        for (long ix = cx - r; ix <= cx + r; ++ix)
        for (long iy = cy - r; iy <= cy + r; ++iy)
        for (long iz = cz - r; iz <= cz + r; ++iz) {
            auto it = grid_.find(key(ix, iy, iz));
            if (it == grid_.end()) continue;
            for (int idx : it->second) {
                const Triangle &t = triangles_[idx];
                const auto q = closest_point_on_triangle(p, t.v0, t.v1, t.v2);
                const auto d = sub(p, q);
                const double dsq = dot(d, d);
                if (dsq < best_sq) best_sq = dsq;
            }
        }
        return std::sqrt(best_sq);
    }

private:
    // Triangle AABBs are inflated by `margin` before insertion, so a point can
    // only be within `margin` of a triangle registered in its OWN cell — the
    // lookup stays a single cell regardless of cell size.
    //
    // Cell size is therefore a pure memory/scan tradeoff. Sizing it to the
    // margin (0.5) made a single 60x40 wall triangle occupy ~9,600 cells;
    // on Apartment_env.dae that produced 10,068,328 cells for 37,114
    // triangles (~1 GB of map). A coarser cell trades a slightly longer
    // per-cell scan for orders of magnitude less memory.
    void build_grid() {
        cell_ = std::max(margin_ * 16.0, 1e-6);
        for (size_t i = 0; i < triangles_.size(); ++i) {
            const Triangle &t = triangles_[i];
            double lo[3], hi[3];
            for (int k = 0; k < 3; ++k) {
                lo[k] = std::min({t.v0[k], t.v1[k], t.v2[k]}) - margin_;
                hi[k] = std::max({t.v0[k], t.v1[k], t.v2[k]}) + margin_;
            }
            for (long ix = cell_index(lo[0]); ix <= cell_index(hi[0]); ++ix)
                for (long iy = cell_index(lo[1]); iy <= cell_index(hi[1]); ++iy)
                    for (long iz = cell_index(lo[2]); iz <= cell_index(hi[2]); ++iz)
                        grid_[key(ix, iy, iz)].push_back(static_cast<int>(i));
        }
    }

    long cell_index(double v) const {
        return static_cast<long>(std::floor(v / cell_));
    }
    static long long key(long x, long y, long z) {
        // 21 bits per axis, offset to keep negatives positive.
        return ((x + 1048576LL) << 42) ^ ((y + 1048576LL) << 21) ^ (z + 1048576LL);
    }
    long long cell_key(const std::array<double, 3> &p) const {
        return key(cell_index(p[0]), cell_index(p[1]), cell_index(p[2]));
    }

    std::vector<Triangle> triangles_;
    std::unordered_map<long long, std::vector<int>> grid_;
    double margin_, margin_sq_, cell_ = 1.0;
    mutable std::atomic<long long> checks_{0};   // [FIX 19]
};

// =============================================================================
//  Configuration structs
// =============================================================================
struct Query {
    std::string name;
    std::vector<double> start, goal;
    // [FIX 15] Cost threshold as a MULTIPLE of the current start-to-goal
    // straight-line distance, not an absolute. The straight-line distance is
    // the lower bound on any achievable path, so the multiple must exceed 1.0;
    // how far it must exceed it depends on how much the obstacles force a
    // detour. Values below are the ones implied by the previously used
    // absolutes (direct 450/391.7 = 1.15, around_wall 340/260.0 = 1.31) and
    // are carried forward so results stay comparable to the existing runs.
    double threshold_mult;
};

struct EnvConfig {
    std::string filename;
    std::vector<double> bounds_low, bounds_high;
    std::vector<Query> queries;          // [FIX 12]
};

// [FIX 17] One simplification variant of one solved path.
struct SimpResult {
    std::string mode = "none";     // none | budget | max
    double budget_sec = 0.0;       // 0 for none/max
    double time_sec = 0.0;
    double energy_direct = 0.0;    // raw RAPL delta, may quantize to 0
    double len = -1.0, smooth = -1.0;
    int wp = 0;
    double clear_min = -1.0, clear_mean = -1.0;   // [FIX 25] path clearance
};

// [FIX 13/16] One trial's results, buffered until batch energy is known.
struct TrialResult {
    int trial = 0;
    double setup_time = 0.0;       // planner construction + setup(), excluded
                                   // from the energy attribution base
    double exec_time = 0.0;        // solve() only
    double solve_energy_direct = 0.0;   // [FIX 16] RAPL bracketed on solve()
    bool   solve_zero_read = false;     // direct read quantized to zero
    long long collision_checks = 0;     // [FIX 19]
    double cost = -1.0;
    double len_raw = -1.0, smooth_raw = -1.0;
    int wp_raw = 0, tree_size = 0;
    double clear_min_raw = -1.0, clear_mean_raw = -1.0;   // [FIX 25]
    bool timed_out = false;
    std::string status = "No solution";
    std::vector<SimpResult> simps;      // [FIX 17] one entry per mode
};

struct PlannerConfig {
    std::string name;
    // [FIX 18] factory now takes thread count too, so pRRT can be swept.
    std::function<ob::PlannerPtr(const ob::SpaceInformationPtr &,
                                 double, double, int)> factory;
    bool is_optimizing;
    bool uses_goal_bias;                 // [FIX 7]
    bool uses_threads = false;           // [FIX 18]
};

// [FIX 14] A start state sampled along a query's reference path.
struct StartPoint {
    int index = 0;
    double fraction = 0.0;        // of reference path length already flown
    double remaining_path = 0.0;  // along-path distance still to fly
    double straight_line = 0.0;   // straight-line distance to goal
    std::vector<double> pos{0.0, 0.0, 0.0};
};

// =============================================================================
// =============================================================================
//  [FIX 25] Path clearance: minimum and mean distance from the path to the
//  nearest obstacle, sampled along densely-interpolated waypoints. A short
//  path that skims a wall is more dangerous than a slightly longer one that
//  keeps its distance -- the EI/J score needs this to reason about whether a
//  coarse collision-resolution run produced a path that is actually risky.
//  The path is interpolated so clearance is measured continuously, not only
//  at the sparse waypoints a simplifier leaves behind.
// =============================================================================
static void path_clearance(const og::PathGeometric &path_in,
                           const MeshValidityChecker &checker,
                           double &out_min, double &out_mean) {
    og::PathGeometric path(path_in);
    path.interpolate(200);                 // dense sampling for a continuous measure
    const std::size_t n = path.getStateCount();
    if (n == 0) { out_min = out_mean = -1.0; return; }
    double mn = 1e18, sum = 0.0;
    std::size_t counted = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const auto *pos = path.getState(i)
                              ->as<ob::RealVectorStateSpace::StateType>();
        const std::array<double, 3> p{pos->values[0], pos->values[1], pos->values[2]};
        const double c = checker.clearance(p);
        mn = std::min(mn, c);
        sum += c; ++counted;
    }
    out_min  = mn;
    out_mean = counted ? sum / static_cast<double>(counted) : -1.0;
}

// =============================================================================
//  [FIX 21] Command line
// =============================================================================
struct Args {
    std::uint_fast32_t seed = 42u;
    std::string planner_filter;              // empty = every planner
    std::string query_filter;                // empty = every query
    int    starts        = 25;   // [FIX 23] finer path sampling for 5% replan model
    int    trials        = 10;
    bool   dry_run       = false;
    std::string out_path = "cpp_parameter_sweep_benchmark.csv";
};

static void print_usage() {
    std::cout <<
      "Usage: ./benchmark [options]\n"
      "  --planner NAME   run only this planner (RRT, RRTConnect, LazyRRT,\n"
      "                   pRRT, TRRT, RRTstar, InformedRRTstar)\n"
      "  --query NAME     run only this query (direct, around_wall)\n"
      "  --starts N       start points sampled along each reference path (default 25)\n"
      "  --trials N       trials per configuration (default 10)\n"
      "  --seed N         RNG seed (default 42)\n"
      "  --out PATH       output CSV path\n"
      "  --dry-run        print the plan and exit without planning\n"
      "  --help\n\n"
      "Run one planner first and check the reported ETA before committing to\n"
      "the full sweep:\n"
      "  ./benchmark --planner RRTstar --starts 3 --trials 3 --out probe.csv\n";
}

static Args parse_args(int argc, char **argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "ERROR: " << k << " needs a value\n";
                std::exit(2);
            }
            return argv[++i];
        };
        if      (k == "--planner")  a.planner_filter = next();
        else if (k == "--query")    a.query_filter   = next();
        else if (k == "--starts")   a.starts  = std::stoi(next());
        else if (k == "--trials")   a.trials  = std::stoi(next());
        else if (k == "--seed")     a.seed    = static_cast<std::uint_fast32_t>(std::stoul(next()));
        else if (k == "--out")      a.out_path = next();
        else if (k == "--dry-run")  a.dry_run = true;
        else if (k == "--help")   { print_usage(); std::exit(0); }
        else {
            std::cerr << "ERROR: unknown option " << k << "\n\n";
            print_usage();
            std::exit(2);
        }
    }
    if (a.starts < 1 || a.trials < 1) {
        std::cerr << "ERROR: --starts and --trials must be >= 1\n";
        std::exit(2);
    }
    return a;
}

// =============================================================================
//  [FIX 14] Reference path + start-point sampling
// =============================================================================
//  The runtime policy conditions on remaining path length, so the training set
//  has to contain more than one value of it. We plan one reference path per
//  query, then treat points along that path as start states. Sampling along a
//  real path rather than the free space matters: it is the distribution the
//  vehicle will actually replan from.
//
//  RRTConnect is used for the reference path because it is the cheapest way to
//  get a feasible path, and simplifyMax + interpolate then make it dense and
//  reasonably direct. The reference path is NOT part of the measured data.
// -----------------------------------------------------------------------------
static bool build_reference_path(const ob::SpaceInformationPtr &si,
                                 const ob::ScopedState<> &start,
                                 const ob::ScopedState<> &goal,
                                 double max_extent,
                                 og::PathGeometric &out) {
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    pdef->setStartAndGoalStates(start, goal);
    auto opt = std::make_shared<ob::PathLengthOptimizationObjective>(si);
    pdef->setOptimizationObjective(opt);

    auto planner = std::make_shared<og::RRTConnect>(si);
    planner->setRange(max_extent * 0.20);
    planner->setProblemDefinition(pdef);
    planner->setup();
    planner->solve(ob::timedPlannerTerminationCondition(60.0));

    if (!pdef->hasSolution()) return false;

    og::PathGeometric path(*pdef->getSolutionPath()->as<og::PathGeometric>());
    og::PathSimplifier(si).simplifyMax(path);
    // Dense resampling so a requested fraction lands close to an actual state.
    path.interpolate(1000);
    out = path;
    return true;
}

static std::vector<StartPoint> sample_start_points(const ob::SpaceInformationPtr &si,
                                                   const og::PathGeometric &ref,
                                                   const ob::ScopedState<> &goal,
                                                   int n) {
    std::vector<StartPoint> out;
    const std::size_t ns = ref.getStateCount();
    if (ns < 2) return out;

    // Cumulative along-path distance.
    std::vector<double> cum(ns, 0.0);
    for (std::size_t i = 1; i < ns; ++i)
        cum[i] = cum[i - 1] + si->distance(ref.getState(i - 1), ref.getState(i));
    const double total = cum.back();
    if (total <= 0.0) return out;

    for (int k = 0; k < n; ++k) {
        // Fractions span [0, 0.9]. Past ~0.9 the remaining problem is a few
        // metres of open space and every planner is indistinguishable, so those
        // rows would be replicate noise rather than data.
        const double f = (n == 1) ? 0.0 : 0.9 * static_cast<double>(k) / (n - 1);
        const double target = f * total;

        std::size_t idx = 0;
        while (idx + 1 < ns && cum[idx] < target) ++idx;

        const ob::State *s = ref.getState(idx);
        if (!si->isValid(s)) continue;   // should not happen on a valid path

        const auto *pos = s->as<ob::RealVectorStateSpace::StateType>();
        StartPoint sp;
        sp.index          = k;
        sp.fraction       = f;
        sp.remaining_path = total - cum[idx];
        sp.straight_line  = si->distance(s, goal.get());
        sp.pos            = {pos->values[0], pos->values[1], pos->values[2]};
        out.push_back(sp);
    }
    return out;
}

// =============================================================================
static std::string fmt_hms(double seconds) {
    if (seconds < 0.0 || !std::isfinite(seconds)) return "?";
    long s = static_cast<long>(seconds);
    std::ostringstream o;
    o << s / 3600 << "h" << std::setw(2) << std::setfill('0') << (s % 3600) / 60
      << "m" << std::setw(2) << std::setfill('0') << (s % 60) << "s";
    return o.str();
}

// =============================================================================
int main(int argc, char **argv) {
    const Args args = parse_args(argc, argv);

    // ---------------------------------------------------------------- config
    const int    trials            = args.trials;
    const double max_planning_time = 300.0;   // [FIX 5] hard cap, ALL planners
    const double simplify_budget   = 0.005;   // [FIX 17] seconds, "budget" mode

    // [FIX 8] Must be called before any ompl::RNG instance is constructed,
    // i.e. before any planner or sampler is built. Seeds the whole run.
    ompl::RNG::setSeed(args.seed);

    // One INFO line per solve x 10 trials x hundreds of batches buries the
    // progress output. Warnings and errors still print.
    ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

    // [FIX 24] Resolution sweep restored. FIX 20 pinned it to finest because it
    // barely moved timing on its own; but the whole point of the EI/J policy is
    // to THROTTLE collision resolution when a mission can tolerate a coarser
    // check (open space, energy-critical), so the data has to contain the
    // coarse/medium/fine operating points for the score to learn from. The
    // pilot_tunneling correctness concern is now directly observable via the
    // clearance columns: a coarse run that produces a low-clearance path is
    // exactly the risky case the score must learn to avoid.
    std::vector<double> resolutions = {0.0005, 0.0020, 0.0080};  // fine / med / coarse

    std::vector<double> goal_biases       = {0.02, 0.05, 0.15};   // Conserv/Standard/Aggressive
    std::vector<double> range_multipliers = {0.05, 0.20, 0.50};   // Short/Standard/Long
    std::vector<int>    thread_counts     = {1, 2, 4, 8};         // [FIX 18] pRRT only
    std::vector<std::string> simp_modes   = {"none", "budget", "max"};  // [FIX 17]

    std::vector<EnvConfig> environments = {
        {"Apartment_env.dae",
         {-80, -185, -5}, {300, 175, 95},
         {
            // [FIX 15] threshold multipliers, not absolutes.
            {"direct",      {266, 68, 24},   {-46, -160, 88}, 1.15},  // los,  391.7
            {"around_wall", {242, -104, 8},  {18, 24, 40},    1.31},  // obst, 260.0
         }},
    };

    RaplReader rapl;
    const double idle_watts = args.dry_run ? 0.0 : rapl.measure_idle_watts(2.0);
    if (!args.dry_run) {
        std::cout << "Idle package power baseline: " << idle_watts << " W\n";
        if (!rapl.available())
            std::cout << "WARNING: RAPL unavailable -- all energy columns will be 0.\n";
    }
    std::cout << "RNG seed: " << args.seed << "\n"
              << "Trials per config: " << trials
              << " | start points per query: " << args.starts << "\n\n";

    std::ofstream csv;
    if (!args.dry_run) {
        csv.open(args.out_path);
        csv << std::setprecision(10);
        csv << "Environment,Query,Start_Index,Start_Fraction,"
            << "Remaining_Path_Dist,Straight_Line_Dist,Start_X,Start_Y,Start_Z,"
            << "Planner,Is_Optimizing,Trial,Seed,"
            << "Threshold_Mult,Target_Threshold,"
            << "Config_Resolution,Config_GoalBias,Config_Range,"
            << "Resolution_Abs,GoalBias_Applied,Range_Abs,Thread_Count,"
            << "Simplify_Mode,Simplify_Budget_Sec,"
            << "Status,Solution_Cost,Timed_Out,"
            << "Setup_Time_Sec,Exec_Time_Sec,"
            << "Solve_Energy_Joules,Solve_Energy_Direct,Solve_Zero_Read,"
            << "Simplify_Time_Sec,Simplify_Energy_Joules,Simplify_Energy_Direct,"
            << "Collision_Checks,"
            << "Batch_Solve_Energy_Joules,Batch_Simplify_Energy_Joules,"
            << "Batch_Energy_Joules,Batch_Wall_Sec,Batch_Trials,Idle_Watts,"
            << "Path_Length_Raw,Path_Smoothness_Raw,Waypoint_Count_Raw,"
            << "Path_Length_Simp,Path_Smoothness_Simp,Waypoint_Count_Simp,"
            << "Clearance_Min_Raw,Clearance_Mean_Raw,"
            << "Clearance_Min_Simp,Clearance_Mean_Simp,"
            << "Tree_Node_Size\n";
    }

    long long batches_done = 0, batches_total = 0;
    const auto run_t0 = std::chrono::steady_clock::now();

    for (const auto &env : environments) {
        std::ifstream file_check(env.filename);
        if (!file_check.good()) {
            std::cout << "Skipping " << env.filename << " (not found)\n";
            continue;
        }

        auto space(std::make_shared<ob::RealVectorStateSpace>(3));
        ob::RealVectorBounds bounds(3);
        for (int i = 0; i < 3; ++i) {
            bounds.setLow(i, env.bounds_low[i]);
            bounds.setHigh(i, env.bounds_high[i]);
        }
        space->setBounds(bounds);

        auto si(std::make_shared<ob::SpaceInformation>(space));
        auto checker = std::make_shared<MeshValidityChecker>(si, env.filename, 0.5);
        si->setStateValidityChecker(checker);
        // [FIX 24] resolution is set per-config inside the sweep now, not once here.
        si->setStateValidityCheckingResolution(resolutions.front());
        si->setup();                              // [FIX 11] once, here only

        const double max_extent = space->getMaximumExtent();
        std::cout << "Max extent: " << max_extent
                  << " | resolutions swept: fine/med/coarse "
                  << resolutions[0] << "/" << resolutions[1] << "/" << resolutions[2]
                  << "\n";

        std::vector<PlannerConfig> planners = {
            {"RRT", [](auto si, double r, double gb, int) {
                auto p = std::make_shared<og::RRT>(si);
                p->setRange(r); p->setGoalBias(gb); return p; }, false, true, false},
            {"RRTConnect", [](auto si, double r, double, int) {
                auto p = std::make_shared<og::RRTConnect>(si);
                p->setRange(r); return p; }, false, false, false},   // [FIX 7]
            {"LazyRRT", [](auto si, double r, double gb, int) {
                auto p = std::make_shared<og::LazyRRT>(si);
                p->setRange(r); p->setGoalBias(gb); return p; }, false, true, false},
            {"pRRT", [](auto si, double r, double gb, int nt) {     // [FIX 18]
                auto p = std::make_shared<og::pRRT>(si);
                p->setRange(r); p->setGoalBias(gb);
                p->setThreadCount(nt); return p; }, false, true, true},
            {"TRRT", [](auto si, double r, double gb, int) {
                auto p = std::make_shared<og::TRRT>(si);
                p->setRange(r); p->setGoalBias(gb); return p; }, false, true, false},
            {"RRTstar", [](auto si, double r, double gb, int) {
                auto p = std::make_shared<og::RRTstar>(si);
                p->setRange(r); p->setGoalBias(gb); return p; }, true, true, false},
            {"InformedRRTstar", [](auto si, double r, double gb, int) {
                auto p = std::make_shared<og::InformedRRTstar>(si);
                p->setRange(r); p->setGoalBias(gb); return p; }, true, true, false},
        };

        if (!args.planner_filter.empty()) {
            std::vector<PlannerConfig> keep;
            for (const auto &p : planners)
                if (p.name == args.planner_filter) keep.push_back(p);
            if (keep.empty()) {
                std::cerr << "ERROR: no planner named '" << args.planner_filter << "'\n";
                return 2;
            }
            planners = keep;
        }

        for (const auto &query : env.queries) {
            if (!args.query_filter.empty() && query.name != args.query_filter) continue;

            ob::ScopedState<> start(space), goal(space);
            for (int i = 0; i < 3; ++i) {
                start[i] = query.start[i];
                goal[i]  = query.goal[i];
            }
            if (!si->isValid(start.get()) || !si->isValid(goal.get())) {
                std::cerr << "ERROR: query '" << query.name
                          << "' has an invalid start or goal state. Skipping.\n";
                continue;
            }
            std::cout << "\nQuery '" << query.name << "': straight-line motion is "
                      << (si->checkMotion(start.get(), goal.get())
                              ? "VALID (line-of-sight)" : "BLOCKED (obstructed)")
                      << "\n";

            // [FIX 14] reference path -> start points
            og::PathGeometric ref(si);
            if (!build_reference_path(si, start, goal, max_extent, ref)) {
                std::cerr << "ERROR: could not build a reference path for '"
                          << query.name << "'. Skipping.\n";
                continue;
            }
            auto start_points = sample_start_points(si, ref, goal, args.starts);
            if (start_points.empty()) {
                std::cerr << "ERROR: reference path yielded no usable start points.\n";
                continue;
            }
            std::cout << "  reference path: " << ref.length()
                      << " units over " << ref.getStateCount() << " states -> "
                      << start_points.size() << " start points\n";

            // ---- count the work this query implies (for --dry-run and the ETA)
            for (const auto &p_config : planners) {
                const std::size_t nb = p_config.uses_goal_bias ? goal_biases.size() : 1;
                const std::size_t nt = p_config.uses_threads ? thread_counts.size() : 1;
                batches_total += static_cast<long long>(start_points.size())
                               * nb * nt * range_multipliers.size()
                               * resolutions.size();   // [FIX 24]
            }
            if (args.dry_run) continue;

            for (const auto &sp : start_points) {
                ob::ScopedState<> s_start(space);
                for (int i = 0; i < 3; ++i) s_start[i] = sp.pos[i];

                // [FIX 15] threshold tracks the CURRENT start-to-goal distance.
                const double threshold = query.threshold_mult * sp.straight_line;

                std::cout << "  start " << sp.index << "/" << start_points.size() - 1
                          << " f=" << std::fixed << std::setprecision(2) << sp.fraction
                          << " remaining=" << std::setprecision(1) << sp.remaining_path
                          << " sl=" << sp.straight_line
                          << " thr=" << threshold
                          << std::defaultfloat << std::setprecision(6) << "\n";

                for (const auto &p_config : planners) {
                    // [FIX 7] planners that ignore goal bias run it once.
                    std::vector<double> biases = p_config.uses_goal_bias
                        ? goal_biases : std::vector<double>{goal_biases[0]};
                    // [FIX 18] only pRRT sweeps threads.
                    std::vector<int> threads = p_config.uses_threads
                        ? thread_counts : std::vector<int>{1};

                    for (int nt : threads)
                    for (double gb : biases)
                    for (double resolution : resolutions)   // [FIX 24]
                    for (double r_mult : range_multipliers) {
                        const double range = max_extent * r_mult;
                        const double res_abs = max_extent * resolution;
                        // [FIX 24] apply this config's collision-check resolution.
                        si->setStateValidityCheckingResolution(resolution);

                        std::vector<TrialResult> batch;
                        batch.reserve(trials);

                        const auto batch_e0 = rapl.sample();
                        const auto batch_t0 = std::chrono::steady_clock::now();

                        for (int t = 1; t <= trials; ++t) {
                            // ---- setup, timed but NOT charged to planning ----
                            const auto su0 = std::chrono::steady_clock::now();
                            auto pdef(std::make_shared<ob::ProblemDefinition>(si));
                            pdef->setStartAndGoalStates(s_start, goal);

                            auto opt = std::make_shared<ob::PathLengthOptimizationObjective>(si);
                            opt->setCostThreshold(ob::Cost(threshold));
                            pdef->setOptimizationObjective(opt);

                            ob::PlannerPtr planner = p_config.factory(si, range, gb, nt);
                            planner->setProblemDefinition(pdef);
                            planner->setup();
                            const auto su1 = std::chrono::steady_clock::now();

                            // [FIX 5] Every planner gets a hard time bound.
                            // exactSolnPlannerTerminationCondition alone is a
                            // no-op here (OMPL #708): RRT*/Informed RRT* only
                            // register solutions with the ProblemDefinition at
                            // the end of solve(), so it never fires. Real
                            // termination comes from the cost threshold, with
                            // this timeout as the backstop.
                            auto ptc = ob::plannerOrTerminationCondition(
                                ob::timedPlannerTerminationCondition(max_planning_time),
                                ob::exactSolnPlannerTerminationCondition(pdef));

                            TrialResult r;
                            r.trial      = t;
                            r.setup_time = std::chrono::duration<double>(su1 - su0).count();

                            // ---- [FIX 16] RAPL bracketed on solve() alone ----
                            const long long cc0 = checker->collision_checks();
                            const auto e0 = rapl.sample();
                            const auto t0 = std::chrono::steady_clock::now();
                            ob::PlannerStatus status = planner->solve(ptc);
                            const auto t1 = std::chrono::steady_clock::now();
                            const auto e1 = rapl.sample();
                            const long long cc1 = checker->collision_checks();

                            r.exec_time           = std::chrono::duration<double>(t1 - t0).count();
                            r.solve_energy_direct = rapl.delta_joules(e0, e1);
                            r.solve_zero_read     = (r.solve_energy_direct <= 0.0);
                            r.collision_checks    = cc1 - cc0;
                            r.timed_out           = r.exec_time >= max_planning_time * 0.999;

                            // [FIX 6] exact vs approximate reported honestly
                            switch (ob::PlannerStatus::StatusType(status)) {
                                case ob::PlannerStatus::EXACT_SOLUTION:
                                    r.status = "Exact solution"; break;
                                case ob::PlannerStatus::APPROXIMATE_SOLUTION:
                                    r.status = "Approximate solution"; break;
                                case ob::PlannerStatus::TIMEOUT:
                                    r.status = "Timeout"; break;
                                default:
                                    r.status = "No solution"; break;
                            }

                            if (pdef->hasSolution()) {
                                auto *path = pdef->getSolutionPath()->as<og::PathGeometric>();
                                r.len_raw    = path->length();
                                r.smooth_raw = path->smoothness();
                                r.wp_raw     = static_cast<int>(path->getStateCount());
                                r.cost       = path->cost(opt).value();
                                path_clearance(*path, *checker,
                                               r.clear_min_raw, r.clear_mean_raw);  // [FIX 25]

                                // [FIX 17] All simplification modes applied to
                                // COPIES of this one solved path. Paired
                                // comparison, and the solve is not repeated.
                                for (const auto &mode : simp_modes) {
                                    SimpResult s;
                                    s.mode = mode;
                                    og::PathGeometric copy(*path);
                                    og::PathSimplifier simplifier(si);

                                    const auto se0 = rapl.sample();
                                    const auto st0 = std::chrono::steady_clock::now();
                                    if (mode == "budget") {
                                        s.budget_sec = simplify_budget;
                                        simplifier.simplify(copy, simplify_budget);
                                    } else if (mode == "max") {
                                        simplifier.simplifyMax(copy);
                                    }   // "none": measure the no-op, keep the row shape
                                    const auto st1 = std::chrono::steady_clock::now();
                                    const auto se1 = rapl.sample();

                                    s.time_sec      = std::chrono::duration<double>(st1 - st0).count();
                                    s.energy_direct = rapl.delta_joules(se0, se1);
                                    s.len           = copy.length();
                                    s.smooth        = copy.smoothness();
                                    s.wp            = static_cast<int>(copy.getStateCount());
                                    path_clearance(copy, *checker,
                                                   s.clear_min, s.clear_mean);   // [FIX 25]
                                    r.simps.push_back(s);
                                }
                            } else {
                                for (const auto &mode : simp_modes) {
                                    SimpResult s;
                                    s.mode = mode;
                                    if (mode == "budget") s.budget_sec = simplify_budget;
                                    r.simps.push_back(s);
                                }
                            }

                            ob::PlannerData data(si);
                            planner->getPlannerData(data);
                            r.tree_size = static_cast<int>(data.numVertices());

                            batch.push_back(std::move(r));
                        }

                        const auto batch_t1 = std::chrono::steady_clock::now();
                        const auto batch_e1 = rapl.sample();
                        const double batch_wall =
                            std::chrono::duration<double>(batch_t1 - batch_t0).count();
                        const double batch_energy = rapl.delta_joules(batch_e0, batch_e1);

                        // [FIX 16] Batch-summed, solve-only energy. Individual
                        // reads still quantize to 0 below the ~1 ms RAPL tick,
                        // but their sum over `trials` does not, and unlike the
                        // whole-batch window it contains no setup, no
                        // simplification, no getPlannerData and no I/O.
                        double sum_time = 0.0, batch_solve_e = 0.0, batch_simp_e = 0.0;
                        double sum_simp_time = 0.0;
                        for (const auto &r : batch) {
                            sum_time      += r.exec_time;
                            batch_solve_e += r.solve_energy_direct;
                            for (const auto &s : r.simps) {
                                batch_simp_e  += s.energy_direct;
                                sum_simp_time += s.time_sec;
                            }
                        }
                        const double batch_solve_net =
                            std::max(0.0, batch_solve_e - idle_watts * sum_time);
                        const double batch_simp_net =
                            std::max(0.0, batch_simp_e - idle_watts * sum_simp_time);

                        for (const auto &r : batch) {
                            // Attribute the batch-summed solve energy across
                            // trials by runtime, so a config whose trials differ
                            // in cost is not flattened to an average.
                            const double share = (sum_time > 0.0)
                                ? r.exec_time / sum_time
                                : 1.0 / static_cast<double>(batch.size());
                            const double solve_energy = batch_solve_net * share;

                            for (const auto &s : r.simps) {
                                const double simp_share = (sum_simp_time > 0.0)
                                    ? s.time_sec / sum_simp_time
                                    : 0.0;
                                const double simp_energy = batch_simp_net * simp_share;

                                csv << env.filename << ','
                                    << query.name << ','
                                    << sp.index << ',' << sp.fraction << ','
                                    << sp.remaining_path << ',' << sp.straight_line << ','
                                    << sp.pos[0] << ',' << sp.pos[1] << ',' << sp.pos[2] << ','
                                    << p_config.name << ','
                                    << (p_config.is_optimizing ? "Yes" : "No") << ','
                                    << r.trial << ','
                                    << args.seed << ','
                                    << query.threshold_mult << ',' << threshold << ','
                                    << resolution << ','
                                    << (p_config.uses_goal_bias ? gb : -1.0) << ','
                                    << r_mult << ','
                                    << res_abs << ','
                                    << (p_config.uses_goal_bias ? "Yes" : "No") << ','
                                    << range << ',' << nt << ','
                                    << s.mode << ',' << s.budget_sec << ','
                                    << r.status << ','
                                    << r.cost << ','
                                    << (r.timed_out ? "Yes" : "No") << ','
                                    << r.setup_time << ',' << r.exec_time << ','
                                    << solve_energy << ',' << r.solve_energy_direct << ','
                                    << (r.solve_zero_read ? "Yes" : "No") << ','
                                    << s.time_sec << ',' << simp_energy << ','
                                    << s.energy_direct << ','
                                    << r.collision_checks << ','
                                    << batch_solve_net << ',' << batch_simp_net << ','
                                    << batch_energy << ',' << batch_wall << ','
                                    << batch.size() << ',' << idle_watts << ','
                                    << r.len_raw << ',' << r.smooth_raw << ',' << r.wp_raw << ','
                                    << s.len << ',' << s.smooth << ',' << s.wp << ','
                                    << r.clear_min_raw << ',' << r.clear_mean_raw << ','
                                    << s.clear_min << ',' << s.clear_mean << ','
                                    << r.tree_size << '\n';
                            }
                        }
                        csv.flush();   // survive a crash mid-sweep

                        // ---- [FIX 21] progress + ETA -----------------------
                        ++batches_done;
                        const double elapsed =
                            std::chrono::duration<double>(
                                std::chrono::steady_clock::now() - run_t0).count();
                        const double per_batch = elapsed / static_cast<double>(batches_done);
                        std::cout << "    [" << batches_done << "/" << batches_total << "] "
                                  << p_config.name
                                  << " thr=" << nt << " gb=" << gb << " rng=" << r_mult
                                  << " | " << batch_wall << " s batch"
                                  << " | solve " << batch_solve_net << " J net"
                                  << " | elapsed " << fmt_hms(elapsed)
                                  << " | ETA "
                                  << fmt_hms(per_batch * (batches_total - batches_done))
                                  << "\n";
                    }
                }
            }
        }
    }

    if (args.dry_run) {
        std::cout << "\nDRY RUN\n"
                  << "  batches            : " << batches_total << "\n"
                  << "  solves             : " << batches_total * trials << "\n"
                  << "  CSV rows           : "
                  << batches_total * trials * static_cast<long long>(simp_modes.size()) << "\n\n"
                  << "Nothing was planned. To get a real time estimate, run one\n"
                  << "planner with a small sweep and read the ETA it prints:\n"
                  << "  ./benchmark --planner RRTstar --starts 3 --trials 3 --out probe.csv\n";
        return 0;
    }

    csv.close();
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - run_t0).count();
    std::cout << "\nSweep complete in " << fmt_hms(elapsed)
              << " -> " << args.out_path << "\n";
    return 0;
}
