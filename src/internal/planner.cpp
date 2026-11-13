#include "internal/planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#include "internal/blend.hpp"
#include "internal/scurve_profile.hpp"
#include "internal/trap_profile.hpp"

namespace justblend::internal
{

namespace
{

// Corners whose direction change is within ~2.6 degrees of a full reversal
// cannot be blended through: the blend would start and end at the same point
// and miss the waypoint entirely.
constexpr double kReversalCosThreshold = -0.999;

// A run of waypoints is merged into one straight leg when every inner
// waypoint lies within this distance of the run's chord, relative to the
// chord length.
constexpr double kCollinearTolerance = 1e-6;

// BLEND_WHERE_FASTER keeps a corner change only when it shortens the trajectory by more
// than this fraction, so rounding cannot make the sweep flip back and forth.
constexpr double kImprovementTolerance = 1e-12;

/// @brief One leg of the reduced path: the straight run between two kept
/// waypoints, with any collinear waypoints in between merged into it.
struct Leg
{
    Eigen::Index start = 0; ///< Waypoint index where the leg starts.
    Eigen::Index end = 0;   ///< Waypoint index where the leg ends.
    double length = 0.0;
    Eigen::VectorXd dir;
    std::vector<double> inner_s;    ///< Arc position of each merged waypoint, from the leg start.
    std::vector<double> inner_miss; ///< Distance of each merged waypoint from the leg.
};

/// @brief Check whether waypoints a..b form one straight run.
/// @param W Waypoint matrix, one waypoint per row.
/// @param a First waypoint of the run.
/// @param b Last waypoint of the run.
/// @param max_miss Largest distance an inner waypoint may be from the chord,
/// on top of the relative kCollinearTolerance.
/// @return True when every inner waypoint lies, in order, within the
/// tolerance of the chord from a to b.
bool isStraightRun(const Eigen::MatrixXd& W, Eigen::Index a, Eigen::Index b, double max_miss)
{
    const Eigen::VectorXd chord = (W.row(b) - W.row(a)).transpose();
    const double length = chord.norm();
    const Eigen::VectorXd e = chord / length;
    const double tolerance = std::min(kCollinearTolerance * length, max_miss);
    double previous = 0.0;
    for (Eigen::Index i = a + 1; i < b; ++i)
    {
        const Eigen::VectorXd offset = (W.row(i) - W.row(a)).transpose();
        const double s = offset.dot(e);
        if (!(s > previous) || !(s < length) || (offset - s * e).norm() > tolerance)
            return false;
        previous = s;
    }
    return true;
}

/// @brief Split the waypoints into straight runs, each extended as far as a
/// doubling-and-bisection search for its end finds.
/// @param W Waypoint matrix, one waypoint per row; consecutive rows are distinct.
/// @param max_miss Largest distance a merged waypoint may be from its leg.
/// @return The legs of the reduced path, in order.
std::vector<Leg> mergeCollinear(const Eigen::MatrixXd& W, double max_miss)
{
    std::vector<Leg> legs;
    Eigen::Index a = 0;
    while (a < W.rows() - 1)
    {
        // Each check costs the run length, so find where the run ends by
        // doubling and then bisecting rather than one waypoint at a time.
        Eigen::Index b = a + 1;
        Eigen::Index too_far = b + 1;
        while (too_far < W.rows() && isStraightRun(W, a, too_far, max_miss))
        {
            b = too_far;
            too_far = a + 2 * (too_far - a);
        }
        too_far = std::min(too_far, W.rows());
        while (too_far - b > 1)
        {
            const Eigen::Index mid = b + (too_far - b) / 2;
            if (isStraightRun(W, a, mid, max_miss))
                b = mid;
            else
                too_far = mid;
        }

        Leg leg;
        leg.start = a;
        leg.end = b;
        const Eigen::VectorXd chord = (W.row(b) - W.row(a)).transpose();
        leg.length = chord.norm();
        leg.dir = chord / leg.length;
        for (Eigen::Index i = a + 1; i < b; ++i)
        {
            const Eigen::VectorXd offset = (W.row(i) - W.row(a)).transpose();
            const double s = offset.dot(leg.dir);
            leg.inner_s.push_back(s);
            leg.inner_miss.push_back((offset - s * leg.dir).norm());
        }
        legs.push_back(std::move(leg));
        a = b;
    }
    return legs;
}

/// @brief Fixed inputs of the junction-speed problem on the reduced path.
/// Vertex k joins leg k - 1 to leg k.
struct SpeedProblem
{
    Eigen::VectorXd length;          ///< Per leg.
    Eigen::VectorXd sd_max;          ///< Per leg: speed limit along the leg.
    Eigen::VectorXd sdd_max;         ///< Per leg: acceleration limit along the leg.
    Eigen::VectorXd sddd_max;        ///< Per leg: jerk limit along the leg; empty for trapezoidal.
    std::vector<double> blend_r;     ///< Per vertex: radius if the corner is blended.
    std::vector<double> blend_v_cap; ///< Per vertex: speed cap if the corner is blended.
    double v_start = 0.0;
    double v_end = 0.0;
    bool scurve = false;
};

/// @brief Junction speeds and durations for one choice of corner types.
struct SpeedPlan
{
    std::vector<CornerType> type; ///< Per vertex.
    Eigen::VectorXd L_eff;        ///< Per leg: straight length left between blends.
    Eigen::VectorXd forward;      ///< Per vertex: speeds after the forward pass.
    Eigen::VectorXd U;            ///< Per vertex: final junction speeds.
    Eigen::VectorXd leg_time;     ///< Per leg: duration of its straight part.
    double duration = 0.0;
};

/// @brief Highest speed reachable from v0 over a straight length of leg m.
/// @param c Speed problem.
/// @param m Leg index.
/// @param v0 Speed at the start of the length.
/// @param length Distance available.
/// @return The reachable speed.
double reachable(const SpeedProblem& c, Eigen::Index m, double v0, double length)
{
    return c.scurve ? reachableScurve(v0, c.sdd_max(m), c.sddd_max(m), length, c.sd_max(m))
                    : reachableTrap(v0, c.sdd_max(m), length);
}

/// @brief Speed cap at vertex k for the plan's corner types.
/// @param c Speed problem.
/// @param p Plan supplying the corner types.
/// @param k Vertex index.
/// @return The boundary speed at an endpoint, the blend cap at a blend, else 0.
double speedCap(const SpeedProblem& c, const SpeedPlan& p, Eigen::Index k)
{
    if (k == 0)
        return c.v_start;
    if (k == static_cast<Eigen::Index>(p.type.size()) - 1)
        return c.v_end;
    return p.type[k] == CornerType::BLEND ? c.blend_v_cap[k] : 0.0;
}

/// @brief Straight length of leg m left after the blends at its ends.
/// @param c Speed problem.
/// @param p Plan supplying the corner types.
/// @param m Leg index.
/// @return The length, clamped at 0 against rounding.
double effectiveLength(const SpeedProblem& c, const SpeedPlan& p, Eigen::Index m)
{
    double L = c.length(m);
    if (p.type[m] == CornerType::BLEND)
        L -= c.blend_r[m];
    if (p.type[m + 1] == CornerType::BLEND)
        L -= c.blend_r[m + 1];
    return std::max(L, 0.0);
}

/// @brief Forward-pass speed at vertex k from the forward speed at k - 1.
double forwardSpeed(const SpeedProblem& c, const SpeedPlan& p, Eigen::Index k)
{
    if (k == 0)
        return speedCap(c, p, 0);
    const double L = p.L_eff(k - 1);
    const double reach = L > 1e-12 ? reachable(c, k - 1, p.forward(k - 1), L) : p.forward(k - 1);
    return std::min(speedCap(c, p, k), reach);
}

/// @brief Final speed at vertex k from the forward speed at k and the final speed at k + 1.
double backwardSpeed(const SpeedProblem& c, const SpeedPlan& p, Eigen::Index k)
{
    if (k == p.forward.size() - 1)
        return p.forward(k);
    const double L = p.L_eff(k);
    const double reach = L > 1e-12 ? reachable(c, k, p.U(k + 1), L) : p.U(k + 1);
    return std::min(p.forward(k), reach);
}

/// @brief Duration of the straight part of leg m.
double legDuration(const SpeedProblem& c, const SpeedPlan& p, Eigen::Index m)
{
    const double L = p.L_eff(m);
    if (!(L > 1e-12))
        return 0.0;
    double T = 0.0;
    if (c.scurve)
    {
        for (const auto& phase : scurvePhases(p.U(m), p.U(m + 1), c.sd_max(m), c.sdd_max(m), c.sddd_max(m), L))
            T += phase.duration;
    }
    else
    {
        for (const auto& phase : trapPhases(p.U(m), p.U(m + 1), c.sd_max(m), c.sdd_max(m), L))
            T += phase.duration;
    }
    return T;
}

/// @brief Sum the leg and blend durations into p.duration.
void updateDuration(const SpeedProblem& c, SpeedPlan& p)
{
    p.duration = p.leg_time.sum();
    for (std::size_t k = 0; k < p.type.size(); ++k)
    {
        if (p.type[k] == CornerType::BLEND)
            p.duration += 2.0 * c.blend_r[k] / p.U(static_cast<Eigen::Index>(k));
    }
}

/// @brief Solve the junction speeds for the plan's corner types.
/// @param c Speed problem.
/// @param p Plan with its corner types set; every field but the durations is
/// overwritten.
void solveSpeeds(const SpeedProblem& c, SpeedPlan& p)
{
    const auto M = static_cast<Eigen::Index>(p.type.size());
    p.L_eff.resize(M - 1);
    p.forward.resize(M);
    p.U.resize(M);
    for (Eigen::Index m = 0; m < M - 1; ++m)
        p.L_eff(m) = effectiveLength(c, p, m);
    for (Eigen::Index k = 0; k < M; ++k)
        p.forward(k) = forwardSpeed(c, p, k);
    for (Eigen::Index k = M - 1; k >= 0; --k)
        p.U(k) = backwardSpeed(c, p, k);
}

/// @brief Fill the leg and total durations of a plan with solved speeds.
///
/// Kept apart from solveSpeeds() because each leg needs a full profile, which
/// only the BLEND_WHERE_FASTER corner choice uses; segment building solves its own.
/// @param c Speed problem.
/// @param p Plan with solved speeds.
void solveDurations(const SpeedProblem& c, SpeedPlan& p)
{
    p.leg_time.resize(p.L_eff.size());
    for (Eigen::Index m = 0; m < p.L_eff.size(); ++m)
        p.leg_time(m) = legDuration(c, p, m);
    updateDuration(c, p);
}

/// @brief Time spent in the blend at vertex k, or 0 when it is not blended.
double blendDuration(const SpeedProblem& c, CornerType type, Eigen::Index k, double speed)
{
    return type == CornerType::BLEND ? 2.0 * c.blend_r[k] / speed : 0.0;
}

/// @brief What toggling one corner did to a plan.
struct ToggleEffect
{
    double delta = 0.0;   ///< Change in duration.
    Eigen::Index lo = 0;  ///< First vertex whose speed may have changed.
    Eigen::Index hi = 0;  ///< Last vertex whose speed may have changed.
    bool feasible = true; ///< Whether the toggled plan passes isFeasible().
};

/// @brief Toggle interior vertex k between BLEND and STOP, recomputing only
/// the junctions whose speed changes. Every field except p.duration then
/// equals a full solveSpeeds() of the toggled corner types, so toggling the
/// same corner again restores the plan exactly.
/// @param c Speed problem.
/// @param p Solved plan, updated in place except for p.duration.
/// @param k Interior vertex index.
/// @return The change in duration and the range of vertices it touched.
ToggleEffect toggleCorner(const SpeedProblem& c, SpeedPlan& p, Eigen::Index k)
{
    const auto M = static_cast<Eigen::Index>(p.type.size());
    const CornerType old_type = p.type[k];
    p.type[k] = old_type == CornerType::BLEND ? CornerType::STOP : CornerType::BLEND;
    p.L_eff(k - 1) = effectiveLength(c, p, k - 1);
    p.L_eff(k) = effectiveLength(c, p, k);

    // Forward speeds before k and their legs are untouched. Past k + 1, a
    // junction that keeps its old forward speed ends the change.
    ToggleEffect effect;
    effect.hi = k;
    for (Eigen::Index j = k; j < M; ++j)
    {
        const double v = forwardSpeed(c, p, j);
        if (j > k + 1 && v == p.forward(j))
            break;
        p.forward(j) = v;
        effect.hi = j;
    }
    // Final speeds past hi are untouched. Below k - 1, a junction that keeps
    // its old final speed ends the change.
    effect.lo = effect.hi;
    for (Eigen::Index j = effect.hi; j >= 0; --j)
    {
        const double v = backwardSpeed(c, p, j);
        if (j < k - 1 && v == p.U(j))
            break;
        const CornerType type_before = j == k ? old_type : p.type[j];
        effect.delta += blendDuration(c, p.type[j], j, v) - blendDuration(c, type_before, j, p.U(j));
        p.U(j) = v;
        effect.lo = j;
    }
    for (Eigen::Index m = std::max<Eigen::Index>(effect.lo - 1, 0); m <= std::min(effect.hi, M - 2); ++m)
    {
        const double T = legDuration(c, p, m);
        effect.delta += T - p.leg_time(m);
        p.leg_time(m) = T;
    }

    // Blends outside the range kept their speeds, and were feasible before.
    for (Eigen::Index j = effect.lo; j <= effect.hi && effect.feasible; ++j)
        effect.feasible = !(p.type[j] == CornerType::BLEND && p.U(j) < 1e-6);
    effect.feasible = effect.feasible && p.U(0) + 1e-9 >= c.v_start && p.U(M - 1) + 1e-9 >= c.v_end;
    return effect;
}

/// @brief Check that a plan keeps every blend moving and meets the boundary speeds.
bool isFeasible(const SpeedProblem& c, const SpeedPlan& p)
{
    for (std::size_t k = 0; k < p.type.size(); ++k)
    {
        if (p.type[k] == CornerType::BLEND && p.U(static_cast<Eigen::Index>(k)) < 1e-6)
            return false;
    }
    return p.U(0) + 1e-9 >= c.v_start && p.U(p.U.size() - 1) + 1e-9 >= c.v_end;
}

/// @brief BLEND_WHERE_FASTER corner choice: starting from the given plan, repeatedly
/// apply the single corner toggle (BLEND to STOP or back) that shortens the
/// trajectory most, until none does; then fall back to stopping at every
/// candidate if that is faster still.
/// @param c Speed problem.
/// @param p Feasible plan with solved speeds; replaced by the chosen plan.
/// @param candidates Vertices that may blend or stop.
void chooseFastestCorners(const SpeedProblem& c, SpeedPlan& p, const std::vector<Eigen::Index>& candidates)
{
    solveDurations(c, p);
    // A toggle only changes the speeds in its effect range, so a cached
    // effect stays exact until a committed toggle lands next to its range.
    std::vector<ToggleEffect> effects(candidates.size());
    std::vector<bool> stale(candidates.size(), true);
    while (true)
    {
        std::size_t best = candidates.size();
        double best_delta = -kImprovementTolerance * p.duration;
        for (std::size_t i = 0; i < candidates.size(); ++i)
        {
            if (stale[i])
            {
                effects[i] = toggleCorner(c, p, candidates[i]);
                toggleCorner(c, p, candidates[i]);
                stale[i] = false;
            }
            if (effects[i].feasible && effects[i].delta < best_delta)
            {
                best = i;
                best_delta = effects[i].delta;
            }
        }
        if (best == candidates.size())
            break;

        const ToggleEffect committed = toggleCorner(c, p, candidates[best]);
        updateDuration(c, p);
        for (std::size_t i = 0; i < candidates.size(); ++i)
        {
            stale[i] = stale[i] || (effects[i].lo - 1 <= committed.hi && committed.lo <= effects[i].hi + 1);
        }
    }

    SpeedPlan all_stop = p;
    for (Eigen::Index k : candidates)
        all_stop.type[k] = CornerType::STOP;
    solveSpeeds(c, all_stop);
    solveDurations(c, all_stop);
    if (all_stop.duration < p.duration && isFeasible(c, all_stop))
        p = std::move(all_stop);
}

/// @brief Smallest t in [0, lin.duration] at which the segment has covered arc s.
/// @param lin Linear segment.
/// @param s Arc length from the segment start, within [0, lin.length].
/// @return The passage time.
double timeAtArc(const LinearSegmentData& lin, double s)
{
    auto arcAt = [&](double t)
    { return lin.is_scurve ? sampleScurve(t, lin.v0, lin.scurve_phases).s : sampleTrap(t, lin.v0, lin.trap_phases).s; };
    double lo = 0.0;
    double hi = lin.duration;
    for (int i = 0; i < 64; ++i)
    {
        const double mid = 0.5 * (lo + hi);
        if (arcAt(mid) < s)
            lo = mid;
        else
            hi = mid;
    }
    return hi;
}

/// @brief Ratio of a blend's corner deviation to r * |d_out - d_in|.
/// @param shape Blend shape.
/// @return 1/4 for a parabolic blend, 3/16 for a Hermite blend.
double deviationFactor(BlendShape shape) { return shape == BlendShape::PARABOLIC ? 0.25 : 3.0 / 16.0; }

SegmentInfo extractInfo(const Segment& seg)
{
    SegmentInfo info;
    info.type = seg.type;
    info.duration = seg.duration;
    if (seg.type == SegmentType::LINEAR)
    {
        info.v0 = seg.linear.v0;
        info.length = seg.linear.length;
        info.u_dir = seg.linear.u_dir;
        info.q0 = seg.linear.q0;
    }
    else
    {
        info.blend_shape = seg.blend.shape;
        info.V = seg.blend.V;
        info.r = seg.blend.r;
        info.d_in = seg.blend.d_in;
        info.d_out = seg.blend.d_out;
        info.q_start = seg.blend.q_start;
    }
    return info;
}

} // namespace

std::shared_ptr<PlannedTrajectoryData> plan(
    const Eigen::MatrixXd& waypoints, const Limits& limits, const GenerationOptions& options,
    BlendShape effective_blend_shape, bool use_scurve, double v_start, double v_end
)
{
    const Eigen::Index N = waypoints.rows();
    const Eigen::Index D = waypoints.cols();

    if (N < 2)
    {
        throw ValidationError("Need at least 2 waypoints.");
    }
    if (!waypoints.allFinite())
    {
        throw ValidationError("Waypoints must be finite.");
    }
    if (limits.v_max.size() != D || limits.a_max.size() != D)
    {
        throw ValidationError("Limits dimension does not match waypoint dimension.");
    }
    if (use_scurve)
    {
        if (!limits.j_max.has_value())
        {
            throw ValidationError("S-curve profile requires j_max in limits.");
        }
        if (limits.j_max->size() != D)
        {
            throw ValidationError("j_max dimension does not match waypoint dimension.");
        }
    }
    if (options.corner_handling != CornerHandling::STRICT_CORNERS &&
        options.corner_handling != CornerHandling::BLEND_ALL &&
        options.corner_handling != CornerHandling::BLEND_WHERE_FEASIBLE &&
        options.corner_handling != CornerHandling::BLEND_WHERE_FASTER)
    {
        throw ValidationError("Unknown corner_handling value.");
    }

    // Per-corner radii size check (1 broadcasts, else must match N - 2).
    const Eigen::Index n_interior = std::max<Eigen::Index>(N - 2, 0);
    if (options.blend_radii.has_value())
    {
        const Eigen::Index sz = options.blend_radii->size();
        if (sz != 1 && sz != n_interior)
        {
            std::ostringstream oss;
            oss << "blend_radii must have size 1 or N-2 = " << n_interior << "; got size " << sz << ".";
            throw ValidationError(oss.str());
        }
    }
    auto radius_for_corner = [&](Eigen::Index k) -> double
    {
        if (options.blend_radii.has_value())
        {
            const auto& v = *options.blend_radii;
            return v.size() == 1 ? v(0) : v(k - 1);
        }
        return options.blend_radius;
    };

    for (Eigen::Index k = 0; k < N - 1; ++k)
    {
        if ((waypoints.row(k + 1) - waypoints.row(k)).norm() < 1e-12)
        {
            throw ValidationError("Duplicate consecutive waypoints (zero-length segment).");
        }
    }

    // Collinear waypoints are merged into one straight leg, so the profile
    // does not have to pass them at a junction. Vertex k of the reduced path
    // joins leg k - 1 to leg k.
    // A merged waypoint's miss counts as a corner deviation, so it must also
    // respect the requested bound.
    const std::vector<Leg> legs =
        mergeCollinear(waypoints, options.max_corner_deviation.value_or(std::numeric_limits<double>::infinity()));
    const auto M = static_cast<Eigen::Index>(legs.size()) + 1;
    std::vector<Eigen::Index> vertex(static_cast<std::size_t>(M));
    for (Eigen::Index m = 0; m < M - 1; ++m)
        vertex[m] = legs[m].start;
    vertex.back() = N - 1;

    SpeedProblem problem;
    problem.scurve = use_scurve;
    problem.v_start = v_start;
    problem.v_end = v_end;
    problem.length.resize(M - 1);
    problem.sd_max.resize(M - 1);
    problem.sdd_max.resize(M - 1);
    problem.sddd_max.resize(use_scurve ? (M - 1) : 0);
    Eigen::MatrixXd u_dir(M - 1, D);
    for (Eigen::Index m = 0; m < M - 1; ++m)
    {
        problem.length(m) = legs[m].length;
        u_dir.row(m) = legs[m].dir.transpose();
        Eigen::VectorXd abs_dir = legs[m].dir.cwiseAbs().cwiseMax(1e-12);
        problem.sd_max(m) = (limits.v_max.array() / abs_dir.array()).minCoeff();
        problem.sdd_max(m) = (limits.a_max.array() / abs_dir.array()).minCoeff();
        if (use_scurve)
        {
            problem.sddd_max(m) = (limits.j_max->array() / abs_dir.array()).minCoeff();
        }
    }
    const Eigen::VectorXd& L_full = problem.length;
    const Eigen::VectorXd& sd_max = problem.sd_max;
    const Eigen::VectorXd& sdd_max = problem.sdd_max;
    const Eigen::VectorXd& sddd_max = problem.sddd_max;

    if (!(v_start >= 0.0) || !(v_end >= 0.0))
    {
        throw ValidationError("Boundary speeds must be finite and >= 0.");
    }
    if (v_start > sd_max(0) + 1e-12)
    {
        throw ValidationError("v_start exceeds the velocity limit along the first segment.");
    }
    if (v_end > sd_max(M - 2) + 1e-12)
    {
        throw ValidationError("v_end exceeds the velocity limit along the last segment.");
    }

    // Classify the corners of the reduced path.
    SpeedPlan speeds;
    speeds.type.assign(static_cast<std::size_t>(M), CornerType::STOP);
    speeds.type.front() = CornerType::ENDPOINT;
    speeds.type.back() = CornerType::ENDPOINT;
    std::vector<CornerType>& vertex_type = speeds.type;
    problem.blend_r.assign(static_cast<std::size_t>(M), 0.0);
    problem.blend_v_cap.assign(static_cast<std::size_t>(M), 0.0);

    for (Eigen::Index k = 1; k < M - 1; ++k)
    {
        const Eigen::Index w = vertex[k];
        if (options.corner_handling == CornerHandling::STRICT_CORNERS)
        {
            continue;
        }
        Eigen::VectorXd u_prev = u_dir.row(k - 1).transpose();
        Eigen::VectorXd u_curr = u_dir.row(k).transpose();
        if (u_prev.dot(u_curr) <= kReversalCosThreshold)
        {
            if (options.corner_handling == CornerHandling::BLEND_ALL)
            {
                std::ostringstream oss;
                oss << "Blend at waypoint " << w << " is a near-reversal; cannot blend through a direction reversal.";
                throw ValidationError(oss.str());
            }
            continue;
        }
        // Neighbouring blends must not overlap, and a blend must not start
        // before a merged collinear waypoint, which would then be cut.
        double max_r_geom = std::min(L_full(k - 1), L_full(k)) / 2.0;
        if (!legs[k - 1].inner_s.empty())
            max_r_geom = std::min(max_r_geom, L_full(k - 1) - legs[k - 1].inner_s.back());
        if (!legs[k].inner_s.empty())
            max_r_geom = std::min(max_r_geom, legs[k].inner_s.front());
        double r_requested = radius_for_corner(w);
        if (options.max_corner_deviation.has_value())
        {
            // Inverse of the corner deviation formula used for corner_deviations below.
            r_requested =
                *options.max_corner_deviation / (deviationFactor(effective_blend_shape) * (u_curr - u_prev).norm());
        }
        double r = std::min(r_requested, max_r_geom);
        if (r > 1e-9)
        {
            double V_max = blendVCap(
                u_prev, u_curr, r, limits.v_max, limits.a_max,
                use_scurve ? limits.j_max : std::optional<Eigen::VectorXd>{}, effective_blend_shape
            );
            if (V_max > 1e-12)
            {
                vertex_type[k] = CornerType::BLEND;
                problem.blend_r[k] = r;
                problem.blend_v_cap[k] = V_max;
            }
            else if (options.corner_handling == CornerHandling::BLEND_ALL)
            {
                std::ostringstream oss;
                oss << "Blend at waypoint " << w << " infeasible (V_max <= 0).";
                throw ValidationError(oss.str());
            }
        }
        else if (options.corner_handling == CornerHandling::BLEND_ALL)
        {
            std::ostringstream oss;
            oss << "Blend at waypoint " << w << ": blend radius " << r << " is too small to blend (requested "
                << r_requested << ", limited to " << max_r_geom
                << " by half the shorter adjacent segment and any collinear waypoints).";
            throw ValidationError(oss.str());
        }
    }

    // Blends whose speed the passes force to zero revert to stops, and so do
    // blends next to an end that leave too little room to reach its boundary
    // speed: a stop gives the leg its full length instead.
    while (true)
    {
        solveSpeeds(problem, speeds);
        std::vector<Eigen::Index> to_stop;
        for (Eigen::Index k = 1; k < M - 1; ++k)
        {
            if (vertex_type[k] == CornerType::BLEND && speeds.U(k) < 1e-6)
            {
                if (options.corner_handling == CornerHandling::BLEND_ALL)
                {
                    std::ostringstream oss;
                    oss << "Blend at waypoint " << vertex[k]
                        << " collapsed (V forced to 0). Use a smaller blend radius, or BLEND_WHERE_FEASIBLE to stop "
                           "there instead.";
                    throw ValidationError(oss.str());
                }
                to_stop.push_back(k);
            }
        }
        if (options.corner_handling != CornerHandling::BLEND_ALL && M > 2)
        {
            if (speeds.U(0) + 1e-9 < v_start && vertex_type[1] == CornerType::BLEND)
                to_stop.push_back(1);
            if (speeds.U(M - 1) + 1e-9 < v_end && vertex_type[M - 2] == CornerType::BLEND)
                to_stop.push_back(M - 2);
        }
        if (to_stop.empty())
            break;
        for (Eigen::Index k : to_stop)
        {
            vertex_type[k] = CornerType::STOP;
            problem.blend_r[k] = 0.0;
            problem.blend_v_cap[k] = 0.0;
        }
    }

    // The passes only ever lower junction speeds, so a boundary speed that
    // came out below the request cannot be realised with these corners.
    if (speeds.U(0) + 1e-9 < v_start)
    {
        throw ValidationError(
            "v_start infeasible: cannot slow down to the next junction speed within the first segment."
        );
    }
    if (speeds.U(M - 1) + 1e-9 < v_end)
    {
        throw ValidationError("v_end infeasible: cannot reach the requested end speed within the last segment.");
    }

    // A blend can be slower than stopping: its speed cap may force the
    // neighbouring legs to slow down more than a stop would cost.
    if (options.corner_handling == CornerHandling::BLEND_WHERE_FASTER)
    {
        std::vector<Eigen::Index> candidates;
        for (Eigen::Index k = 1; k < M - 1; ++k)
        {
            if (vertex_type[k] == CornerType::BLEND)
                candidates.push_back(k);
        }
        chooseFastestCorners(problem, speeds, candidates);
    }
    const Eigen::VectorXd& U = speeds.U;
    const Eigen::VectorXd& L_eff = speeds.L_eff;
    std::vector<double> blend_r(static_cast<std::size_t>(M), 0.0);
    for (Eigen::Index k = 1; k < M - 1; ++k)
    {
        if (vertex_type[k] == CornerType::BLEND)
            blend_r[k] = problem.blend_r[k];
    }

    // Build segment list
    auto out = std::make_shared<PlannedTrajectoryData>();
    out->dim = static_cast<std::size_t>(D);
    out->blend_shape = effective_blend_shape;
    out->corner_handling = options.corner_handling;
    out->is_scurve = use_scurve;
    out->waypoints = waypoints;

    // Per-waypoint outputs. Merged collinear waypoints are PASS corners.
    std::vector<CornerType> corner_type(static_cast<std::size_t>(N), CornerType::PASS);
    std::vector<double> waypoint_blend_r(static_cast<std::size_t>(N), 0.0);
    Eigen::VectorXd junction_speed(N);
    // Passage times (blend corners: time of closest approach, i.e. the blend
    // midpoint) and the distance from each waypoint to the path at that time.
    std::vector<double> waypoint_time(static_cast<std::size_t>(N), 0.0);
    std::vector<double> corner_deviation(static_cast<std::size_t>(N), 0.0);
    for (Eigen::Index k = 0; k < M; ++k)
    {
        corner_type[vertex[k]] = vertex_type[k];
        waypoint_blend_r[vertex[k]] = blend_r[k];
        junction_speed(vertex[k]) = U(k);
    }
    double t_cursor = 0.0;

    for (Eigen::Index k = 0; k < M - 1; ++k)
    {
        Eigen::VectorXd start_pos = waypoints.row(vertex[k]).transpose();
        if (vertex_type[k] == CornerType::BLEND)
        {
            start_pos += blend_r[k] * u_dir.row(k).transpose();
        }

        double L = L_eff(k);
        if (L > 1e-12)
        {
            double v0 = U(k);
            double v1 = U(k + 1);
            Segment seg;
            seg.type = SegmentType::LINEAR;
            seg.linear.u_dir = u_dir.row(k).transpose();
            seg.linear.q0 = start_pos;
            seg.linear.v0 = v0;
            seg.linear.length = L;
            seg.linear.is_scurve = use_scurve;
            if (use_scurve)
            {
                seg.linear.scurve_phases = scurvePhases(v0, v1, sd_max(k), sdd_max(k), sddd_max(k), L);
                double T = 0.0;
                for (const auto& p : seg.linear.scurve_phases)
                    T += p.duration;
                seg.linear.duration = T;
                seg.duration = T;
            }
            else
            {
                seg.linear.trap_phases = trapPhases(v0, v1, sd_max(k), sdd_max(k), L);
                double T = 0.0;
                for (const auto& p : seg.linear.trap_phases)
                    T += p.duration;
                seg.linear.duration = T;
                seg.duration = T;
            }

            const Leg& leg = legs[k];
            for (std::size_t i = 0; i < leg.inner_s.size(); ++i)
            {
                const auto w = static_cast<std::size_t>(leg.start) + i + 1;
                const double s_local = std::clamp(leg.inner_s[i] - blend_r[k], 0.0, L);
                const double t_local = timeAtArc(seg.linear, s_local);
                waypoint_time[w] = t_cursor + t_local;
                corner_deviation[w] = leg.inner_miss[i];
                junction_speed(static_cast<Eigen::Index>(w)) =
                    use_scurve ? sampleScurve(t_local, v0, seg.linear.scurve_phases).sd
                               : sampleTrap(t_local, v0, seg.linear.trap_phases).sd;
            }
            t_cursor += seg.duration;
            out->segments.push_back(std::move(seg));
        }
        else
        {
            // No straight part: a merged waypoint sits where two blends meet.
            for (std::size_t i = 0; i < legs[k].inner_s.size(); ++i)
            {
                const auto w = static_cast<std::size_t>(legs[k].start) + i + 1;
                waypoint_time[w] = t_cursor;
                corner_deviation[w] = legs[k].inner_miss[i];
                junction_speed(static_cast<Eigen::Index>(w)) = U(k);
            }
        }

        Eigen::Index kk = k + 1;
        const auto w_end = static_cast<std::size_t>(vertex[kk]);
        if (kk < M - 1 && vertex_type[kk] == CornerType::BLEND)
        {
            double r_b = blend_r[kk];
            double V = U(kk);
            double T_b = 2.0 * r_b / V;
            Segment seg;
            seg.type = SegmentType::BLEND;
            seg.blend.shape = effective_blend_shape;
            seg.blend.duration = T_b;
            seg.blend.V = V;
            seg.blend.r = r_b;
            seg.blend.d_in = u_dir.row(k).transpose();
            seg.blend.d_out = u_dir.row(k + 1).transpose();
            seg.blend.q_start = waypoints.row(vertex[kk]).transpose() - r_b * u_dir.row(k).transpose();
            seg.duration = T_b;
            out->segments.push_back(std::move(seg));

            // Deviation at the blend midpoint: (r/4)|d_out - d_in| for a
            // parabolic blend, (3r/16)|d_out - d_in| for a Hermite blend.
            const double delta_norm = (u_dir.row(kk) - u_dir.row(k)).norm();
            corner_deviation[w_end] = deviationFactor(effective_blend_shape) * r_b * delta_norm;
            waypoint_time[w_end] = t_cursor + T_b / 2.0;
            t_cursor += T_b;
        }
        else
        {
            waypoint_time[w_end] = t_cursor;
        }
    }
    out->junction_speeds = std::move(junction_speed);
    out->blend_radii = std::move(waypoint_blend_r);
    out->corner_types = std::move(corner_type);
    out->waypoint_times = std::move(waypoint_time);
    out->corner_deviations = std::move(corner_deviation);

    out->cumulative_t.assign(out->segments.size() + 1, 0.0);
    for (std::size_t i = 0; i < out->segments.size(); ++i)
    {
        out->cumulative_t[i + 1] = out->cumulative_t[i] + out->segments[i].duration;
    }
    out->total_duration = out->cumulative_t.back();

    // Finite inputs can still overflow or underflow on the way here. A
    // segment too short for its speed loses every phase and has no duration.
    bool valid = !out->segments.empty() && std::isfinite(out->total_duration);
    for (const auto& seg : out->segments)
    {
        valid = valid && seg.duration > 0.0;
    }
    if (!valid)
    {
        throw ValidationError("Generation produced an empty trajectory or a segment with a zero or non-finite "
                              "duration; check the scale of the waypoints and limits.");
    }

    out->segment_infos.reserve(out->segments.size());
    for (const auto& seg : out->segments)
    {
        out->segment_infos.push_back(extractInfo(seg));
    }

    return out;
}

} // namespace justblend::internal
