#ifndef JUSTBLEND_INTERNAL_SEGMENT_HPP
#define JUSTBLEND_INTERNAL_SEGMENT_HPP

#include <Eigen/Core>
#include <vector>

#include "justblend/types.hpp"

namespace justblend::internal
{

struct TrapPhase
{
    double duration;
    double sdd;
};

struct ScurvePhase
{
    double duration;
    double jerk;
};

struct LinearSegmentData
{
    Eigen::VectorXd u_dir;
    Eigen::VectorXd q0;
    double v0 = 0.0;
    double length = 0.0;
    double duration = 0.0;
    bool is_scurve = false;
    std::vector<TrapPhase> trap_phases;
    std::vector<ScurvePhase> scurve_phases;
};

struct BlendSegmentData
{
    BlendShape shape = BlendShape::Parabolic;
    double duration = 0.0;
    double V = 0.0;
    double r = 0.0;
    Eigen::VectorXd d_in;
    Eigen::VectorXd d_out;
    Eigen::VectorXd q_start;
};

struct Segment
{
    SegmentType type = SegmentType::Linear;
    double duration = 0.0;
    LinearSegmentData linear;
    BlendSegmentData blend;
};

struct PlannedTrajectoryData
{
    std::vector<Segment> segments;
    std::vector<double> cumulative_t; // size segments.size()+1; cumulative_t[0] = 0
    Eigen::VectorXd junction_speeds;
    std::vector<double> blend_radii;
    std::vector<CornerType> corner_types;
    BlendShape blend_shape = BlendShape::Parabolic;
    CornerHandling corner_handling = CornerHandling::StrictCorners;
    std::size_t dim = 0;
    double total_duration = 0.0;
    Eigen::MatrixXd waypoints;
    bool is_scurve = false;

    // Pre-extracted public-facing segment info (matches segments vector).
    std::vector<SegmentInfo> segment_infos;
};

} // namespace justblend::internal

#endif // JUSTBLEND_INTERNAL_SEGMENT_HPP
