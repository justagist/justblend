// Built with EIGEN_RUNTIME_NO_MALLOC and assertions enabled (see
// CMakeLists.txt), so any heap allocation by Eigen while allocation is
// disallowed aborts the test.

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <vector>

#include "justblend/justblend.hpp"

using namespace justblend;

namespace
{

std::vector<Trajectory> blendedTrajectories()
{
    Eigen::MatrixXd W(5, 3);
    W << 0.0, 0.0, 0.0, 1.0, 0.5, -0.3, 1.5, -0.2, 0.4, 0.5, 0.8, 1.0, 0.0, 0.0, 0.0;
    Limits L;
    L.v_max = (Eigen::VectorXd(3) << 1.5, 1.2, 1.0).finished();
    L.a_max = (Eigen::VectorXd(3) << 3.0, 2.5, 2.0).finished();
    L.j_max = (Eigen::VectorXd(3) << 20.0, 18.0, 15.0).finished();

    std::vector<Trajectory> out;
    for (BlendShape shape : {BlendShape::PARABOLIC, BlendShape::HERMITE})
    {
        GenerationOptions opts;
        opts.blend_radius = 0.15;
        opts.corner_handling = CornerHandling::BLEND_WHERE_FASTER;
        opts.blend_shape = shape;

        SCurveTrajectoryGenerator scurve(3);
        scurve.setLimits(L);
        scurve.setOptions(opts);
        out.push_back(scurve.generate(W));

        TrapezoidalTrajectoryGenerator trap(3);
        trap.setLimits(L);
        trap.setOptions(opts);
        out.push_back(trap.generate(W));
    }
    return out;
}

} // namespace

TEST(Realtime, SampleIntoDoesNotAllocate)
{
    for (const auto& traj : blendedTrajectories())
    {
        ASSERT_GT(traj.numSegments(), 1u);
        const auto& boundaries = traj.segmentStartTimes();
        Sample s;
        traj.sampleInto(0.0, s);
        Eigen::VectorXd q(3), qd(3), qdd(3), qddd(3);

        Eigen::internal::set_is_malloc_allowed(false);
        for (int i = -1; i <= 1001; ++i)
        {
            const double t = traj.duration() * i / 1000.0;
            traj.sampleInto(t, s);
            traj.sampleInto(t, q, qd, qdd, qddd);
        }
        for (double t : boundaries)
        {
            traj.sampleInto(t, s);
            traj.sampleInto(t, q, qd, qdd, qddd);
        }
        Eigen::internal::set_is_malloc_allowed(true);
    }
}
