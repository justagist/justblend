// 03_blend_comparison.cpp
//
// Run the same waypoints with both blend shapes and print summary statistics
// for the resulting trajectory. Hermite blends are slightly slower (smaller
// allowed corner speed because the peak in-blend acceleration is 1.5x
// parabolic for the same V and r) but have zero acceleration at the blend
// boundaries -- so paired with an S-curve linear profile there is no
// acceleration step at the linear/blend junctions.

#include <Eigen/Core>
#include <cstdio>

#include <justblend/justblend.hpp>

namespace
{

void runOne(justblend::BlendShape shape)
{
    Eigen::MatrixXd waypoints(5, 3);
    waypoints << 0.0, 0.0, 0.0, 1.0, 0.5, -0.3, 1.5, -0.2, 0.4, 0.5, 0.8, 1.0, 0.0, 0.0, 0.0;

    justblend::Limits limits;
    limits.v_max = (Eigen::VectorXd(3) << 1.5, 1.2, 1.0).finished();
    limits.a_max = (Eigen::VectorXd(3) << 3.0, 2.5, 2.0).finished();
    limits.j_max = (Eigen::VectorXd(3) << 20.0, 18.0, 15.0).finished();

    justblend::GenerationOptions opts;
    opts.blend_radius = 0.15;
    opts.corner_handling = justblend::CornerHandling::Hybrid;
    opts.blend_shape = shape;

    justblend::SCurveTrajectoryGenerator gen(3);
    gen.setLimits(limits);
    gen.setOptions(opts);
    auto traj = gen.generate(waypoints);

    auto dense = traj.samples(0.002);
    double max_step = 0.0;
    for (Eigen::Index i = 1; i < dense.qdd.rows(); ++i)
    {
        max_step = std::max(max_step, (dense.qdd.row(i) - dense.qdd.row(i - 1)).cwiseAbs().maxCoeff());
    }
    const char* name = shape == justblend::BlendShape::Parabolic ? "parabolic" : "hermite";
    std::printf("%-9s  duration=%.4f s  max |Δqdd| between adjacent dt=2ms = %.4f\n", name, traj.duration(), max_step);
}

} // namespace

int main()
{
    runOne(justblend::BlendShape::Parabolic);
    runOne(justblend::BlendShape::Hermite);
    return 0;
}
