// 01_basic_2d.cpp
//
// Smallest possible justblend example: a 2D path through four points using a
// trapezoidal-velocity profile with no corner blending.

#include <Eigen/Core>
#include <cstdio>

#include <justblend/justblend.hpp>

int main()
{
    Eigen::MatrixXd waypoints(4, 2);
    waypoints << 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0;

    justblend::Limits limits;
    limits.v_max = (Eigen::VectorXd(2) << 1.0, 1.0).finished();
    limits.a_max = (Eigen::VectorXd(2) << 2.0, 2.0).finished();

    justblend::TrapezoidalTrajectoryGenerator gen(2);
    gen.setLimits(limits);
    gen.setOptions({});

    auto traj = gen.generate(waypoints);
    std::printf("duration: %.6f s\n", traj.duration());
    std::printf("segments: %zu\n", traj.numSegments());

    for (double t = 0.0; t <= traj.duration() + 1e-9; t += 0.25)
    {
        auto s = traj.sample(t);
        std::printf("  t=%.3f  q=(%.4f, %.4f)  |qd|=%.4f\n", s.t, s.q(0), s.q(1), s.qd.norm());
    }
    return 0;
}
