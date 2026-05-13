// 02_robot_joints_scurve.cpp
//
// Joint-space S-curve trajectory through several knots with Hermite corner
// blends. The same code works for any D-dimensional waypoint list -- robot
// joints just happen to be one common case.

#include <Eigen/Core>
#include <cstdio>

#include <justblend/justblend.hpp>

int main() {
    Eigen::MatrixXd waypoints(5, 3);
    waypoints << 0.0, 0.0, 0.0,
                 1.0, 0.5, -0.3,
                 1.5, -0.2, 0.4,
                 0.5, 0.8, 1.0,
                 0.0, 0.0, 0.0;

    justblend::Limits limits;
    limits.v_max = (Eigen::VectorXd(3) << 1.5, 1.2, 1.0).finished();
    limits.a_max = (Eigen::VectorXd(3) << 3.0, 2.5, 2.0).finished();
    limits.j_max = (Eigen::VectorXd(3) << 20.0, 18.0, 15.0).finished();

    justblend::GenerationOptions opts;
    opts.blend_radius = 0.15;
    opts.corner_handling = justblend::CornerHandling::Hybrid;

    justblend::SCurveTrajectoryGenerator gen(3);
    gen.setLimits(limits);
    gen.setOptions(opts);

    auto traj = gen.generate(waypoints);

    std::printf("duration: %.6f s\n", traj.duration());
    std::printf("segments: %zu\n", traj.numSegments());

    auto dense = traj.samples(0.01);
    double max_v = dense.qd.cwiseAbs().maxCoeff();
    double max_a = dense.qdd.cwiseAbs().maxCoeff();
    std::printf("max|qd| = %.4f  max|qdd| = %.4f\n", max_v, max_a);
    return 0;
}
