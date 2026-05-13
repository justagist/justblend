"""S-curve trajectory through several knots with Hermite corner blends."""

import numpy as np

import justblend as jb


def main() -> None:
    waypoints = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.5, -0.3],
            [1.5, -0.2, 0.4],
            [0.5, 0.8, 1.0],
            [0.0, 0.0, 0.0],
        ]
    )

    limits = jb.Limits(
        v_max=np.array([1.5, 1.2, 1.0]),
        a_max=np.array([3.0, 2.5, 2.0]),
        j_max=np.array([20.0, 18.0, 15.0]),
    )

    gen = jb.SCurveTrajectoryGenerator(dim=3)
    gen.set_limits(limits)
    gen.set_options(
        jb.GenerationOptions(blend_radius=0.15, corner_handling=jb.CornerHandling.HYBRID)
    )

    traj = gen.generate(waypoints)
    print(f"duration: {traj.duration():.6f} s")
    print(f"segments: {traj.num_segments()}")

    dense = traj.samples(dt=0.01)
    print(f"max|qd| = {np.max(np.abs(dense.qd)):.4f}")
    print(f"max|qdd| = {np.max(np.abs(dense.qdd)):.4f}")


if __name__ == "__main__":
    main()
