"""Smallest possible justblend example: a 2D path with trapezoidal velocity."""

import numpy as np

import justblend as jb


def main() -> None:
    waypoints = np.array(
        [
            [0.0, 0.0],
            [1.0, 0.0],
            [1.0, 1.0],
            [0.0, 1.0],
        ]
    )

    limits = jb.Limits(v_max=np.array([1.0, 1.0]), a_max=np.array([2.0, 2.0]))

    gen = jb.TrapezoidalTrajectoryGenerator(dim=2)
    gen.set_limits(limits)
    gen.set_options(jb.GenerationOptions())

    traj = gen.generate(waypoints)
    print(f"duration: {traj.duration():.6f} s")
    print(f"segments: {traj.num_segments()}")

    for t in np.arange(0.0, traj.duration() + 1e-9, 0.25):
        s = traj.sample(float(t))
        print(f"  t={s.t:.3f}  q=({s.q[0]:.4f}, {s.q[1]:.4f})  |qd|={np.linalg.norm(s.qd):.4f}")


if __name__ == "__main__":
    main()
