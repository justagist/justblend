"""Smallest possible justblend example: a 2D path with trapezoidal velocity."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from _plot import plot_trajectory  # noqa: E402

import justblend as jb  # noqa: E402


def main() -> None:
    waypoints = np.array(
        [
            [0.0, 0.0],
            [1.0, 0.0],
            [1.0, 1.0],
            [0.0, 1.0],
        ]
    )

    v_max = np.array([1.0, 1.0])
    a_max = np.array([2.0, 2.0])
    limits = jb.Limits(v_max=v_max, a_max=a_max)

    gen = jb.TrapezoidalTrajectoryGenerator(dim=2)
    gen.set_limits(limits)
    gen.set_options(jb.GenerationOptions())

    traj = gen.generate(waypoints)
    print(f"duration: {traj.duration():.6f} s")
    print(f"segments: {traj.num_segments()}")

    for t in np.arange(0.0, traj.duration() + 1e-9, 0.25):
        s = traj.sample(float(t))
        print(f"  t={s.t:.3f}  q=({s.q[0]:.4f}, {s.q[1]:.4f})  |qd|={np.linalg.norm(s.qd):.4f}")

    plot_trajectory(traj, v_max, a_max, title="basic 2D (trapezoidal)")
    plt.show()


if __name__ == "__main__":
    main()
