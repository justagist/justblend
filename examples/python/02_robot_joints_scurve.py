"""S-curve trajectory through several knots with Hermite corner blends.

Cycles through all three corner-handling modes so you can compare them.
"""

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
            [0.0, 0.0, 0.0],
            [1.0, 0.5, -0.3],
            [1.5, -0.2, 0.4],
            [0.5, 0.8, 1.0],
            [0.0, 0.0, 0.0],
        ]
    )
    v_max = np.array([1.5, 1.2, 1.0])
    a_max = np.array([3.0, 2.5, 2.0])
    j_max = np.array([20.0, 18.0, 15.0])
    limits = jb.Limits(v_max=v_max, a_max=a_max, j_max=j_max)

    modes = [
        ("strict_corners", jb.CornerHandling.STRICT_CORNERS),
        ("hybrid", jb.CornerHandling.HYBRID),
        ("use_blending", jb.CornerHandling.USE_BLENDING),
    ]

    for name, ch in modes:
        gen = jb.SCurveTrajectoryGenerator(dim=3)
        gen.set_limits(limits)
        gen.set_options(jb.GenerationOptions(blend_radius=0.15, corner_handling=ch))
        traj = gen.generate(waypoints)

        dense = traj.samples(dt=0.005)
        print(f"\n=== corner_handling = {name} (S-curve, hermite blend) ===")
        print(f"  duration       : {traj.duration():.3f} s")
        print(f"  segments       : {traj.num_segments()}")
        print(f"  junction speeds: {np.round(traj.junction_speeds(), 3)}")
        print(f"  max |qd|       : {np.round(np.max(np.abs(dense.qd), axis=0), 3)}  (v_max {v_max})")
        print(f"  max |qdd|      : {np.round(np.max(np.abs(dense.qdd), axis=0), 3)}  (a_max {a_max})")

        plot_trajectory(traj, v_max, a_max, j_max, title=f"corner_handling={name}", waypoints=waypoints)

    plt.show()


if __name__ == "__main__":
    main()
