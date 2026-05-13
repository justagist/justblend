"""Compare parabolic vs Hermite blend shapes on the same S-curve waypoints.

Hermite blends have zero acceleration at the blend boundaries (no accel step
against the S-curve linear segment's endpoint qdd=0), at the cost of 1.5x
peak in-blend acceleration for the same V and r. The plots show this as
smooth qdd and bounded qddd for Hermite vs spikes in qddd for parabolic.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from _plot import plot_trajectory  # noqa: E402

import justblend as jb  # noqa: E402


def run_one(shape: jb.BlendShape, name: str):
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

    opts = jb.GenerationOptions(
        blend_radius=0.15,
        corner_handling=jb.CornerHandling.HYBRID,
        blend_shape=shape,
    )

    gen = jb.SCurveTrajectoryGenerator(dim=3)
    gen.set_limits(limits)
    gen.set_options(opts)
    traj = gen.generate(waypoints)

    dense = traj.samples(dt=0.002)
    qdd_steps = np.max(np.abs(np.diff(dense.qdd, axis=0)))
    print(f"{name:<9}  duration={traj.duration():.4f} s  max |qdd step| over 2ms = {qdd_steps:.4f}")

    plot_trajectory(traj, v_max, a_max, j_max, dt=0.002, title=f"blend_shape={name}, scurve+hybrid", waypoints=waypoints)


def main() -> None:
    run_one(jb.BlendShape.PARABOLIC, "parabolic")
    run_one(jb.BlendShape.HERMITE, "hermite")

    plt.show()


if __name__ == "__main__":
    main()
