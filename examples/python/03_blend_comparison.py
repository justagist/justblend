"""Compare parabolic vs Hermite blend shapes on the same S-curve waypoints."""

import numpy as np

import justblend as jb


def run_one(shape: jb.BlendShape) -> None:
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

    name = "parabolic" if shape == jb.BlendShape.PARABOLIC else "hermite"
    print(f"{name:<9}  duration={traj.duration():.4f} s  max |Δqdd| between adjacent dt=2ms = {qdd_steps:.4f}")


def main() -> None:
    run_one(jb.BlendShape.PARABOLIC)
    run_one(jb.BlendShape.HERMITE)


if __name__ == "__main__":
    main()
