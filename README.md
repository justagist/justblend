# justblend

Closed-form N-dimensional trajectory generator with smooth corner blends.
General-purpose: works for joint trajectories, Cartesian paths, CNC tool
paths, animation curves -- anywhere you have a list of points in some
D-dimensional space and want a velocity/acceleration/jerk-limited smooth
trajectory through them.

Two per-segment profiles:

- **Trapezoidal** -- bang-bang acceleration, fastest rest-to-rest motion
  under velocity/acceleration limits.
- **S-curve** -- 7-phase jerk-limited motion when a jerk limit is given.

Three corner-handling modes (pass-through, blending, hybrid) and two blend
shapes (parabolic, cubic-Hermite). The Hermite blend has zero acceleration
at its boundaries, so it matches an S-curve linear segment's endpoint
`qdd = 0` and respects a jerk limit through the corner.

The output `Trajectory` object is lazy: it owns a list of analytic segments
and can be sampled at any time `t ∈ [0, duration]` without precomputing a
dense buffer.

## Getting started -- C++

```cpp
#include <justblend/justblend.hpp>

Eigen::MatrixXd waypoints(4, 2);
waypoints << 0.0, 0.0,
             1.0, 0.5,
             1.5,-0.2,
             0.0, 0.0;

justblend::Limits limits;
limits.v_max = (Eigen::VectorXd(2) << 1.5, 1.2).finished();
limits.a_max = (Eigen::VectorXd(2) << 3.0, 2.5).finished();
limits.j_max = (Eigen::VectorXd(2) << 20.0, 18.0).finished();

justblend::GenerationOptions opts;
opts.blend_radius = 0.15;
opts.corner_handling = justblend::CornerHandling::Hybrid;

justblend::SCurveTrajectoryGenerator gen(2);
gen.setLimits(limits);
gen.setOptions(opts);

auto traj = gen.generate(waypoints);
auto s = traj.sample(0.5);          // single sample
auto dense = traj.samples(0.002);   // uniform dt sweep
```

## Getting started -- Python

```python
import numpy as np
import justblend as jb

waypoints = np.array([[0.0, 0.0], [1.0, 0.5], [1.5, -0.2], [0.0, 0.0]])
limits = jb.Limits(
    v_max=np.array([1.5, 1.2]),
    a_max=np.array([3.0, 2.5]),
    j_max=np.array([20.0, 18.0]),
)
opts = jb.GenerationOptions(
    blend_radius=0.15,
    corner_handling=jb.CornerHandling.HYBRID,
)

gen = jb.SCurveTrajectoryGenerator(dim=2)
gen.set_limits(limits)
gen.set_options(opts)
traj = gen.generate(waypoints)

s = traj.sample(0.5)
dense = traj.samples(dt=0.002)
arbitrary = traj.samples(np.linspace(0, traj.duration(), 1000))
```

## Install

```bash
pip install .
```

Requires CMake ≥ 3.18, a C++17 compiler, and Eigen 3.3+. `pip install`
fetches `scikit-build-core` and `pybind11` automatically.

To build the C++ library and tests directly:

```bash
cmake -S . -B build -DJUSTBLEND_BUILD_TESTS=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

## License

MIT -- see [LICENSE](LICENSE).
