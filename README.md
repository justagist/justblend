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

## Corner semantics

- `blend_radius` is the cut-back distance along each adjacent leg of the
  corner, not an arc radius: a blend replaces the final `r` of the incoming
  segment and the first `r` of the outgoing one.
- Blended corners cut the corner -- the trajectory does not pass through a
  blended interior waypoint. Use `StrictCorners` (or a per-corner radius via
  `blend_radii`) where exact pass-through matters.
- Corners within ~2.6 degrees of a full reversal (the path doubles back on
  itself) cannot be blended: `Hybrid` falls back to a full stop at the
  waypoint, `UseBlending` raises a `ValidationError`.
- An S-curve trajectory is jerk-limited end to end only with the default
  Hermite blends. Forcing `blend_shape = Parabolic` on the S-curve generator
  keeps the velocity and acceleration limits but introduces acceleration
  steps at blend boundaries, i.e. unbounded jerk there.

All input-validation failures throw `justblend::ValidationError`, which the
Python bindings raise as a `ValueError` subclass.

## Boundary speeds, timing and deviation queries

`generate()` accepts optional tangential boundary speeds along the first and
last segment direction; infeasible requests (too fast for the limits or for
the available distance) raise a `ValidationError`:

```cpp
auto traj = gen.generate(waypoints, /*v_start=*/0.3, /*v_end=*/0.0);
```

```python
traj = gen.generate(waypoints, v_start=0.3, v_end=0.0)
```

`GenerationOptions` carries `velocity_scaling_factor` and
`acceleration_scaling_factor` in `(0, 1]` (matching MoveIt's semantics):
they scale `v_max` / `a_max` at generation time; `j_max` is never scaled.

The `Trajectory` exposes timing and corner-cut metadata:

- `segment_start_times()` -- start time of each segment plus the total
  duration as the last entry.
- `waypoint_times()` -- passage time per waypoint; for blended corners this
  is the time of closest approach (the blend midpoint).
- `corner_deviations()` / `max_corner_deviation()` -- closed-form distance
  between each blended corner's waypoint and the blend path
  (`(r/4)·|d_out - d_in|` for parabolic, `(3r/16)·|d_out - d_in|` for
  Hermite blends; zero for non-blend corners). Useful to bound how far the
  blended trajectory strays from the original polyline.

## Install -- C++

Requirements: CMake ≥ 3.18, a C++17 compiler, and Eigen 3.3+ (any package
that exposes `find_package(Eigen3 NO_MODULE)` -- the Debian/Ubuntu
`libeigen3-dev` package, vcpkg, conan, brew, or a manual install all work).

### Build and install to a prefix

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    -DJUSTBLEND_BUILD_TESTS=OFF \
    -DJUSTBLEND_BUILD_EXAMPLES=OFF \
    -DJUSTBLEND_BUILD_PYTHON_BINDINGS=OFF
cmake --build build --parallel
cmake --install build --prefix /your/install/prefix    # omit --prefix for /usr/local
```

This installs:

- `<prefix>/include/justblend/*.hpp`
- `<prefix>/lib/libjustblend.a`
- `<prefix>/lib/cmake/justblend/justblend{Config,ConfigVersion,Targets}.cmake`

### Use from another CMake project

After installing, consume justblend with `find_package`:

```cmake
find_package(justblend 0.1.0 REQUIRED)

add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE justblend::justblend)
```

Then configure your project with `-DCMAKE_PREFIX_PATH=/your/install/prefix`
(or set `justblend_DIR=<prefix>/lib/cmake/justblend`).

You don't have to install: justblend also exports its targets from the
build tree, so pointing `CMAKE_PREFIX_PATH` at the build directory works
just as well.

To vendor justblend as a subdirectory (git submodule, FetchContent, or a
plain copy), drop `add_subdirectory(path/to/justblend)` into your
`CMakeLists.txt` -- the same `justblend::justblend` target becomes
available, and the test / example / Python-binding targets stay OFF by
default when consumed this way.

### Run tests and examples

```bash
cmake -S . -B build -DJUSTBLEND_BUILD_TESTS=ON -DJUSTBLEND_BUILD_EXAMPLES=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
./build/examples/cpp/example_basic_2d
```

## Install -- Python

```bash
pip install .
```

`pip install` fetches `scikit-build-core` and `pybind11` automatically and
drives the same CMake build under the hood -- no separate C++ install step
needed. Eigen 3.3+ must still be installed on the host.

## License

MIT -- see [LICENSE](LICENSE).
