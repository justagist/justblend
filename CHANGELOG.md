# Changelog

Notable changes to justblend. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.3.0] - 2026-11-12

### Added

- `GenerationOptions::max_corner_deviation`: a corner tolerance that replaces
  the blend radius. Each corner uses the largest radius whose deviation stays
  within it, still clipped to half the shorter adjacent segment.
- Analytic jerk: `qddd` in `Sample` and `SamplesResult`. Impulses at
  acceleration steps are not represented.
- `Trajectory::sampleInto` (C++ only): allocation-free sampling into a
  reusable `Sample` or into caller-owned `Eigen::Ref` buffers, for real-time
  loops.

## [0.2.0] - 2026-11-11

### Fixed

- NaN and infinite values are rejected in limits, blend radii, waypoints,
  boundary speeds, sample steps and stretch durations. A NaN waypoint used to
  give an empty trajectory that crashed when sampled.
- NaN sample times are rejected. Infinite sample times are clamped to the
  trajectory ends, like any other out-of-range time.
- Generation raises `ValidationError` when the result would be empty or have a
  non-finite duration, for example when a segment length overflows.
- Python `TrajectoryGenerator.limits()` and `options()` return copies. Editing
  the returned object could bypass the validating setters.
- Single and batch sampling choose the segment the same way, so a sample
  exactly on a segment boundary no longer depends on the order of the
  requested times.
- `samples(dt)` rejects a step too small for the sample count to be
  represented, and includes the final time when `dt` far exceeds the duration.
- Generation raises `ValidationError` when a segment would have zero duration,
  for example a very short collinear segment crossed at high speed.
- `stretchedTo` rejects a target so large that the time scale overflows, and
  never speeds a trajectory up when the target is within rounding below the
  current duration.
- The `USE_BLENDING` error for a radius too small to blend names the actual
  cause.
- `CMAKE_BUILD_TYPE` is no longer forced when justblend is built as a
  subdirectory, and the tests build in that setup.

### Changed

- At an acceleration step, a sample reports the value just before the step,
  except at `t = 0`. The ends of a trapezoidal trajectory report the
  acceleration of the adjacent phase where they used to report zero, and a
  sample exactly on a trapezoid phase boundary reports the earlier phase.
- The last time returned by `samples(dt)` is exactly `duration()`.
- `TrajectoryGenerator::generate` is `const`, and generators are copyable.
- `find_package(justblend X.Y)` only accepts the same minor version until 1.0,
  since minor releases may break the API.
- `__version__` comes from the CMake project version.
- Python 3.10 is the minimum supported version.

### Added

- `__repr__` for `Limits`, `GenerationOptions`, `SegmentInfo` and
  `Trajectory` in Python.

## [0.1.0]

Initial release.
