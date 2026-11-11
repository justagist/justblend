# Changelog

Notable changes to justblend. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.2.0] - 2026-11-11

### Fixed

- NaN and infinite values are rejected in limits, blend radii, waypoints,
  boundary speeds, sample times, sample steps and stretch durations. A NaN
  waypoint used to give an empty trajectory that crashed when sampled.
- Generation raises `ValidationError` when the result would be empty or have a
  non-finite duration, for example when a segment length overflows.
- Python `TrajectoryGenerator.limits()` and `options()` return copies. Editing
  the returned object could bypass the validating setters.
- Single and batch sampling choose the segment the same way, so a sample
  exactly on a segment boundary no longer depends on the order of the
  requested times.
- `samples(dt)` rejects a step too small for the sample count to be
  represented, and includes the final time when `dt` far exceeds the duration.
- The `USE_BLENDING` error for a radius too small to blend names the actual
  cause.
- `CMAKE_BUILD_TYPE` is no longer forced when justblend is built as a
  subdirectory, and the tests build in that setup.

### Changed

- At an acceleration step, a sample reports the value just before the step,
  except at `t = 0`. Trapezoidal trajectories report their acceleration limit
  at both ends where they used to report zero, and a sample exactly on a
  trapezoid phase boundary reports the earlier phase.
- The last time returned by `samples(dt)` is exactly `duration()`.
- `TrajectoryGenerator::generate` is `const`, and generators are copyable.
- `__version__` comes from the CMake project version.
- Python 3.10 is the minimum supported version.

### Added

- `__repr__` for `Limits`, `GenerationOptions`, `SegmentInfo` and
  `Trajectory` in Python.

## [0.1.0]

Initial release.
