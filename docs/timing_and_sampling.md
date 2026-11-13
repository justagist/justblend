# Timing and sampling

## Boundary speeds

`generate()` accepts optional tangential boundary speeds along the first and
last segment direction; infeasible requests (too fast for the limits or for
the available distance) raise a `ValidationError`:

```cpp
auto traj = gen.generate(waypoints, /*v_start=*/0.3, /*v_end=*/0.0);
```

```python
traj = gen.generate(waypoints, v_start=0.3, v_end=0.0)
```

The boundary state is restricted: each speed is a scalar along the first or
last segment direction, and the boundary acceleration is zero for the
S-curve profile. A start velocity in another direction, or a non-zero start
acceleration, cannot be expressed.

## Limit scaling

`GenerationOptions` carries `velocity_scaling_factor` and
`acceleration_scaling_factor` in `(0, 1]` (matching MoveIt's semantics):
they scale `v_max` / `a_max` at generation time; `j_max` is never scaled.

## Timing and deviation queries

The `Trajectory` exposes timing and corner-cut metadata:

- `segment_start_times()` -- start time of each segment plus the total
  duration as the last entry.
- `waypoint_times()` -- passage time per waypoint; for blended corners this
  is the time of closest approach (the blend midpoint).
- `junction_speeds()` -- path speed per waypoint: zero at a stop, the speed
  at its passage time at a merged collinear waypoint, and for a blended
  corner the corner speed, at which the blend starts and ends (the speed
  dips between them).
- `corner_types()` -- how each waypoint was handled: `ENDPOINT`, `PASS`
  (merged collinear waypoint), `STOP` or `BLEND`.
- `blend_radii()` -- the radius used at each blended corner, after clipping.
- `corner_deviations()` / `max_corner_deviation()` -- distance between each
  waypoint and the trajectory at its passage time: the closed-form corner
  cut at a blend (see [Corner handling](corner_handling.md#bounding-the-corner-cut)),
  the tiny distance to the merged segment at a collinear waypoint, and zero
  otherwise. Useful to bound how far the trajectory strays from the original
  polyline.
- `stretched_to(duration)` -- uniformly time-dilated copy with exactly the
  requested total duration (same path; velocities scale by `T/duration`,
  accelerations quadratically, jerks cubically, so all limits remain
  satisfied). Durations shorter than the current one raise a
  `ValidationError`.

## Sampling

Every sample carries position `q`, velocity `qd`, acceleration `qdd` and
jerk `qddd`. The jerk is analytic: the derivative of the acceleration within
each phase. The impulses at acceleration steps (trapezoidal phases, parabolic
blends) are not represented, so `qddd` is zero there; use the S-curve profile
with Hermite blends when the jerk must be bounded everywhere.

For real-time loops, `sampleInto` (C++ only) evaluates the trajectory
without allocating. It writes either into a reusable `Sample`, sized on the
first call, or into caller-owned buffers of size `dim()`:

```cpp
justblend::Sample s;
traj.sampleInto(t, s);                // allocates only on the first call

Eigen::VectorXd q(2), qd(2), qdd(2), qddd(2);
traj.sampleInto(t, q, qd, qdd, qddd); // never allocates unless it throws
```

### Conventions

- Times outside `[0, duration]` are clamped, including infinite ones; a NaN
  time raises a `ValidationError`.
- `samples(dt)` always ends with a sample at exactly `duration`.
- Position and velocity are continuous everywhere. Acceleration steps at the
  phase boundaries of a trapezoidal profile and at the ends of a parabolic
  blend; a sample exactly on a step reports the value just before it, except
  at `t = 0`, which reports the value just after. The ends of a trapezoidal
  trajectory therefore report the acceleration of the adjacent phase: the
  acceleration limit when it starts or ends in a ramp, zero when it starts or
  ends in cruise (a boundary speed at the velocity limit).
