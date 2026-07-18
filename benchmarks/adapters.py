"""Adapters that run each trajectory generator on a common scenario format.

Every adapter takes waypoints + limits and returns a `BenchResult` with the
trajectory sampled on a uniform grid plus the wall-clock generation time.
Optional back-ends (ruckig, toppra) are imported lazily so the suite degrades
gracefully when they are not installed.
"""

from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass

import numpy as np

try:
    import justblend as jb
except ImportError:  # in-tree dev build fallback
    sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "python")))
    import justblend as jb


@dataclass
class BenchResult:
    name: str
    duration: float
    compute_ms: float
    t: np.ndarray
    q: np.ndarray
    qd: np.ndarray
    qdd: np.ndarray


def _time_ms(fn, repeats: int = 3):
    """Times a zero-argument callable.

    Args:
        fn (callable): Callable to run and time.
        repeats (int): Number of runs; the best time is kept.

    Returns:
        tuple: (result of the last call, best wall-clock time in ms).
    """
    best = float("inf")
    out = None
    for _ in range(repeats):
        t0 = time.perf_counter()
        out = fn()
        best = min(best, (time.perf_counter() - t0) * 1e3)
    return out, best


def _sample_ruckig_trajectories(trajs, dof: int, dt: float):
    """Samples chained ruckig trajectories on a uniform grid.

    Args:
        trajs (list): Ruckig Trajectory objects, executed back to back.
        dof (int): Degrees of freedom.
        dt (float): Sampling step [s].

    Returns:
        tuple: (total duration, t, q, qd, qdd) with arrays over the grid.
    """
    durations = np.array([tr.duration for tr in trajs])
    starts = np.concatenate([[0.0], np.cumsum(durations)])
    total = float(starts[-1])

    ts = np.arange(0.0, total, dt)
    if ts[-1] < total:
        ts = np.append(ts, total)
    q = np.empty((len(ts), dof))
    qd = np.empty_like(q)
    qdd = np.empty_like(q)
    idx = np.clip(np.searchsorted(starts, ts, side="right") - 1, 0, len(trajs) - 1)
    for i, (t, j) in enumerate(zip(ts, idx)):
        pos, vel, acc = trajs[j].at_time(min(t - starts[j], trajs[j].duration))
        q[i], qd[i], qdd[i] = pos, vel, acc
    return total, ts, q, qd, qdd


def run_justblend(name, W, v_max, a_max, j_max, blend_radius, corner_handling, dt) -> BenchResult:
    """Runs a justblend generator on the scenario.

    Args:
        name (str): Row label for the report.
        W (np.ndarray): N x dof waypoint matrix.
        v_max/a_max (np.ndarray): Per-axis velocity/acceleration limits.
        j_max (np.ndarray | None): Per-axis jerk limits; None selects the trapezoidal generator.
        blend_radius (float): Corner cut-back distance.
        corner_handling (jb.CornerHandling): Corner mode.
        dt (float): Sampling step [s].
    """
    scurve = j_max is not None
    gen = (jb.SCurveTrajectoryGenerator if scurve else jb.TrapezoidalTrajectoryGenerator)(dim=W.shape[1])
    gen.set_limits(jb.Limits(v_max=v_max, a_max=a_max, j_max=j_max))
    gen.set_options(jb.GenerationOptions(blend_radius=blend_radius, corner_handling=corner_handling))
    traj, ms = _time_ms(lambda: gen.generate(W))
    r = traj.samples(dt)
    return BenchResult(name, traj.duration(), ms, np.asarray(r.t), np.asarray(r.q), np.asarray(r.qd), np.asarray(r.qdd))


def run_ruckig_chained(name, W, v_max, a_max, j_max, dt) -> BenchResult:
    """Chains community ruckig segments rest-to-rest per waypoint.

    Per-axis time-optimal and comparable to strict corners, but the path
    between waypoints is NOT constrained to the straight line.

    Args:
        name (str): Row label for the report.
        W (np.ndarray): N x dof waypoint matrix.
        v_max/a_max/j_max (np.ndarray): Per-axis limits.
        dt (float): Sampling step [s].
    """
    from ruckig import InputParameter, Result, Ruckig
    from ruckig import Trajectory as RuckigTrajectory

    dof = W.shape[1]

    def build():
        otg = Ruckig(dof)
        trajs = []
        for k in range(W.shape[0] - 1):
            inp = InputParameter(dof)
            inp.current_position = list(W[k])
            inp.target_position = list(W[k + 1])
            inp.max_velocity = list(v_max)
            inp.max_acceleration = list(a_max)
            inp.max_jerk = list(j_max)
            traj = RuckigTrajectory(dof)
            res = otg.calculate(inp, traj)
            if res not in (Result.Working, Result.Finished):
                raise RuntimeError(f"ruckig failed on segment {k}: {res}")
            trajs.append(traj)
        return trajs

    trajs, ms = _time_ms(build)
    total, ts, q, qd, qdd = _sample_ruckig_trajectories(trajs, dof, dt)
    return BenchResult(name, total, ms, ts, q, qd, qdd)


def run_ruckig_waypoints(name, W, v_max, a_max, j_max, dt) -> BenchResult:
    """Runs ruckig's intermediate-waypoint mode (no stops at waypoints).

    The community edition solves this via ruckig's cloud API (Ruckig Pro is
    local), so generation time includes a network round trip and the
    scenario data leaves the machine.

    Args:
        name (str): Row label for the report.
        W (np.ndarray): N x dof waypoint matrix.
        v_max/a_max/j_max (np.ndarray): Per-axis limits.
        dt (float): Sampling step [s].
    """
    from ruckig import InputParameter, Result, Ruckig
    from ruckig import Trajectory as RuckigTrajectory

    dof = W.shape[1]
    n_intermediate = W.shape[0] - 2

    def build():
        otg = Ruckig(dof, 0.01, n_intermediate)
        inp = InputParameter(dof)
        inp.current_position = list(W[0])
        inp.intermediate_positions = [list(w) for w in W[1:-1]]
        inp.target_position = list(W[-1])
        inp.max_velocity = list(v_max)
        inp.max_acceleration = list(a_max)
        inp.max_jerk = list(j_max)
        traj = RuckigTrajectory(dof, n_intermediate)
        res = otg.calculate(inp, traj)
        if res not in (Result.Working, Result.Finished):
            raise RuntimeError(f"ruckig waypoint calculation failed: {res}")
        return traj

    # Single run: the community edition round-trips to the cloud per call.
    traj, ms = _time_ms(build, repeats=1)
    total, ts, q, qd, qdd = _sample_ruckig_trajectories([traj], dof, dt)
    return BenchResult(name, total, ms, ts, q, qd, qdd)


def _toppra_parametrize(path, v_max, a_max):
    """Time-parametrizes a toppra path under velocity/acceleration limits.

    Args:
        path: A toppra interpolator over [0, 1].
        v_max/a_max (np.ndarray): Per-axis limits.

    Returns:
        A sampled toppra joint trajectory.
    """
    import toppra as ta
    import toppra.algorithm as algo
    import toppra.constraint as constraint

    ta.setup_logging("WARNING")
    vlim = np.vstack([-v_max, v_max]).T
    alim = np.vstack([-a_max, a_max]).T
    instance = algo.TOPPRA(
        [constraint.JointVelocityConstraint(vlim), constraint.JointAccelerationConstraint(alim)],
        path,
        parametrizer="ParametrizeConstAccel",
    )
    traj = instance.compute_trajectory()
    if traj is None:
        raise RuntimeError("toppra failed to parametrize the path")
    return traj


def run_toppra_chained(name, W, v_max, a_max, dt) -> BenchResult:
    """Runs toppra per straight-line segment, rest-to-rest at every waypoint.

    Traverses exactly the same path as justblend's strict-corner mode.

    Args:
        name (str): Row label for the report.
        W (np.ndarray): N x dof waypoint matrix.
        v_max/a_max (np.ndarray): Per-axis limits.
        dt (float): Sampling step [s].
    """
    import toppra as ta

    def build():
        return [
            _toppra_parametrize(
                ta.SplineInterpolator([0.0, 1.0], W[k : k + 2], bc_type="clamped"), v_max, a_max
            )
            for k in range(W.shape[0] - 1)
        ]

    trajs, ms = _time_ms(build)
    durations = np.array([tr.duration for tr in trajs])
    starts = np.concatenate([[0.0], np.cumsum(durations)])
    total = float(starts[-1])

    ts = np.arange(0.0, total, dt)
    if ts[-1] < total:
        ts = np.append(ts, total)
    idx = np.clip(np.searchsorted(starts, ts, side="right") - 1, 0, len(trajs) - 1)
    q = np.empty((len(ts), W.shape[1]))
    qd = np.empty_like(q)
    qdd = np.empty_like(q)
    for i, (t, j) in enumerate(zip(ts, idx)):
        tl = min(t - starts[j], trajs[j].duration)
        q[i] = trajs[j](tl)
        qd[i] = trajs[j](tl, 1)
        qdd[i] = trajs[j](tl, 2)
    return BenchResult(name, total, ms, ts, q, qd, qdd)


def run_toppra_spline(name, W, v_max, a_max, dt) -> BenchResult:
    """Runs toppra on one cubic spline through all waypoints (no stops).

    Passes exactly through every waypoint at speed; the natural comparison
    for justblend's blending modes.

    Args:
        name (str): Row label for the report.
        W (np.ndarray): N x dof waypoint matrix.
        v_max/a_max (np.ndarray): Per-axis limits.
        dt (float): Sampling step [s].
    """
    import toppra as ta

    def build():
        ss = np.linspace(0.0, 1.0, W.shape[0])
        return _toppra_parametrize(ta.SplineInterpolator(ss, W), v_max, a_max)

    traj, ms = _time_ms(build)
    total = float(traj.duration)
    ts = np.arange(0.0, total, dt)
    if ts[-1] < total:
        ts = np.append(ts, total)
    return BenchResult(name, total, ms, ts, traj(ts), traj(ts, 1), traj(ts, 2))
