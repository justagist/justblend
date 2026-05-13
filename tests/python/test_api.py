"""Smoke tests for the Python binding API."""

from __future__ import annotations

import numpy as np
import pytest

import justblend as jb


@pytest.fixture
def waypoints_3d():
    return np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.5, -0.3],
            [1.5, -0.2, 0.4],
            [0.5, 0.8, 1.0],
            [0.0, 0.0, 0.0],
        ]
    )


@pytest.fixture
def limits_3d():
    return jb.Limits(
        v_max=np.array([1.5, 1.2, 1.0]),
        a_max=np.array([3.0, 2.5, 2.0]),
        j_max=np.array([20.0, 18.0, 15.0]),
    )


def _make_scurve(waypoints, limits, **opts):
    o = jb.GenerationOptions(**opts)
    gen = jb.SCurveTrajectoryGenerator(dim=waypoints.shape[1])
    gen.set_limits(limits)
    gen.set_options(o)
    return gen.generate(waypoints)


def test_scurve_strict_corners(waypoints_3d, limits_3d):
    traj = _make_scurve(waypoints_3d, limits_3d,
                        blend_radius=0.15, corner_handling=jb.CornerHandling.STRICT_CORNERS)
    assert traj.duration() > 0
    assert traj.dim() == 3
    assert traj.blend_shape() == jb.BlendShape.HERMITE  # default for SCurve


def test_sample_and_samples_dt_agree(waypoints_3d, limits_3d):
    traj = _make_scurve(waypoints_3d, limits_3d, blend_radius=0.15,
                        corner_handling=jb.CornerHandling.HYBRID)
    dense = traj.samples(dt=0.01)
    for i in (0, 5, 17, dense.q.shape[0] - 1):
        s = traj.sample(float(dense.t[i]))
        np.testing.assert_allclose(s.q, dense.q[i], atol=1e-12)
        np.testing.assert_allclose(s.qd, dense.qd[i], atol=1e-12)
        np.testing.assert_allclose(s.qdd, dense.qdd[i], atol=1e-12)


def test_samples_dt_and_times_agree(waypoints_3d, limits_3d):
    traj = _make_scurve(waypoints_3d, limits_3d, blend_radius=0.15,
                        corner_handling=jb.CornerHandling.HYBRID)
    by_dt = traj.samples(dt=0.005)
    by_t = traj.samples(np.asarray(by_dt.t))
    np.testing.assert_allclose(by_dt.q, by_t.q, atol=1e-15)
    np.testing.assert_allclose(by_dt.qd, by_t.qd, atol=1e-15)
    np.testing.assert_allclose(by_dt.qdd, by_t.qdd, atol=1e-15)


def test_endpoint_snap(waypoints_3d, limits_3d):
    traj = _make_scurve(waypoints_3d, limits_3d, blend_radius=0.15,
                        corner_handling=jb.CornerHandling.HYBRID)
    s0 = traj.sample(0.0)
    s1 = traj.sample(traj.duration())
    np.testing.assert_allclose(s0.q, waypoints_3d[0])
    np.testing.assert_allclose(s1.q, waypoints_3d[-1])
    np.testing.assert_allclose(s0.qd, np.zeros(3))
    np.testing.assert_allclose(s0.qdd, np.zeros(3))
    np.testing.assert_allclose(s1.qd, np.zeros(3))
    np.testing.assert_allclose(s1.qdd, np.zeros(3))


def test_limits_required(waypoints_3d):
    gen = jb.SCurveTrajectoryGenerator(dim=3)
    with pytest.raises(jb.ValidationError):
        gen.generate(waypoints_3d)


def test_scurve_requires_jmax(waypoints_3d):
    gen = jb.SCurveTrajectoryGenerator(dim=3)
    L = jb.Limits(
        v_max=np.array([1.5, 1.2, 1.0]),
        a_max=np.array([3.0, 2.5, 2.0]),
    )
    with pytest.raises(jb.ValidationError):
        gen.set_limits(L)


def test_duplicate_waypoints_rejected(limits_3d):
    W = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    gen = jb.SCurveTrajectoryGenerator(dim=3)
    gen.set_limits(limits_3d)
    with pytest.raises(jb.ValidationError):
        gen.generate(W)


def test_trap_generator(waypoints_3d):
    L = jb.Limits(
        v_max=np.array([1.5, 1.2, 1.0]),
        a_max=np.array([3.0, 2.5, 2.0]),
    )
    gen = jb.TrapezoidalTrajectoryGenerator(dim=3)
    gen.set_limits(L)
    gen.set_options(jb.GenerationOptions(blend_radius=0.15, corner_handling=jb.CornerHandling.HYBRID))
    traj = gen.generate(waypoints_3d)
    assert traj.blend_shape() == jb.BlendShape.PARABOLIC
    r = traj.samples(dt=0.005)
    for i in range(3):
        assert np.max(np.abs(r.qd[:, i])) <= L.v_max[i] + 1e-9
        assert np.max(np.abs(r.qdd[:, i])) <= L.a_max[i] + 1e-9


def test_segments_metadata(waypoints_3d, limits_3d):
    traj = _make_scurve(waypoints_3d, limits_3d, blend_radius=0.15,
                        corner_handling=jb.CornerHandling.HYBRID)
    segs = traj.segments()
    assert len(segs) == traj.num_segments()
    total = sum(s.duration for s in segs)
    assert total == pytest.approx(traj.duration(), rel=0, abs=1e-12)
    # cumulative durations across segments add up to duration
    has_blend = any(s.type == jb.SegmentType.BLEND for s in segs)
    has_linear = any(s.type == jb.SegmentType.LINEAR for s in segs)
    assert has_blend and has_linear


def test_use_blending_collapse_raises(limits_3d, waypoints_3d):
    gen = jb.SCurveTrajectoryGenerator(dim=3)
    gen.set_limits(limits_3d)
    gen.set_options(
        jb.GenerationOptions(
            blend_radius=1e-12, corner_handling=jb.CornerHandling.USE_BLENDING
        )
    )
    with pytest.raises(jb.ValidationError):
        gen.generate(waypoints_3d)


def test_dim_mismatch_raises(limits_3d):
    gen = jb.SCurveTrajectoryGenerator(dim=2)
    with pytest.raises(ValueError):
        gen.set_limits(limits_3d)
