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
    traj = _make_scurve(waypoints_3d, limits_3d, blend_radius=0.15, corner_handling=jb.CornerHandling.STRICT_CORNERS)
    assert traj.duration() > 0
    assert traj.dim() == 3
    assert traj.blend_shape() == jb.BlendShape.HERMITE  # default for SCurve


def test_sample_and_samples_dt_agree(waypoints_3d, limits_3d):
    traj = _make_scurve(waypoints_3d, limits_3d, blend_radius=0.15, corner_handling=jb.CornerHandling.HYBRID)
    dense = traj.samples(dt=0.01)
    for i in (0, 5, 17, dense.q.shape[0] - 1):
        s = traj.sample(float(dense.t[i]))
        np.testing.assert_allclose(s.q, dense.q[i], atol=1e-12)
        np.testing.assert_allclose(s.qd, dense.qd[i], atol=1e-12)
        np.testing.assert_allclose(s.qdd, dense.qdd[i], atol=1e-12)


def test_samples_dt_and_times_agree(waypoints_3d, limits_3d):
    traj = _make_scurve(waypoints_3d, limits_3d, blend_radius=0.15, corner_handling=jb.CornerHandling.HYBRID)
    by_dt = traj.samples(dt=0.005)
    by_t = traj.samples(np.asarray(by_dt.t))
    np.testing.assert_allclose(by_dt.q, by_t.q, atol=1e-15)
    np.testing.assert_allclose(by_dt.qd, by_t.qd, atol=1e-15)
    np.testing.assert_allclose(by_dt.qdd, by_t.qdd, atol=1e-15)


def test_endpoint_snap(waypoints_3d, limits_3d):
    traj = _make_scurve(waypoints_3d, limits_3d, blend_radius=0.15, corner_handling=jb.CornerHandling.HYBRID)
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
    traj = _make_scurve(waypoints_3d, limits_3d, blend_radius=0.15, corner_handling=jb.CornerHandling.HYBRID)
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
    gen.set_options(jb.GenerationOptions(blend_radius=1e-12, corner_handling=jb.CornerHandling.USE_BLENDING))
    with pytest.raises(jb.ValidationError):
        gen.generate(waypoints_3d)


def test_dim_mismatch_raises(limits_3d):
    gen = jb.SCurveTrajectoryGenerator(dim=2)
    with pytest.raises(ValueError):
        gen.set_limits(limits_3d)


def test_per_corner_blend_radii(waypoints_3d, limits_3d):
    # 5 waypoints -> 3 interior corners; supply three distinct radii.
    radii = np.array([0.10, 0.20, 0.05])
    opts = jb.GenerationOptions(
        blend_radius=999.0,
        blend_radii=radii,
        corner_handling=jb.CornerHandling.HYBRID,
    )
    gen = jb.SCurveTrajectoryGenerator(dim=3)
    gen.set_limits(limits_3d)
    gen.set_options(opts)
    traj = gen.generate(waypoints_3d)
    planned = list(traj.blend_radii())
    assert planned[0] == 0.0  # endpoint
    assert planned[-1] == 0.0  # endpoint
    np.testing.assert_allclose(planned[1:-1], radii, atol=1e-12)


def test_blend_radii_size_one_broadcasts(waypoints_3d, limits_3d):
    opts = jb.GenerationOptions(
        blend_radius=0.999,
        blend_radii=np.array([0.12]),
        corner_handling=jb.CornerHandling.HYBRID,
    )
    gen = jb.SCurveTrajectoryGenerator(dim=3)
    gen.set_limits(limits_3d)
    gen.set_options(opts)
    traj = gen.generate(waypoints_3d)
    np.testing.assert_allclose(list(traj.blend_radii())[1:-1], [0.12, 0.12, 0.12], atol=1e-12)


def test_blend_radii_wrong_size_raises(waypoints_3d, limits_3d):
    opts = jb.GenerationOptions(
        blend_radii=np.array([0.1, 0.1]),  # need 3 or 1
        corner_handling=jb.CornerHandling.HYBRID,
    )
    gen = jb.SCurveTrajectoryGenerator(dim=3)
    gen.set_limits(limits_3d)
    gen.set_options(opts)
    with pytest.raises(ValueError):
        gen.generate(waypoints_3d)


def test_blend_radii_non_positive_rejected_in_set_options():
    opts = jb.GenerationOptions(blend_radii=np.array([0.1, -0.05, 0.2]))
    gen = jb.SCurveTrajectoryGenerator(dim=3)
    with pytest.raises(ValueError):
        gen.set_options(opts)


def test_boundary_speeds():
    W = np.array([[0.0, 0.0], [2.0, 0.0]])
    gen = jb.SCurveTrajectoryGenerator(dim=2)
    gen.set_limits(jb.Limits(v_max=np.ones(2), a_max=2 * np.ones(2), j_max=10 * np.ones(2)))
    traj = gen.generate(W, v_start=0.3, v_end=0.2)
    np.testing.assert_allclose(traj.sample(0.0).qd, [0.3, 0.0], atol=1e-9)
    np.testing.assert_allclose(traj.sample(traj.duration()).qd, [0.2, 0.0], atol=1e-9)
    with pytest.raises(jb.ValidationError):
        gen.generate(np.array([[0.0, 0.0], [0.01, 0.0]]), v_end=0.9)


def test_waypoint_times_and_deviations(waypoints_3d, limits_3d):
    traj = _make_scurve(waypoints_3d, limits_3d, blend_radius=0.15, corner_handling=jb.CornerHandling.HYBRID)
    wt = traj.waypoint_times()
    assert len(wt) == waypoints_3d.shape[0]
    assert wt[0] == 0.0
    assert wt[-1] == pytest.approx(traj.duration(), abs=1e-12)
    assert all(b > a for a, b in zip(wt, wt[1:]))

    dev = traj.corner_deviations()
    assert len(dev) == waypoints_3d.shape[0]
    assert traj.max_corner_deviation() == pytest.approx(max(dev))
    for k, ct in enumerate(traj.corner_types()):
        if ct == jb.CornerType.BLEND:
            s = traj.sample(wt[k])
            assert np.linalg.norm(s.q - waypoints_3d[k]) == pytest.approx(dev[k], abs=1e-9)
        else:
            assert dev[k] == 0.0

    st = traj.segment_start_times()
    assert len(st) == traj.num_segments() + 1
    assert st[-1] == pytest.approx(traj.duration(), abs=1e-12)


def test_scaling_factors(waypoints_3d, limits_3d):
    full = _make_scurve(waypoints_3d, limits_3d, corner_handling=jb.CornerHandling.HYBRID)
    scaled = _make_scurve(
        waypoints_3d,
        limits_3d,
        corner_handling=jb.CornerHandling.HYBRID,
        velocity_scaling_factor=0.5,
        acceleration_scaling_factor=0.5,
    )
    assert scaled.duration() > full.duration()
    r = scaled.samples(dt=0.005)
    for i in range(3):
        assert np.max(np.abs(r.qd[:, i])) <= 0.5 * limits_3d.v_max[i] + 1e-9
        assert np.max(np.abs(r.qdd[:, i])) <= 0.5 * limits_3d.a_max[i] + 1e-9
    with pytest.raises(jb.ValidationError):
        jb.SCurveTrajectoryGenerator(dim=3).set_options(jb.GenerationOptions(velocity_scaling_factor=1.5))


def test_stretched_to(waypoints_3d, limits_3d):
    traj = _make_scurve(waypoints_3d, limits_3d, blend_radius=0.15, corner_handling=jb.CornerHandling.HYBRID)
    target = 2.0 * traj.duration()
    slow = traj.stretched_to(target)
    assert slow.duration() == target
    s = target / traj.duration()
    for f in (0.0, 0.25, 0.6, 1.0):
        t = f * traj.duration()
        a, b = traj.sample(t), slow.sample(s * t)
        np.testing.assert_allclose(b.q, a.q, atol=1e-9)
        np.testing.assert_allclose(b.qd, a.qd / s, atol=1e-9)
    with pytest.raises(jb.ValidationError):
        traj.stretched_to(0.5 * traj.duration())
