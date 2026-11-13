"""Parity test against the original Python reference implementation.

Loads pre-computed reference outputs from tests/python/reference_outputs/ and
samples the C++ implementation at the same time points. Asserts that q, qd,
qdd agree to within 1e-9 (the reference math is what the C++ port mirrors).

If no fixtures are present, the test is skipped with instructions to
regenerate them via:

    python tests/python/generate_reference.py path/to/trajectory.py
"""

from __future__ import annotations

import glob
import os

import numpy as np
import pytest

import justblend as jb

HERE = os.path.dirname(os.path.abspath(__file__))
FIX_DIR = os.path.join(HERE, "reference_outputs")
PARITY_TOL = 1e-9


def _fixture_paths():
    return sorted(glob.glob(os.path.join(FIX_DIR, "*.npz")))


def _ids(paths):
    return [os.path.splitext(os.path.basename(p))[0] for p in paths]


paths = _fixture_paths()
if not paths:
    pytest.skip(
        f"No parity fixtures in {FIX_DIR}. Run: python tests/python/generate_reference.py <path/to/trajectory.py>",
        allow_module_level=True,
    )


@pytest.mark.parametrize("path", paths, ids=_ids(paths))
def test_parity(path):
    data = np.load(path)
    profile = str(data["profile"])
    corner_handling = str(data["corner_handling"])
    blend_shape = str(data["blend_shape"])

    W = data["waypoints"]
    v_max = data["v_max"]
    a_max = data["a_max"]
    j_max = data["j_max"] if "j_max" in data.files else None
    blend_radius = float(data["blend_radius"])
    t_ref = data["t"]

    limits = jb.Limits(v_max=v_max, a_max=a_max, j_max=j_max if j_max is not None else None)

    ch_map = {
        "strict_corners": jb.CornerHandling.STRICT_CORNERS,
        "use_blending": jb.CornerHandling.BLEND_ALL,
        # The reference's hybrid blends every corner that allows it.
        "hybrid": jb.CornerHandling.BLEND_WHERE_FEASIBLE,
    }
    bs_map = {"parabolic": jb.BlendShape.PARABOLIC, "hermite": jb.BlendShape.HERMITE}
    opts = jb.GenerationOptions(
        blend_radius=blend_radius,
        corner_handling=ch_map[corner_handling],
        blend_shape=bs_map[blend_shape],
    )

    if profile == "scurve":
        gen = jb.SCurveTrajectoryGenerator(dim=W.shape[1])
    elif profile == "trap":
        gen = jb.TrapezoidalTrajectoryGenerator(dim=W.shape[1])
    else:
        pytest.fail(f"unknown profile {profile!r}")
    gen.set_limits(limits)
    gen.set_options(opts)
    traj = gen.generate(W)

    out = traj.samples(t_ref)

    err_q = float(np.max(np.abs(out.q - data["q"])))
    err_qd = float(np.max(np.abs(out.qd - data["qd"])))
    ref_qdd = data["qdd"].copy()
    if profile == "trap":
        # Trapezoidal acceleration is piecewise constant. A sample exactly on a
        # step reports the value before the step (after it at t = 0), where the
        # reference reports the value after it and zero at both ends. Compare
        # those samples with the neighbouring reference sample instead. A
        # mismatch anywhere else still fails.
        is_step = np.ones(len(ref_qdd), dtype=bool)
        is_step[1:-1] = np.any(np.abs(np.diff(ref_qdd, axis=0)) > PARITY_TOL, axis=1)[:-1]
        mismatch = np.any(np.abs(out.qdd - ref_qdd) > PARITY_TOL, axis=1)
        on_step = np.flatnonzero(is_step & mismatch)
        ref_qdd[on_step] = ref_qdd[np.where(on_step == 0, 1, on_step - 1)]
    err_qdd = float(np.max(np.abs(out.qdd - ref_qdd)))
    assert err_q <= PARITY_TOL, f"q diff {err_q}"
    assert err_qd <= PARITY_TOL, f"qd diff {err_qd}"
    assert err_qdd <= PARITY_TOL, f"qdd diff {err_qdd}"
