"""Generate parity reference fixtures from the original Python script.

Usage:
    python tests/python/generate_reference.py <path/to/trajectory.py>

Writes .npz files under tests/python/reference_outputs/.
Each file contains arrays t, q, qd, qdd and metadata fields describing
the (profile, corner_handling, blend_shape, dataset) combination.
"""

from __future__ import annotations

import importlib.util
import json
import os
import sys
from itertools import product

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.normpath(os.path.join(HERE, "..", "data"))
OUT_DIR = os.path.join(HERE, "reference_outputs")


def load_reference(path: str):
    spec = importlib.util.spec_from_file_location("trajectory_ref", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load {path}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main(ref_path: str) -> None:
    ref = load_reference(ref_path)
    os.makedirs(OUT_DIR, exist_ok=True)

    with open(os.path.join(DATA_DIR, "waypoints.json")) as f:
        datasets = json.load(f)

    written = []
    for ds_name, ds in datasets.items():
        W = np.array(ds["waypoints"])
        vmax = np.array(ds["v_max"])
        amax = np.array(ds["a_max"])
        jmax = np.array(ds["j_max"]) if "j_max" in ds else None
        dt = float(ds["dt"])
        r = float(ds["blend_radius"])

        # scurve cases
        if jmax is not None:
            for ch, shape in product(("strict_corners", "hybrid", "use_blending"), ("parabolic", "hermite")):
                try:
                    traj = ref.generate_trajectory(
                        W, vmax, amax, jmax,
                        corner_handling=ch, blend_radius=r,
                        dt=dt, blend_shape=shape,
                    )
                except Exception as e:
                    print(f"  skip scurve {ds_name} {ch} {shape}: {e}")
                    continue
                fname = f"{ds_name}__scurve__{ch}__{shape}.npz"
                np.savez(
                    os.path.join(OUT_DIR, fname),
                    t=traj["t"], q=traj["q"], qd=traj["qd"], qdd=traj["qdd"],
                    waypoints=W, v_max=vmax, a_max=amax, j_max=jmax,
                    dt=np.array(dt), blend_radius=np.array(r),
                    profile="scurve", corner_handling=ch, blend_shape=shape,
                    dataset=ds_name,
                )
                written.append(fname)

        # trap cases (parabolic only; trap default)
        for ch in ("strict_corners", "hybrid"):
            try:
                traj = ref.generate_trajectory(
                    W, vmax, amax,
                    corner_handling=ch, blend_radius=r, dt=dt,
                )
            except Exception as e:
                print(f"  skip trap {ds_name} {ch}: {e}")
                continue
            fname = f"{ds_name}__trap__{ch}__parabolic.npz"
            np.savez(
                os.path.join(OUT_DIR, fname),
                t=traj["t"], q=traj["q"], qd=traj["qd"], qdd=traj["qdd"],
                waypoints=W, v_max=vmax, a_max=amax,
                dt=np.array(dt), blend_radius=np.array(r),
                profile="trap", corner_handling=ch, blend_shape="parabolic",
                dataset=ds_name,
            )
            written.append(fname)

    print(f"Wrote {len(written)} fixtures to {OUT_DIR}")
    for f in written:
        print(f"  {f}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    main(sys.argv[1])
