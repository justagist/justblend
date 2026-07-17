"""Benchmark justblend against ruckig and toppra on shared scenarios.

Regenerates benchmarks/README.md (tables + plots + notes):

    pip install .[bench,plot]
    python benchmarks/run.py [--dt 0.001]
"""

from __future__ import annotations

import argparse
import json
import os
from dataclasses import dataclass

import numpy as np

import adapters
from adapters import BenchResult

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "tests", "data", "waypoints.json")
PLOTS_DIR = os.path.join(HERE, "plots")

# Methods overlaid on the path plots: only those whose geometry differs from
# the waypoint polyline (strict/chained methods follow it exactly).
PATH_PLOT_METHODS = ("jb-scurve-hybrid", "jb-trap-hybrid", "ruckig-chained", "toppra-spline")

# Categorical palette (light mode), fixed slot order; single-hue bars use slot 1.
SERIES_COLOURS = ("#2a78d6", "#008300", "#e87ba4", "#eda100")
BAR_COLOUR = "#2a78d6"
POLYLINE_GREY = "#9a9994"


@dataclass
class Scenario:
    name: str
    W: np.ndarray
    v_max: np.ndarray
    a_max: np.ndarray
    j_max: np.ndarray | None
    blend_radius: float
    dt: float = 1e-3


def scenarios() -> list[Scenario]:
    out = []
    with open(DATA) as f:
        for name, ds in json.load(f).items():
            out.append(
                Scenario(
                    name,
                    np.array(ds["waypoints"]),
                    np.array(ds["v_max"]),
                    np.array(ds["a_max"]),
                    np.array(ds["j_max"]) if "j_max" in ds else None,
                    float(ds["blend_radius"]),
                )
            )
    rng = np.random.default_rng(42)
    out.append(
        Scenario(
            "joints_6dof",
            rng.uniform(-1.5, 1.5, size=(6, 6)),
            np.full(6, 2.0),
            np.full(6, 5.0),
            np.full(6, 50.0),
            blend_radius=0.15,
        )
    )
    return out


def polyline_deviation(q: np.ndarray, W: np.ndarray) -> float:
    """Max distance from the sampled path to the waypoint polyline."""
    best = np.full(q.shape[0], np.inf)
    for a, b in zip(W[:-1], W[1:]):
        ab = b - a
        tt = np.clip((q - a) @ ab / float(ab @ ab), 0.0, 1.0)
        best = np.minimum(best, np.linalg.norm(q - (a + tt[:, None] * ab), axis=1))
    return float(best.max())


def waypoint_miss(q: np.ndarray, W: np.ndarray) -> float:
    """Worst closest-approach distance to any interior waypoint."""
    if W.shape[0] <= 2:
        return 0.0
    return max(float(np.linalg.norm(q - wp, axis=1).min()) for wp in W[1:-1])


def limit_ratios(r: BenchResult, sc: Scenario):
    vel = float(np.max(np.abs(r.qd) / sc.v_max))
    acc = float(np.max(np.abs(r.qdd) / sc.a_max))
    jerk = float("nan")
    if sc.j_max is not None and len(r.t) > 2:
        qddd = np.diff(r.qdd, axis=0) / np.diff(r.t)[:, None]
        jerk = float(np.max(np.abs(qddd) / sc.j_max))
    return vel, acc, jerk


def run_scenario(sc: Scenario) -> list[BenchResult]:
    import justblend as jb

    results = []

    def add(fn):
        try:
            results.append(fn())
        except ImportError as e:
            results.append(BenchResult(f"({e.name} not installed)", np.nan, np.nan, None, None, None, None))
        except Exception as e:  # noqa: BLE001 - report and continue benchmarking
            results.append(BenchResult(f"FAILED: {e}", np.nan, np.nan, None, None, None, None))

    for ch, tag in ((jb.CornerHandling.STRICT_CORNERS, "strict"), (jb.CornerHandling.HYBRID, "hybrid")):
        add(
            lambda ch=ch, tag=tag: adapters.run_justblend(
                f"jb-trap-{tag}", sc.W, sc.v_max, sc.a_max, None, sc.blend_radius, ch, sc.dt
            )
        )
        if sc.j_max is not None:
            add(
                lambda ch=ch, tag=tag: adapters.run_justblend(
                    f"jb-scurve-{tag}", sc.W, sc.v_max, sc.a_max, sc.j_max, sc.blend_radius, ch, sc.dt
                )
            )
    if sc.j_max is not None:
        add(lambda: adapters.run_ruckig_chained("ruckig-chained", sc.W, sc.v_max, sc.a_max, sc.j_max, sc.dt))
    add(lambda: adapters.run_toppra_chained("toppra-chained", sc.W, sc.v_max, sc.a_max, sc.dt))
    add(lambda: adapters.run_toppra_spline("toppra-spline", sc.W, sc.v_max, sc.a_max, sc.dt))
    return results


def plot_scenario(sc: Scenario, results: list[BenchResult], out_png: str) -> None:
    """One figure per scenario: path overlay, durations, generation times."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ok = [r for r in results if r.t is not None]
    fig, (ax_path, ax_dur, ax_ms) = plt.subplots(1, 3, figsize=(13.5, 4.2), facecolor="white")

    ax_path.plot(sc.W[:, 0], sc.W[:, 1], "--", color=POLYLINE_GREY, lw=1.2, label="waypoint polyline", zorder=1)
    ax_path.plot(sc.W[:, 0], sc.W[:, 1], "o", color=POLYLINE_GREY, ms=6, zorder=2)
    overlay = [r for r in ok if r.name in PATH_PLOT_METHODS]
    for r, colour in zip(overlay, SERIES_COLOURS):
        ax_path.plot(r.q[:, 0], r.q[:, 1], color=colour, lw=2.0, label=r.name, zorder=3)
    ax_path.set_title(f"{sc.name}: path (dims 0-1)", fontsize=10)
    ax_path.set_aspect("equal", adjustable="datalim")
    ax_path.legend(fontsize=7, frameon=False)

    names = [r.name for r in ok]
    for ax, values, title, log in (
        (ax_dur, [r.duration for r in ok], "trajectory duration [s]", False),
        (ax_ms, [r.compute_ms for r in ok], "generation time [ms]", True),
    ):
        y = np.arange(len(ok))[::-1]
        ax.barh(y, values, height=0.55, color=BAR_COLOUR)
        ax.set_yticks(y, names, fontsize=8)
        ax.set_title(title, fontsize=10)
        if log:
            ax.set_xscale("log")
        for yi, v in zip(y, values):
            ax.annotate(
                f" {v:.3g}", (v, yi), va="center", fontsize=7, color="#52514e"
            )
        ax.spines[["top", "right"]].set_visible(False)
        ax.tick_params(axis="x", labelsize=8)

    fig.tight_layout()
    fig.savefig(out_png, dpi=140)
    plt.close(fig)


INSTRUCTIONS = """\
# justblend benchmarks

Comparison of justblend against [ruckig](https://github.com/pantor/ruckig)
(community edition) and [toppra](https://github.com/hungpham2511/toppra) on
shared waypoint scenarios.

**This file is generated** by `benchmarks/run.py`; do not edit it by hand.

## Running

```bash
pip install .[bench,plot]
python benchmarks/run.py [--dt 0.001]
```

## Methods

| method | profile | corner handling | path |
|---|---|---|---|
| jb-trap-strict | trapezoidal | full stop at every waypoint | exact polyline |
| jb-trap-hybrid | trapezoidal | parabolic blends where feasible | polyline with rounded corners |
| jb-scurve-strict | jerk-limited S-curve | full stop at every waypoint | exact polyline |
| jb-scurve-hybrid | jerk-limited S-curve | Hermite blends where feasible | polyline with rounded corners |
| ruckig-chained | jerk-limited | full stop at every waypoint | unconstrained between waypoints (per-axis profiles) |
| toppra-chained | vel/acc time-optimal | full stop at every waypoint | exact polyline |
| toppra-spline | vel/acc time-optimal | passes through waypoints at speed | cubic spline through waypoints |

Intermediate-waypoint support in ruckig is a Pro feature, so the community
edition is chained rest-to-rest per segment. toppra has no jerk constraint.

## Results
"""

NOTES = """\
## Metric definitions

- **vel / acc / jerk**: peak per-axis ratio against the corresponding limit
  (<= 1 means the limit is respected). Jerk is finite-differenced from
  sampled accelerations, so large values indicate acceleration steps --
  expected for trapezoidal profiles, parabolic blends, ruckig segment
  boundaries and toppra (no jerk constraint).
- **polyline dev**: max distance from the sampled path to the waypoint
  polyline.
- **wp miss**: worst closest approach to any interior waypoint.

## Reading the numbers

- jb-*-strict and toppra-chained solve the identical polyline-with-stops
  problem. jb-trap-strict is analytically time-optimal there; toppra's grid
  discretisation makes it slightly conservative.
- jb-scurve-strict has matched ruckig-chained's duration on all scenarios to
  date: for straight rest-to-rest legs where the same axis binds the limits,
  the closed-form path profile is per-axis time-optimal.
- ruckig-chained moves each axis independently between waypoints, so its
  path leaves the polyline (see polyline dev) even though it stops at every
  waypoint.
- jb-*-hybrid rounds corners; the deviation is bounded and reported by
  `Trajectory::maxCornerDeviation()`.
- toppra-spline follows a smooth spline through all waypoints without
  stopping: shortest durations, but an unconstrained-jerk, non-polyline
  path.
"""


def render(dt: float) -> str:
    os.makedirs(PLOTS_DIR, exist_ok=True)
    lines = [INSTRUCTIONS]
    for sc in scenarios():
        sc.dt = dt
        results = run_scenario(sc)
        lines.append(f"\n### {sc.name} ({sc.W.shape[0]} waypoints, {sc.W.shape[1]} dof")
        lines.append(f", blend_radius={sc.blend_radius})\n\n")
        lines.append("| method | duration [s] | compute [ms] | vel | acc | jerk | polyline dev | wp miss |\n")
        lines.append("|---|---|---|---|---|---|---|---|\n")
        for r in results:
            if r.t is None:
                lines.append(f"| {r.name} | - | - | - | - | - | - | - |\n")
                continue
            vel, acc, jerk = limit_ratios(r, sc)
            dev = polyline_deviation(r.q, sc.W)
            miss = waypoint_miss(r.q, sc.W)
            jerk_s = "-" if np.isnan(jerk) else f"{jerk:.2f}"
            lines.append(
                f"| {r.name} | {r.duration:.3f} | {r.compute_ms:.2f} | {vel:.3f} | {acc:.3f} "
                f"| {jerk_s} | {dev:.4f} | {miss:.4f} |\n"
            )
        try:
            png = os.path.join(PLOTS_DIR, f"{sc.name}.png")
            plot_scenario(sc, results, png)
            lines.append(f"\n![{sc.name}](plots/{sc.name}.png)\n")
        except ImportError:
            lines.append("\n(matplotlib not installed; plots skipped)\n")
    lines.append("\n" + NOTES)
    return "".join(lines)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dt", type=float, default=1e-3, help="sampling step for metrics [s]")
    ap.add_argument(
        "--output", type=str, default=os.path.join(HERE, "README.md"), help="markdown report destination"
    )
    args = ap.parse_args()

    doc = render(args.dt)
    with open(args.output, "w") as f:
        f.write(doc)
    print(doc)
    print(f"[written to {args.output}]")


if __name__ == "__main__":
    main()
