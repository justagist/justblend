"""Regenerates the figures embedded in the top-level README.

Usage:
    pip install .[plot]
    python docs/generate_readme_figures.py
"""

from __future__ import annotations

import os
import sys

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

try:
    import justblend as jb
except ImportError:  # in-tree dev build fallback
    sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "python")))
    import justblend as jb

HERE = os.path.dirname(os.path.abspath(__file__))

BLUE = "#2a78d6"
GREEN = "#008300"
GREY = "#9a9994"
INK = "#52514e"


def make_trajectory(corner_handling, W, blend_radius=0.18):
    """Builds an S-curve trajectory over the demo waypoints.

    Args:
        corner_handling (jb.CornerHandling): Corner mode to use.
        W (np.ndarray): N x 2 waypoint matrix.
        blend_radius (float): Corner cut-back distance.
    """
    gen = jb.SCurveTrajectoryGenerator(dim=2)
    gen.set_limits(
        jb.Limits(v_max=np.array([1.5, 1.2]), a_max=np.array([3.0, 2.5]), j_max=np.array([20.0, 18.0]))
    )
    gen.set_options(jb.GenerationOptions(blend_radius=blend_radius, corner_handling=corner_handling))
    return gen.generate(W)


def blending_figure(path):
    """Renders the hero figure: blended vs strict path and speed profiles.

    Args:
        path (str): Output PNG path.
    """
    W = np.array([[0.0, 0.0], [1.0, 0.5], [1.5, -0.2], [0.6, -0.6], [0.0, 0.0]])
    strict = make_trajectory(jb.CornerHandling.STRICT_CORNERS, W)
    hybrid = make_trajectory(jb.CornerHandling.HYBRID, W)

    fig, (ax_path, ax_speed) = plt.subplots(1, 2, figsize=(11.0, 4.0), facecolor="white")

    ax_path.plot(W[:, 0], W[:, 1], "--", color=GREY, lw=1.2, label="waypoint polyline", zorder=1)
    ax_path.plot(W[:, 0], W[:, 1], "o", color=GREY, ms=6, zorder=2)
    q = hybrid.samples(0.002).q
    ax_path.plot(q[:, 0], q[:, 1], color=BLUE, lw=2.2, label="hybrid (Hermite blends)", zorder=3)
    ax_path.set_title("Blends round the corners without stopping", fontsize=10)
    ax_path.set_aspect("equal", adjustable="datalim")
    ax_path.legend(fontsize=8, frameon=False)
    ax_path.spines[["top", "right"]].set_visible(False)

    for traj, colour, label in ((strict, GREY, "strict corners"), (hybrid, BLUE, "hybrid blending")):
        r = traj.samples(0.002)
        speed = np.linalg.norm(np.asarray(r.qd), axis=1)
        ax_speed.plot(np.asarray(r.t), speed, color=colour, lw=2.0, label=f"{label} ({traj.duration():.2f} s)")
    ax_speed.set_title("Speed: no stops at interior waypoints", fontsize=10)
    ax_speed.set_xlabel("time [s]", fontsize=9)
    ax_speed.set_ylabel("|qd| [rad/s]", fontsize=9)
    ax_speed.legend(fontsize=8, frameon=False)
    ax_speed.spines[["top", "right"]].set_visible(False)
    ax_speed.tick_params(labelsize=8)

    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def profiles_figure(path):
    """Renders trapezoidal vs S-curve velocity/acceleration profiles.

    Args:
        path (str): Output PNG path.
    """
    W = np.array([[0.0], [1.0]])
    limits = {"v_max": np.array([1.0]), "a_max": np.array([3.0])}

    trap_gen = jb.TrapezoidalTrajectoryGenerator(dim=1)
    trap_gen.set_limits(jb.Limits(**limits))
    trap = trap_gen.generate(W)

    scurve_gen = jb.SCurveTrajectoryGenerator(dim=1)
    scurve_gen.set_limits(jb.Limits(**limits, j_max=np.array([30.0])))
    scurve = scurve_gen.generate(W)

    fig, (ax_v, ax_a) = plt.subplots(1, 2, figsize=(11.0, 3.4), facecolor="white")
    for traj, colour, label in ((trap, GREY, "trapezoidal"), (scurve, BLUE, "S-curve (jerk-limited)")):
        r = traj.samples(0.001)
        ax_v.plot(np.asarray(r.t), np.asarray(r.qd)[:, 0], color=colour, lw=2.0, label=label)
        ax_a.plot(np.asarray(r.t), np.asarray(r.qdd)[:, 0], color=colour, lw=2.0, label=label)
    ax_v.set_title("velocity", fontsize=10)
    ax_a.set_title("acceleration", fontsize=10)
    for ax in (ax_v, ax_a):
        ax.set_xlabel("time [s]", fontsize=9)
        ax.legend(fontsize=8, frameon=False)
        ax.spines[["top", "right"]].set_visible(False)
        ax.tick_params(labelsize=8)

    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


if __name__ == "__main__":
    blending_figure(os.path.join(HERE, "blending.png"))
    profiles_figure(os.path.join(HERE, "profiles.png"))
    print("wrote docs/blending.png and docs/profiles.png")
