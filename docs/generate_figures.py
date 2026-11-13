"""Regenerates the figures embedded in the README and the docs.

Usage:
    pip install .[plot]
    python docs/generate_figures.py
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
RED = "#c4314b"


def make_trajectory(corner_handling, W, blend_radius=0.18):
    """Builds an S-curve trajectory over 2D waypoints with the demo limits.

    Args:
        corner_handling (jb.CornerHandling): Corner mode to use.
        W (np.ndarray): N x 2 waypoint matrix.
        blend_radius (float): Corner cut-back distance.
    """
    gen = jb.SCurveTrajectoryGenerator(dim=2)
    gen.set_limits(jb.Limits(v_max=np.array([1.5, 1.2]), a_max=np.array([3.0, 2.5]), j_max=np.array([20.0, 18.0])))
    gen.set_options(jb.GenerationOptions(blend_radius=blend_radius, corner_handling=corner_handling))
    return gen.generate(W)


def blending_figure(path):
    """Renders the hero figure: blended vs strict path and speed profiles.

    Args:
        path (str): Output PNG path.
    """
    W = np.array([[0.0, 0.0], [1.0, 0.5], [1.5, -0.2], [0.6, -0.6], [0.0, 0.0]])
    strict = make_trajectory(jb.CornerHandling.STRICT_CORNERS, W)
    blended = make_trajectory(jb.CornerHandling.BLEND_WHERE_FASTER, W)

    fig, (ax_path, ax_speed) = plt.subplots(1, 2, figsize=(11.0, 4.0), facecolor="white")

    ax_path.plot(W[:, 0], W[:, 1], "--", color=GREY, lw=1.2, label="waypoint polyline", zorder=1)
    ax_path.plot(W[:, 0], W[:, 1], "o", color=GREY, ms=6, zorder=2)
    q = blended.samples(0.002).q
    ax_path.plot(q[:, 0], q[:, 1], color=BLUE, lw=2.2, label="blend where faster (Hermite blends)", zorder=3)
    ax_path.set_title("Blends round the corners without stopping", fontsize=10)
    ax_path.set_aspect("equal", adjustable="datalim")
    ax_path.legend(fontsize=8, frameon=False)
    ax_path.spines[["top", "right"]].set_visible(False)

    for traj, colour, label in ((strict, GREY, "strict corners"), (blended, BLUE, "blend where faster")):
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


def corner_modes_figure(path):
    """Renders the four corner modes on one path, with their speed profiles.

    The path has a gentle corner where a blend is faster than stopping, a
    hairpin where the blend's speed cap makes it slower, and a spur that
    doubles back, which cannot be blended.

    Args:
        path (str): Output PNG path.
    """
    gentle, hairpin = np.radians(35.0), np.radians(35.0 + 135.0)
    p1 = np.array([1.5, 0.0])
    p2 = p1 + 1.5 * np.array([np.cos(gentle), np.sin(gentle)])
    p3 = p2 + 1.2 * np.array([np.cos(hairpin), np.sin(hairpin)])
    p4 = p3 - 0.4 * np.array([np.cos(hairpin), np.sin(hairpin)])
    W = np.array([[0.0, 0.0], p1, p2, p3, p4])
    radius = 0.15

    modes = (
        ("STRICT_CORNERS", jb.CornerHandling.STRICT_CORNERS, GREY),
        ("BLEND_ALL", jb.CornerHandling.BLEND_ALL, None),
        ("BLEND_WHERE_FEASIBLE", jb.CornerHandling.BLEND_WHERE_FEASIBLE, GREEN),
        ("BLEND_WHERE_FASTER", jb.CornerHandling.BLEND_WHERE_FASTER, BLUE),
    )
    fig = plt.figure(figsize=(11.0, 8.4), facecolor="white")
    grid = fig.add_gridspec(3, 2, height_ratios=(1.0, 1.0, 1.1))
    ax_speed = fig.add_subplot(grid[2, :])

    for i, (name, mode, colour) in enumerate(modes):
        ax = fig.add_subplot(grid[i // 2, i % 2])
        ax.plot(W[:, 0], W[:, 1], "--", color=GREY, lw=1.0, zorder=1)
        ax.plot(W[:, 0], W[:, 1], "o", color=GREY, ms=4, zorder=2)
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xticks([])
        ax.set_yticks([])
        ax.spines[["top", "right", "bottom", "left"]].set_visible(False)
        try:
            traj = make_trajectory(mode, W, radius)
        except ValueError as e:
            ax.plot(W[3, 0], W[3, 1], "x", color=RED, ms=11, mew=2.5, zorder=3)
            ax.set_title(f"{name}: ValidationError\n{str(e).split(';')[0]}", fontsize=9)
            continue
        q = traj.samples(0.002).q
        ax.plot(q[:, 0], q[:, 1], color=colour, lw=2.0, zorder=3)
        for w, kind in zip(W[1:-1], traj.corner_types()[1:-1]):
            if kind == jb.CornerType.STOP:
                ax.plot(w[0], w[1], "s", color=colour, ms=7, zorder=4)
        ax.set_title(f"{name}: {traj.duration():.2f} s", fontsize=9)

        r = traj.samples(0.002)
        speed = np.linalg.norm(np.asarray(r.qd), axis=1)
        ax_speed.plot(np.asarray(r.t), speed, color=colour, lw=2.0, label=f"{name} ({traj.duration():.2f} s)")

    ax_speed.set_title("Speed (squares on the paths mark stops)", fontsize=10)
    ax_speed.set_xlabel("time [s]", fontsize=9)
    ax_speed.set_ylabel("|qd|", fontsize=9)
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
    corner_modes_figure(os.path.join(HERE, "corner_modes.png"))
    profiles_figure(os.path.join(HERE, "profiles.png"))
    print("wrote docs/blending.png, docs/corner_modes.png and docs/profiles.png")
