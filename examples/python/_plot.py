"""Plot helper for the Python examples.
"""

from __future__ import annotations

import numpy as np


def plot_trajectory(traj, v_max, a_max, j_max=None, title="", dt=0.005):
    import matplotlib.pyplot as plt

    r = traj.samples(dt=dt)
    t = np.asarray(r.t)
    q = np.asarray(r.q)
    qd = np.asarray(r.qd)
    qdd = np.asarray(r.qdd)

    n_dim = q.shape[1]
    is_scurve = j_max is not None
    n_rows = 4 if is_scurve else 3

    fig, ax = plt.subplots(n_rows, 1, figsize=(10, 2.2 * n_rows), sharex=True)
    if title:
        fig.suptitle(title)

    qddd = None
    if n_rows == 4 and len(t) > 1:
        qddd = np.gradient(qdd, t[1] - t[0], axis=0)

    for j in range(n_dim):
        ax[0].plot(t, q[:, j], label=f"dim {j}")
        ax[1].plot(t, qd[:, j])
        ax[2].plot(t, qdd[:, j])
        if qddd is not None:
            ax[3].plot(t, qddd[:, j])

    for j in range(n_dim):
        ax[1].axhline(v_max[j], color=f"C{j}", alpha=0.2, linestyle=":")
        ax[1].axhline(-v_max[j], color=f"C{j}", alpha=0.2, linestyle=":")
        ax[2].axhline(a_max[j], color=f"C{j}", alpha=0.2, linestyle=":")
        ax[2].axhline(-a_max[j], color=f"C{j}", alpha=0.2, linestyle=":")
        if qddd is not None and j_max is not None:
            ax[3].axhline(j_max[j], color=f"C{j}", alpha=0.2, linestyle=":")
            ax[3].axhline(-j_max[j], color=f"C{j}", alpha=0.2, linestyle=":")

    t_wp = np.concatenate(([0.0], np.cumsum([s.duration for s in traj.segments()])))
    for tw in t_wp:
        for a in ax:
            a.axvline(tw, color="k", alpha=0.15, linestyle="--")

    ax[0].set_ylabel("q")
    ax[1].set_ylabel("qd")
    ax[2].set_ylabel("qdd")
    if n_rows == 4:
        ax[3].set_ylabel("qddd")
    ax[-1].set_xlabel("t [s]")
    ax[0].legend(loc="best")
    plt.tight_layout()
    return fig
