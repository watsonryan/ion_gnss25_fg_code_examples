
"""Matplotlib 3.9+-compatible plotting helpers for all examples.

We avoid `gtsam.utils.plot.plot_pose2` here to sidestep the Ellipse API change
in Matplotlib 3.9. This draws the covariance ellipse and a heading arrow directly.
"""
from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

try:
    import gtsam  # only for typing/pose accessors
except Exception:  # pragma: no cover
    gtsam = None  # not needed for simple import-time checks


def _draw_covariance_ellipse(ax, xy, cov_2x2, n_std: float = 1.0, **kwargs) -> None:
    """Draw an n-σ covariance ellipse given 2x2 covariance.

    Args:
        ax: Matplotlib axes.
        xy: (x, y) center.
        cov_2x2: 2x2 covariance for position.
        n_std: How many standard deviations (1.0 -> 1σ, ~2.447 -> 95% conf).
        kwargs: Matplotlib Patch kwargs (edgecolor, linewidth, alpha, etc.).
    """
    # Eigen-decomposition
    vals, vecs = np.linalg.eigh(cov_2x2)
    # Order largest -> smallest
    order = np.argsort(vals)[::-1]
    vals = vals[order]
    vecs = vecs[:, order]

    # Width/height are 2 * n_std * sqrt(eigenvalue)
    width = 2.0 * n_std * np.sqrt(max(vals[0], 0.0))
    height = 2.0 * n_std * np.sqrt(max(vals[1], 0.0))

    # Angle of the major axis in degrees
    angle = np.degrees(np.arctan2(vecs[1, 0], vecs[0, 0]))

    e = Ellipse(xy=xy, width=width, height=height, angle=angle, fill=False, **kwargs)
    ax.add_patch(e)


def _draw_pose_heading(ax, xy, theta, axis_len: float = 0.5, **kwargs) -> None:
    """Draw a simple heading arrow for a Pose2."""
    dx = axis_len * np.cos(theta)
    dy = axis_len * np.sin(theta)
    ax.arrow(
        xy[0], xy[1],
        dx, dy,
        head_width=0.15 * axis_len,
        head_length=0.25 * axis_len,
        length_includes_head=True,
        linewidth=1.5,
        **kwargs
    )


def _plot_pose2_with_cov(ax, pose2, axis_len: float, cov3x3) -> None:
    """Plot a Pose2 with a covariance ellipse on `ax`.

    This mirrors the essentials of `gtsam.utils.plot.plot_pose2` but uses the
    current Matplotlib Ellipse API to avoid version incompatibilities.
    """
    xy = (pose2.x(), pose2.y())
    theta = pose2.theta()

    # Draw 1-sigma ellipse from the 2x2 position covariance block
    if cov3x3 is not None:
        cov_2x2 = np.array([[cov3x3[0, 0], cov3x3[0, 1]],
                            [cov3x3[1, 0], cov3x3[1, 1]]], dtype=float)
        _draw_covariance_ellipse(ax, xy, cov_2x2, n_std=1.0, edgecolor='k', alpha=0.8)

    # Draw heading
    _draw_pose_heading(ax, xy, theta, axis_len=axis_len, color='k')


def plot_three_local_graphs(result, marginals, file_path) -> None:
    """Plot three local pose chains in red/green/blue with covariances.

    This matches the visualization produced by the original notebook, but
    uses our compatibility pose/covariance drawer.
    """
    color_dict = {1: 'red', 100: 'green', 1000: 'blue'}
    ax = plt.gca()

    for i in range(1, 6):
        for scale in (1, 100, 1000):
            key = i * scale
            pose = result.atPose2(key)
            cov = marginals.marginalCovariance(key)
            _plot_pose2_with_cov(ax, pose, axis_len=0.5, cov3x3=cov)

            # Connect to the next pose in the same local graph, if it exists
            try:
                p_now = pose
                p_next = result.atPose2((i + 1) * scale)
                ax.plot(
                    [p_now.x(),  p_next.x()],
                    [p_now.y(),  p_next.y()],
                    color=color_dict[scale],
                    linewidth=3,
                )
            except Exception:
                # The last pose has no (i+1) neighbor; match notebook behavior
                pass

    ax.axis('equal')
    plt.savefig(file_path)
