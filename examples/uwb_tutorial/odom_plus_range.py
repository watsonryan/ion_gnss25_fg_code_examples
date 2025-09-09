
"""
Second Test with UWB ranging ...

This code includes both odom and ranging measurements

For this example,

    Graph 1 starts at (0,0)
    Graph2 is offset at (-4.0, 2.0) w.r.t. G1
    Graph3 is offset at (-1.0, -4.0) w.r.t. G1
(Direct translation from the original notebook — functionality unchanged.)
"""

import gtsam
import numpy as np
import matplotlib.pyplot as plt

from common.noise_models import ODOMETRY_NOISE, PRIOR_NOISE_TIGHT, RANGE_NOISE
from common.plotting import plot_three_local_graphs


def add_local_graph_1(graph: gtsam.NonlinearFactorGraph, initial: gtsam.Values) -> None:
    """Graph 1: horizontal chain with a tight prior at key=1."""
    prior_mean = gtsam.Pose2(0.0, 0.0, 0.0)
    graph.add(gtsam.PriorFactorPose2(1, prior_mean, PRIOR_NOISE_TIGHT))

    odometry = gtsam.Pose2(2.0, 0.0, 0.0)
    graph.add(gtsam.BetweenFactorPose2(1, 2, odometry, ODOMETRY_NOISE))
    graph.add(gtsam.BetweenFactorPose2(2, 3, odometry, ODOMETRY_NOISE))
    graph.add(gtsam.BetweenFactorPose2(3, 4, odometry, ODOMETRY_NOISE))
    graph.add(gtsam.BetweenFactorPose2(4, 5, odometry, ODOMETRY_NOISE))

    initial.insert(1, gtsam.Pose2(0.5, 0.0, 0.2))
    initial.insert(2, gtsam.Pose2(2.3, 0.1, -0.2))
    initial.insert(3, gtsam.Pose2(4.1, 0.1, -0.1))
    initial.insert(4, gtsam.Pose2(5.9, -0.2, 0.1))
    initial.insert(5, gtsam.Pose2(8.1, 0.01, 0.14))


def add_local_graph_2(graph: gtsam.NonlinearFactorGraph, initial: gtsam.Values) -> None:
    """Graph 2: interleaved vertical/horizontal steps, *no prior* (as in notebook)."""
    graph.add(gtsam.BetweenFactorPose2(100, 200, gtsam.Pose2(0.0, 2.0, 0.0), ODOMETRY_NOISE))
    graph.add(gtsam.BetweenFactorPose2(200, 300, gtsam.Pose2(2.0, 0.0, 0.0), ODOMETRY_NOISE))
    graph.add(gtsam.BetweenFactorPose2(300, 400, gtsam.Pose2(0.0, 2.0, 0.0), ODOMETRY_NOISE))
    graph.add(gtsam.BetweenFactorPose2(400, 500, gtsam.Pose2(2.0, 0.0, 0.0), ODOMETRY_NOISE))

    initial.insert(100, gtsam.Pose2(0.3, 0.0, 0.2))
    initial.insert(200, gtsam.Pose2(-0.1, 2.1, -0.2))
    initial.insert(300, gtsam.Pose2(2.1, 2.3, -0.1))
    initial.insert(400, gtsam.Pose2(1.9, 4.2, 0.1))
    initial.insert(500, gtsam.Pose2(4.1, 4.01, 0.14))


def add_local_graph_3(graph: gtsam.NonlinearFactorGraph, initial: gtsam.Values) -> None:
    """Graph 3: vertical steps downward + horizontal, *no prior* (as in notebook)."""
    graph.add(gtsam.BetweenFactorPose2(1000, 2000, gtsam.Pose2(0.0, -2.0, 0.0), ODOMETRY_NOISE))
    graph.add(gtsam.BetweenFactorPose2(2000, 3000, gtsam.Pose2(2.0, 0.0, 0.0), ODOMETRY_NOISE))
    graph.add(gtsam.BetweenFactorPose2(3000, 4000, gtsam.Pose2(0.0, -2.0, 0.0), ODOMETRY_NOISE))
    graph.add(gtsam.BetweenFactorPose2(4000, 5000, gtsam.Pose2(2.0, 0.0, 0.0), ODOMETRY_NOISE))

    initial.insert(1000, gtsam.Pose2(0.3, 0.0, 0.2))
    initial.insert(2000, gtsam.Pose2(-0.1, -2.1, -0.2))
    initial.insert(3000, gtsam.Pose2(2.1, -2.3, -0.1))
    initial.insert(4000, gtsam.Pose2(1.9, -4.2, 0.1))
    initial.insert(5000, gtsam.Pose2(4.1, -4.01, 0.14))


def add_range_constraints(graph: gtsam.NonlinearFactorGraph) -> None:
    """Add inter-graph range constraints exactly as in the notebook."""
    # Nominal (true) 2D positions, used to generate ranges
    G1 = {
        1:  np.array([0.0, 0.0]),
        2:  np.array([2.0, 0.0]),
        3:  np.array([4.0, 0.0]),
        4:  np.array([6.0, 0.0]),
        5:  np.array([8.0, 0.0]),
    }

    G2_offset = np.array([-4.0,  2.0])
    G2 = {
        100: np.array([0.0, 0.0]) + G2_offset,
        200: np.array([0.0, 2.0]) + G2_offset,
        300: np.array([2.0, 2.0]) + G2_offset,
        400: np.array([2.0, 4.0]) + G2_offset,
        500: np.array([4.0, 4.0]) + G2_offset,
    }

    G3_offset = np.array([-1.0, -4.0])
    G3 = {
        1000: np.array([0.0,  0.0]) + G3_offset,
        2000: np.array([0.0, -2.0]) + G3_offset,
        3000: np.array([2.0, -2.0]) + G3_offset,
        4000: np.array([2.0, -4.0]) + G3_offset,
        5000: np.array([4.0, -4.0]) + G3_offset,
    }

    for k, v in G1.items():
        # g1(i) <-> g2(i)
        r1 = np.linalg.norm(v - G2[k * 100])
        graph.add(gtsam.RangeFactorPose2(k, k * 100, float(r1), RANGE_NOISE))

        # g1(i) <-> g3(i)
        r2 = np.linalg.norm(v - G3[k * 1000])
        graph.add(gtsam.RangeFactorPose2(k, k * 1000, float(r2), RANGE_NOISE))

        # g2(i) <-> g3(i)
        r3 = np.linalg.norm(G2[k * 100] - G3[k * 1000])
        graph.add(gtsam.RangeFactorPose2(k * 100, k * 1000, float(r3), RANGE_NOISE))


def main() -> None:
    graph = gtsam.NonlinearFactorGraph()
    initial = gtsam.Values()

    add_local_graph_1(graph, initial)
    add_local_graph_2(graph, initial)
    add_local_graph_3(graph, initial)
    add_range_constraints(graph)

    # Optimize
    params = gtsam.LevenbergMarquardtParams()
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
    result = optimizer.optimize()

    marginals = gtsam.Marginals(graph, result)

    print("\nThis is the result for 3 local with odom + range")
    print("\nG1 (0,0)--> RED")
    print("G2 (offset at (-4.0, 2.0) w.r.t. G1)--> Green")
    print("G3 (offset at (-1.0, -4.0) w.r.t. G1)--> Blue")
    print("\n \n ")
    print("\n Final Results for odom + ranging")

    plot_three_local_graphs(result, marginals, 'plots/uwb_example2_graph.png')


if __name__ == "__main__":
    main()
