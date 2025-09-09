
"""
First Test with UWB ranging ...

This code will only includes odom as a baseline
(Direct translation from the original notebook — functionality unchanged.)
"""

import gtsam
import numpy as np
import matplotlib.pyplot as plt

from common.noise_models import ODOMETRY_NOISE, PRIOR_NOISE_LOOSE
from common.plotting import plot_three_local_graphs


def add_local_graph_1(graph: gtsam.NonlinearFactorGraph, initial: gtsam.Values) -> None:
    """Graph 1: horizontal chain, prior at key=1."""
    prior_mean = gtsam.Pose2(0.0, 0.0, 0.0)
    graph.add(gtsam.PriorFactorPose2(1, prior_mean, PRIOR_NOISE_LOOSE))

    odometry = gtsam.Pose2(2.0, 0.0, 0.0)
    graph.add(gtsam.BetweenFactorPose2(1, 2, odometry, ODOMETRY_NOISE))
    graph.add(gtsam.BetweenFactorPose2(2, 3, odometry, ODOMETRY_NOISE))
    graph.add(gtsam.BetweenFactorPose2(3, 4, odometry, ODOMETRY_NOISE))
    graph.add(gtsam.BetweenFactorPose2(4, 5, odometry, ODOMETRY_NOISE))

    # Initial guesses (intentionally off)
    initial.insert(1, gtsam.Pose2(0.5, 0.0, 0.2))
    initial.insert(2, gtsam.Pose2(2.3, 0.1, -0.2))
    initial.insert(3, gtsam.Pose2(4.1, 0.1, -0.1))
    initial.insert(4, gtsam.Pose2(5.9, -0.2, 0.1))
    initial.insert(5, gtsam.Pose2(8.1, 0.01, 0.14))


def add_local_graph_2(graph: gtsam.NonlinearFactorGraph, initial: gtsam.Values) -> None:
    """Graph 2: interleaved vertical/horizontal steps, prior at key=100."""
    prior_mean = gtsam.Pose2(0.0, 0.0, 0.0)
    graph.add(gtsam.PriorFactorPose2(100, prior_mean, PRIOR_NOISE_LOOSE))

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
    """Graph 3: vertical steps downward + horizontal, prior at key=1000."""
    prior_mean = gtsam.Pose2(0.0, 0.0, 0.0)
    graph.add(gtsam.PriorFactorPose2(1000, prior_mean, PRIOR_NOISE_LOOSE))

    graph.add(gtsam.BetweenFactorPose2(1000, 2000, gtsam.Pose2(0.0, -2.0, 0.0), ODOMETRY_NOISE))
    graph.add(gtsam.BetweenFactorPose2(2000, 3000, gtsam.Pose2(2.0, 0.0, 0.0), ODOMETRY_NOISE))
    graph.add(gtsam.BetweenFactorPose2(3000, 4000, gtsam.Pose2(0.0, -2.0, 0.0), ODOMETRY_NOISE))
    graph.add(gtsam.BetweenFactorPose2(4000, 5000, gtsam.Pose2(2.0, 0.0, 0.0), ODOMETRY_NOISE))

    initial.insert(1000, gtsam.Pose2(0.3, 0.0, 0.2))
    initial.insert(2000, gtsam.Pose2(-0.1, -2.1, -0.2))
    initial.insert(3000, gtsam.Pose2(2.1, -2.3, -0.1))
    initial.insert(4000, gtsam.Pose2(1.9, -4.2, 0.1))
    initial.insert(5000, gtsam.Pose2(4.1, -4.01, 0.14))


def main() -> None:
    # Build the factor graph
    graph = gtsam.NonlinearFactorGraph()
    initial = gtsam.Values()

    add_local_graph_1(graph, initial)
    add_local_graph_2(graph, initial)
    add_local_graph_3(graph, initial)

    # Levenberg–Marquardt optimization
    params = gtsam.LevenbergMarquardtParams()
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
    result = optimizer.optimize()

    marginals = gtsam.Marginals(graph, result)

    # Print the same legend as in the notebook
    print("\nThis is the result for 3 local odom only solutions")
    print("\nG1 --> RED")
    print("G2 --> Green")
    print("G3 --> Blue")
    print("\n \n ")
    print("\nOdom only local graph optimization")

    # Plot
    plot_three_local_graphs(result, marginals, "plots/uwb_example1_graph.png")


if __name__ == "__main__":
    main()
