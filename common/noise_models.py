
"""Noise model setup for the GTSAM UWB + Odometry examples.

Values match those used in the original notebook to preserve behavior.
"""
from __future__ import annotations

import numpy as np
import gtsam

# Odometry and prior noise used in the *first* example (odom-only)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))
PRIOR_NOISE_LOOSE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))

# Tighter prior noise used in the ranging examples (for graph 1)
PRIOR_NOISE_TIGHT = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.001, 0.001, 0.001]))

# Range noise (scalar)
RANGE_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1]))

# Robust range noise (Geman–McClure on top of the same base noise)
ROBUST_RANGE_NOISE = gtsam.noiseModel.Robust.Create(
    gtsam.noiseModel.mEstimator.GemanMcClure.Create(1.0),
    RANGE_NOISE,
)
