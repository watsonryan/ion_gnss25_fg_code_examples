
# GTSAM UWB + Odometry — Detailed Tutorial

This tutorial shows how to model simple 2D mobile-robot scenarios as **factor graphs** and solve them with GTSAM.
The code mirrors a Colab notebook but is restructured into small, readable Python scripts for reuse in courses and demos.
Functionality is unchanged.

---

## 1. Background: Factor Graphs in a Nutshell

A **factor graph** encodes a probabilistic estimation problem. Unknown variables (nodes) are connected by
**factors** that encode measurements, constraints, or priors. Solving the graph means finding the most likely
variables given all factors (a nonlinear least squares problem).

In these examples the variables are 2D robot poses `Pose2(x, y, θ)` and the factors are:

- **PriorFactorPose2**: anchors a pose to a known mean with a Gaussian noise model.
- **BetweenFactorPose2**: encodes odometry increments (relative motion) between consecutive poses.
- **RangeFactorPose2**: encodes scalar distance measurements between two poses (acting as UWB-like constraints).

We use **Levenberg–Marquardt** optimization (LM) to minimize the sum of squared, whitened residuals. We also compute
marginal covariances for each pose for visualization (uncertainty ellipses).

---

## 2. Conventions and Notation

- Poses are keyed by integers.
  - Graph 1 (red): 1, 2, 3, 4, 5
  - Graph 2 (green): 100, 200, 300, 400, 500
  - Graph 3 (blue): 1000, 2000, 3000, 4000, 5000
- The **odometry increments** are mostly `Pose2(2.0, 0.0, 0.0)` (move 2 meters in x), with occasional vertical steps.
- We use the same **odometry noise** in all examples: diagonal with sigmas `[0.2, 0.2, 0.1]`.
- **Priors**:
  - Example 1: each local graph has a prior at its first pose (`1`, `100`, `1000`) with sigma `[0.3, 0.3, 0.1]`.
  - Example 2: only Graph 1 has a tight prior at `1` with sigma `[0.001, 0.001, 0.001]`.
  - Example 3: same tight prior at `1` for all three problem variants.
- **Initial guesses** are intentionally biased to make optimization visible.

---

## 3. Example 1 — Odometry Only (`examples/odom_only.py`)

### Goal
Build three **independent** pose chains, each with a prior on its first node. This sets a baseline before adding ranges.

### Graph Construction
- Add `PriorFactorPose2` at keys `1`, `100`, `1000` (looser prior noise).
- Add `BetweenFactorPose2` edges to form each chain:
  - Graph 1: `(1↔2↔3↔4↔5)` horizontal
  - Graph 2: vertical/horizontal interleaving `(100↔200↔300↔400↔500)`
  - Graph 3: vertical down/horizontal `(1000↔2000↔3000↔4000↔5000)`
- Insert **initial guesses** for every pose (slightly wrong on purpose).

### Solve & Visualize
- Optimize with LM.
- Compute marginals.
- Plot three graphs with **red/green/blue** colors and covariance ellipses.

Expected outcome: chains straighten out consistent with odometry and priors; uncertainty grows along each chain but
is bounded by priors.

---

## 4. Example 2 — Odometry + Ranging (`examples/odom_plus_range.py`)

### Goal
Link the three local chains using **inter-graph range factors**, while only Graph 1 has a prior at key `1`.
This emulates UWB-like pairwise distances and demonstrates **global alignment** via relative constraints.

### Offsets and "True" Positions (for measurement generation)
- Graph 1 starts at the origin `(0, 0)`.
- Graph 2 is offset by `(-4.0, 2.0)` relative to Graph 1.
- Graph 3 is offset by `(-1.0, -4.0)` relative to Graph 1.

We construct a small dictionary of **nominal 2D positions** (not variables) for each key in G1, G2, and G3,
then compute the exact distances between corresponding nodes:
- `range(1, 100)`, `range(2, 200)`, ..., `range(5, 500)`
- `range(1, 1000)`, ..., `range(5, 5000)`
- `range(100, 1000)`, ..., `range(500, 5000)`

These deterministic distances are then added as `RangeFactorPose2` constraints with scalar noise `[0.1]`.

### Solve & Visualize
- Optimize with LM using the single tight prior on key `1`.
- The ranges **tie** all three chains into a consistent global frame.
- Plot with the same red/green/blue convention.

---

## 5. Example 3 — Robust Ranging (`examples/robust_ranging.py`)

### Goal
Compare three variants:
1) **Ideal** ranges with standard L2 noise (no faults)
2) **Faulty** ranges with L2 (inject one biased range measurement)
3) **Faulty** ranges with a **robust** loss (Geman–McClure) to reduce outlier influence

### Setup
- Build three **separate** factor graphs (ideal, L2, robust) with identical odometry and priors.
- Add the same ranges to all graphs, but inject a **single biased range** on the pair where `k == 3`
  for the L2 and robust graphs.
- Use `noiseModel.Robust.Create(noiseModel.mEstimator.GemanMcClure(1.0), baseRangeNoise)`
  for the robust graph's range factors.

### Expectation
- **Ideal (L2)**: matches the deterministic ranges; consistent solution and small covariances.
- **Faulty (L2)**: a single bad range can noticeably warp the solution and inflate uncertainties.
- **Faulty (Robust)**: the robust loss **downweights** the outlier's impact, improving the solution.

### Note on Reproducibility
We intentionally do **not** set a random seed to preserve the original notebook's behavior. If you want repeatable
biased-noise draws for slides or papers, insert `np.random.seed(<int>)` before the biased range is added.

---

## 6. Plotting and Colors

- **Red**: keys `1..5`
- **Green**: keys `100..500`
- **Blue**: keys `1000..5000`
- Each pose is drawn with a marginal covariance ellipse (from `gtsam.Marginals`).
- Lines connect consecutive poses within each local graph.

---

## 7. Extending the Examples (optional)

- Add more poses or change the odometry step to see uncertainty growth.
- Vary noise sigmas to show how priors or ranges tighten/loosen the solution.
- Try other robust losses (e.g., Huber, Cauchy) to compare behavior on outliers.
