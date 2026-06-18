# Inverse Dynamics Derivatives Benchmarks

Performance benchmarks for `firstOrderInverseDynamicsDerivatives` and related algorithms in GRBDA. Each executable is self-contained with its own `main()`, links only against the `grbda` library, and is built when `BUILD_BENCHMARKS=ON`.

---

## Benchmarks

### `benchmarkIDDerivatives`

Measures the average execution time of `firstOrderInverseDynamicsDerivatives` across a representative set of robots. Runs 1000 timed iterations after a warmup call (which also triggers CasADi JIT compilation for configuration-dependent joints). Reports average time in microseconds and exports results to `benchmark_figures/data/robot_performance.csv`.

**Robots covered:** MiniCheetah (with/without rotors), MIT Humanoid (with/without rotors), Tello factorial design (`-R/-M`, `+R/-M`, `+R/+M`), TelloWithArms, KUKA LWR 4+, Cassie.

---

### `benchmarkIDDerivativesComparison`

Compares `firstOrderInverseDynamicsDerivatives` (standard, body-frame) against `firstOrderInverseDynamicsDerivativesWorldFrame` side-by-side. For each model, both variants are timed at 5000 iterations and a correctness check is performed (relative error threshold 1e-6 for most robots, 0.1 for floating-base robots with implicit constraints). Reports speedup as `standard_time / world_frame_time`.

**Models covered:** RevoluteChain (1-body clusters, N=4–20), RevolutePairChain (2-body clusters, N=2–8), RevoluteTripleChain (3-body clusters, N=3–9), Tello variants, TeleopArm, MiniCheetah, MIT Humanoid, and several URDF-parsed robots (JVRC1, Kuka LWR 4+).

---

### `benchmarkIDDerivativesScaling`

Measures how computation time and numerical accuracy scale with the number of links, for serial and binary-tree topologies. Validates analytical derivatives against central-difference finite differences (h=1e-7), using the constraint Jacobian G to project perturbations onto the constraint manifold for implicit joints. Exports results to CSV.

**Topologies:** Serial chains (RevoluteChainWithRotor, RevolutePairChainWithRotor, RevoluteTripleChainWithRotor) and binary-branching trees, swept across increasing link counts. Also tests specific robots (Tello, MiniCheetah, MIT Humanoid).

---

### `benchmarkIDDerivativesBreakdown`

Fine-grained profiling that breaks total cost into six buckets:

| Bucket | What it measures |
|---|---|
| `fwd_kin` | `forwardAccelerationKinematics` |
| `fwd_casadi` | `evalSTimesVec_dq` (×2) + `getSdotqd_q` per cluster (config-dependent S joints only) |
| `fwd_other` | Remaining forward pass: Ψ̇, Ψ̈, Υ̇, M_cup, B_cup, F |
| `bwd_casadi` | `evalSTTimesVec_dq` on the diagonal (config-dependent S joints only) |
| `bwd_other` | t1–t4 setup + walk-to-root block fills |
| `bwd_prop` | `accumulateBlockDiagonalPair` + `inverseTransformForceVector` to propagate to parent cluster |

Calls `enableIDDerivativesProfiling()` / `getIDDerivativesProfilingData()` / `resetIDDerivativesProfiling()` from the grbda profiling API. Exports per-robot breakdown to `Benchmarking/data/fig4_performance_breakdown_current.csv` (used for figure generation).

**Robots covered:** KUKA LWR 4+, MiniCheetah (±R), MIT Humanoid (±R), Tello (-R/-M, +R/-M, +R/+M), TelloWithArms, Cassie.

---

### `benchmarkComplexJointChains`

Investigates how per-cluster joint complexity interacts with the number of clusters. Compares RevoluteChainWithRotor (1 DOF/cluster) against RevolutePairChainWithRotor (2 DOF/cluster) at equal DOF counts (6 and 12 DOF). Reports per-cluster cost and fits an empirical time-complexity exponent O(n^α) from the smallest-to-largest chain ratio.

Also validates analytical ID derivatives against central-difference finite differences, with constraint-manifold-aware perturbations for implicit joints.

---

### `benchmarkCRBAComparison`

Parallel to `benchmarkIDDerivativesComparison` but for the Composite Rigid-Body Algorithm (CRBA). Compares `runStandardCRBA` against `runWorldFrameCRBA` at 10000 iterations. Verifies that both produce the same mass matrix H (relative error threshold 1e-10). Reports DOF, body count, times, and speedup.

**Models covered:** Same sweep as `benchmarkIDDerivativesComparison` — RevoluteChain, RevolutePairChain, RevoluteTripleChain, Tello variants, TeleopArm, MiniCheetah, MIT Humanoid, JVRC1, Kuka LWR 4+.

---

### `benchmarkParallelChainDepth`

Measures how ID derivative cost varies with loop size in A-shaped parallel chain topologies. Loads pre-built URDF files from `Benchmarking/urdfs/parallel_chains/Implicit/` (depths 5, 10, 20, 40; loop sizes determined by connection depth via `loop_size = 2 * connection_depth + 1`). Each configuration is compared against a no-loop baseline. Uses a fixed random seed (42) for reproducibility.

---

## Build

- Create a build directory: `mkdir build && cd build`
- Run CMake: `cmake ..`
- Build: `make`
- Each benchmark can be run individually with `./bin/<benchmark-name>`, e.g. `./bin/benchmarkIDDerivatives`

## Output

All benchmarks print a formatted table to stdout. Four of them additionally write CSV files:

- `benchmarkIDDerivatives` → `benchmark_figures/data/robot_performance.csv`
  Columns: `robot_name, label, dof, time_us`

- `benchmarkIDDerivativesScaling` → `benchmark_figures/data/serial_chain_scaling.csv`, `binary_tree_scaling.csv`, `complex_joint_scaling.csv`
  Columns: `topology, joint_type, num_links, dof, time_us, max_err_dq, max_err_dqd`

- `benchmarkIDDerivativesBreakdown` → `Benchmarking/data/fig4_performance_breakdown_current.csv`
  Columns: `robot_name, label, dof, bodies, fwd_kin_us, fwd_casadi_us, fwd_other_us, bwd_casadi_us, bwd_other_us, bwd_prop_us, total_us`

- `benchmarkParallelChainDepth` → `Benchmarking/data/parallel_chain_depth.csv` and `loop_depth_sweep.csv`
  Columns: `chain_depth, loop_size, connection_depth, dof, num_bodies, is_baseline, ...` (per-pass timing and profiling breakdown)
