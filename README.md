# Autonomous In Situ Soil Sensing — Research Code

Reference implementations of the two core control methods described in the manuscript **“Autonomous in situ soil sensing with location and penetration depth control”**:

- **DCA-MPC** for path tracking and stopping at target detection points;
- **adaptive probe penetration** for touchdown detection, joint-torque-based axial-force estimation, force-feedback feed-speed regulation, and depth/force-constrained termination.

The code is written for C++11 and is organized as two independent research modules.

## Repository structure

```text
.
├── DCA_MPC/
│   ├── include/dca_mpc/
│   ├── src/
│   ├── examples/
│   ├── tests/
│   └── docs/
├── adaptive_penetration/
│   ├── include/adaptive_penetration/
│   ├── src/
│   ├── calibration/
│   ├── examples/
│   ├── tests/
│   └── docs/
├── CMakeLists.txt
└── README.md
```

## Build

```bash
mkdir -p build
cd build
cmake ..
cmake --build . -j4
```

## Run the test suite

```bash
ctest --output-on-failure
```

The test suite covers DCA-MPC scheduling, vehicle-model linearization, MPC constraints, path-curvature handling, closed-loop numerical scenarios, adaptive penetration state transitions, axial-force estimation, termination priority, a complete adaptive-penetration control-cycle verification, and the offline calibration workflow.

## DCA-MPC examples

From the root build directory:

```bash
./DCA_MPC/dca_mpc_one_step
./DCA_MPC/dca_mpc_closed_loop
./DCA_MPC/dca_mpc_scenario_sweep
./DCA_MPC/dca_mpc_benchmark
```

See [`DCA_MPC/README.md`](DCA_MPC/README.md) for module details.

## Adaptive penetration examples

```bash
./adaptive_penetration/adaptive_penetration_one_step
./adaptive_penetration/adaptive_penetration_cycle_example
```

The calibration utility can be run from the repository root:

```bash
python3 adaptive_penetration/calibration/fit_force_estimator.py \
  adaptive_penetration/calibration/calibration_verification_data.csv \
  --output-dir build/calibration_demo
```

See [`adaptive_penetration/README.md`](adaptive_penetration/README.md) for module details.

## Dependencies

- CMake 3.10 or later
- a C++11 compiler
- Python 3 for the calibration utility

The core research modules use only the C++ standard library. The calibration utility uses only the Python standard library.

## Manuscript mapping

Equation- and section-level code maps are provided in:

- [`DCA_MPC/docs/PAPER_CODE_MAP.md`](DCA_MPC/docs/PAPER_CODE_MAP.md)
- [`adaptive_penetration/docs/PAPER_CODE_MAP.md`](adaptive_penetration/docs/PAPER_CODE_MAP.md)
