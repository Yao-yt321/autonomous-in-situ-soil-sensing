# DCA-MPC

C++11 reference implementation of the **Distance-Curvature Adaptive Model Predictive Control (DCA-MPC)** method described in Section 2.2 of the manuscript **“Autonomous in situ soil sensing with location and penetration depth control.”**

## Implemented components

- four-wheel Ackermann equivalent two-axle kinematic model;
- forward-Euler discretization with `Ts = 0.10 s`;
- linear time-varying model linearization;
- prediction horizon `Np = 6` and control horizon `Nc = 2`;
- local-path-coordinate longitudinal, lateral, and heading errors;
- MPC state, control, and control-increment costs;
- speed, steering-angle, and control-increment constraints;
- equal-and-opposite front/rear steering;
- distance-curvature adaptation of look-ahead distance, reference speed, and `Q/R` weights;
- curvature-dependent speed upper bound;
- receding-horizon control with a compact dense QP solver.

## Main interface

```cpp
#include "dca_mpc/ltv_mpc.h"

using namespace dca_mpc;

LtvMpcController controller;
MpcStepResult result = controller.compute(
    robot_pose,
    previous_control,
    current_target,
    reference_path);
```

The returned `MpcStepResult` contains the optimized first control action, adaptive scheduling values, the active look-ahead point, and QP diagnostics.

## Build this module only

```bash
mkdir -p build
cd build
cmake ..
cmake --build . -j4
ctest --output-on-failure
```

## Examples

```bash
./dca_mpc_one_step
./dca_mpc_closed_loop
./dca_mpc_scenario_sweep
./dca_mpc_benchmark
```

## Code organization

```text
include/dca_mpc/
    dca_mpc_scheduler.h
    dense_qp.h
    ltv_mpc.h
    mpc_types.h
    reference_path.h
    vehicle_model.h
src/
examples/
tests/
docs/PAPER_CODE_MAP.md
```
