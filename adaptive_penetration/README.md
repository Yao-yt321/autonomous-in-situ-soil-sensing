# Adaptive Probe Penetration

C++11 reference implementation of the **depth-prioritized and force-constrained adaptive probe penetration** method described in Section 2.3 of the manuscript **“Autonomous in situ soil sensing with location and penetration depth control.”**

## Implemented components

- Joints 3 and 4 axial-force estimation using the published calibration relations;
- covariance-weighted two-joint force fusion;
- three-sample moving-average force smoothing;
- touchdown detection and consecutive-sample confirmation;
- penetration reference determination from TCP axial position;
- penetration depth calculation from TCP pose feedback;
- force-feedback adaptive feed-speed regulation;
- normal completion at the target penetration depth;
- force and maximum-depth safety termination;
- safety-priority stopping logic;
- trigger depth, terminal depth, and stopping-response-deviation bookkeeping;
- sensing hold and retraction state transitions;
- offline calibration and nested-LOOCV utility in Python 3.

## Main interface

```cpp
#include "adaptive_penetration/adaptive_penetration_controller.h"

using namespace adaptive_penetration;

AxialForceEstimator::Config estimator_config;
estimator_config.joint3_reference_torque_nm = joint3_reference_torque_nm;
estimator_config.joint4_reference_torque_nm = joint4_reference_torque_nm;

AdaptivePenetrationController::Config controller_config;
AdaptivePenetrationController controller(controller_config, estimator_config);

PenetrationOutput output = controller.update(input);
```

The controller input contains Joints 3 and 4 torques, TCP Z, and state flags supplied by the robot integration layer. The output contains the penetration state, requested motion action, estimated axial force, penetration depth, feed speed, and termination diagnostics.

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
./adaptive_penetration_one_step
./adaptive_penetration_cycle_example
```

## Calibration utility

```bash
python3 calibration/test_calibration_tool.py
python3 calibration/fit_force_estimator.py \
  calibration/calibration_verification_data.csv \
  --output-dir build/calibration_demo
```

The calibration workflow includes six-joint response screening, robust zero-intercept model fitting, model-order selection, covariance-weighted fusion, and nested leave-one-load-level-out cross-validation.

## Units

- joint torque: N·m
- axial force: N
- TCP Z and penetration depth: m
- linear speed: m/s
- time: s

## Code organization

```text
include/adaptive_penetration/
    adaptive_penetration_controller.h
    axial_force_estimator.h
    penetration_types.h
src/
calibration/
examples/
tests/
docs/PAPER_CODE_MAP.md
```
