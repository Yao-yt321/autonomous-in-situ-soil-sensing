# Adaptive penetration manuscript-to-code map

This module corresponds to Section 2.3 of the manuscript.

| Manuscript element | Implementation |
|---|---|
| No-contact torque referencing, Eqs. (20) and (23) | `src/axial_force_estimator.cpp` |
| Joint-3/Joint-4 calibration inversion, Eqs. (22) and (24) | `src/axial_force_estimator.cpp` |
| Covariance-weighted fusion, Eq. (25) | `src/axial_force_estimator.cpp` |
| Three-sample moving average, Eq. (26) | `src/adaptive_penetration_controller.cpp` — `movingAverage` |
| Touchdown confirmation, Eq. (27) | `src/adaptive_penetration_controller.cpp` — `update` |
| Penetration reference `Zref`, Eq. (28) | `src/adaptive_penetration_controller.cpp` — `update` |
| Actual penetration depth, Eq. (29) | `src/adaptive_penetration_controller.cpp` — `update` |
| Adaptive feed-speed regulation, Eq. (30) | `src/adaptive_penetration_controller.cpp` — `adaptiveFeedSpeed` |
| Depth/force termination states, Eq. (31) | `src/adaptive_penetration_controller.cpp` — `update` |
| Trigger/terminal depth bookkeeping, Eq. (32) | `src/adaptive_penetration_controller.cpp` — `latchTrigger` and `makeOutput` |
| Calibration, model selection, covariance fusion, nested LOOCV | `calibration/fit_force_estimator.py` |
