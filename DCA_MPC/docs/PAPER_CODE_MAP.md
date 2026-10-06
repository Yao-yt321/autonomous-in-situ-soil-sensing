# DCA-MPC manuscript-to-code map

This module corresponds to Section 2.2 of the manuscript.

| Manuscript element | Implementation |
|---|---|
| Continuous four-wheel-steering kinematic model, Eq. (1) | `src/vehicle_model.cpp` — `VehicleModel::derivative` |
| Euler discretization, Eqs. (2)–(3) | `src/vehicle_model.cpp` — `VehicleModel::step` |
| LTV linearization, Eqs. (4)–(6) | `src/vehicle_model.cpp` — `VehicleModel::linearize` |
| MPC objective and prediction model, Eq. (7) | `src/ltv_mpc.cpp` — `LtvMpcController::buildQp` |
| Input and increment constraints, Eq. (8) | `src/ltv_mpc.cpp` — `LtvMpcController::buildQp` |
| Reference-path heading and signed curvature, Eqs. (11)–(12) | `src/reference_path.cpp` |
| Target distance, Eq. (13) | `src/dca_mpc_scheduler.cpp` — `targetDistance` |
| Look-ahead adaptation, Table 1 | `src/dca_mpc_scheduler.cpp` — `lookaheadDistance` |
| Reference-speed adaptation, Eq. (14) | `src/dca_mpc_scheduler.cpp` — `referenceSpeed` |
| Local path-coordinate error, Eq. (15) | `include/dca_mpc/mpc_types.h` and `src/ltv_mpc.cpp` |
| Reference steering, Eq. (16) | `src/dca_mpc_scheduler.cpp` — `referenceControl` |
| Reference control sequence, Eq. (17) | `src/ltv_mpc.cpp` — `buildHorizon` |
| Adaptive `Q/R` and fixed `S`, Table 2 | `src/dca_mpc_scheduler.cpp` — `assignWeights` |
| Curvature-dependent speed bound, Eq. (18) | `src/dca_mpc_scheduler.cpp` and `src/ltv_mpc.cpp` |
| Receding-horizon first-input application | `src/ltv_mpc.cpp` — `compute` |
