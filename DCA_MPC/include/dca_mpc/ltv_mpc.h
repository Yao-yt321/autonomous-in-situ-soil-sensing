#pragma once

#include "dca_mpc/dca_mpc_scheduler.h"
#include "dca_mpc/dense_qp.h"
#include "dca_mpc/reference_path.h"
#include "dca_mpc/vehicle_model.h"

#include <vector>

namespace dca_mpc {

struct MpcParameters {
    int prediction_horizon;     // N_p = 6
    int control_horizon;        // N_c = 2

    double min_speed;           // 0.0 m/s
    double max_speed;           // 1.0 m/s
    double max_steering;        // 8 deg, rad

    // Control-increment bounds used by this implementation.
    double max_delta_speed;     // m/s per control interval
    double max_delta_steering;  // rad per control interval

    MpcParameters();
};

struct MpcStepResult {
    bool solved;
    ControlInput command;
    AdaptiveSettings adaptive;
    ReferencePoint lookahead;
    int qp_iterations;
    double primal_residual;
    double dual_residual;

    // Diagnostics for reproducibility and integration checks.
    double projection_arc_length;
    double target_arc_length;
    bool fallback_used;

    MpcStepResult();
};

class LtvMpcController {
public:
    LtvMpcController();
    LtvMpcController(const MpcParameters& params,
                     const DenseQpSettings& qp_settings);

    // One receding-horizon control step. The target is the current target
    // detection point. previous_input is the command applied at k-1.
    MpcStepResult compute(const Pose2D& robot,
                          const ControlInput& previous_input,
                          const Pose2D& target,
                          const ReferencePath& path);

    const MpcParameters& parameters() const { return params_; }

    // Clear the QP warm start when a new path/task is loaded.
    void reset();

private:
    struct HorizonData {
        std::vector<ReferencePoint> ref_states; // size N_p + 1
        std::vector<ControlInput> ref_inputs;   // size N_p
        std::vector<LinearizedModel> models;    // size N_p
    };

    HorizonData buildHorizon(const ReferencePath& path,
                             const PathProjection& projection,
                             const AdaptiveSettings& adaptive,
                             double target_s) const;

    DenseQpProblem buildQp(const Pose2D& robot,
                           const ControlInput& previous_input,
                           const HorizonData& horizon,
                           const AdaptiveSettings& adaptive) const;

    static std::vector<double> controlToVector(const ControlInput& u);
    static ControlInput vectorToControl(const std::vector<double>& x, std::size_t offset);

    MpcParameters params_;
    VehicleModel model_;
    DcaMpcScheduler scheduler_;
    DenseAdmmQpSolver solver_;
    std::vector<double> warm_start_;
};

} // namespace dca_mpc
