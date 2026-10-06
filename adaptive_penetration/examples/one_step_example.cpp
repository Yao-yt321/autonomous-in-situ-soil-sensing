#include "adaptive_penetration/adaptive_penetration_controller.h"

#include <iomanip>
#include <iostream>

using namespace adaptive_penetration;

namespace {

void setTorquesFromAxialForce(double axial_force_n,
                              const AxialForceEstimator::Config& config,
                              double& joint3_torque_nm,
                              double& joint4_torque_nm)
{
    joint3_torque_nm = config.joint3_reference_torque_nm
        + config.joint3_linear_coefficient * axial_force_n
        + config.joint3_quadratic_coefficient * axial_force_n * axial_force_n;
    joint4_torque_nm = config.joint4_reference_torque_nm
        + config.joint4_linear_coefficient * axial_force_n;
}

}  // namespace

int main()
{
    AxialForceEstimator::Config estimator_config;
    // Example-only reference torques. In a robot integration, these values are
    // measured in the no-contact penetration pose before a penetration sequence.
    estimator_config.joint3_reference_torque_nm = 0.80;
    estimator_config.joint4_reference_torque_nm = -1.20;

    AdaptivePenetrationController::Config controller_config;
    AdaptivePenetrationController controller(controller_config, estimator_config);

    PenetrationInput input;
    input.tcp_z_m = 0.250;

    // Fill the three-sample moving-average window with a representative contact load.
    for (int sample_index = 0; sample_index < 3; ++sample_index) {
        setTorquesFromAxialForce(
            5.5,
            estimator_config,
            input.joint3_torque_nm,
            input.joint4_torque_nm);

        const PenetrationOutput output = controller.update(input);
        std::cout << "k=" << sample_index
                  << " state=" << toString(output.state)
                  << " F_fused=" << std::fixed << std::setprecision(3)
                  << output.fused_force_n
                  << " N F_sm=" << output.smoothed_force_n
                  << " N speed=" << output.feed_speed_mps << " m/s\n";
    }

    return 0;
}
