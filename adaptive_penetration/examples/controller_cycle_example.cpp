#include "adaptive_penetration/adaptive_penetration_controller.h"

#include <algorithm>
#include <iomanip>
#include <iostream>

using namespace adaptive_penetration;

namespace {

void setTorquesFromAxialForce(double axial_force_n,
                              const AxialForceEstimator::Config& config,
                              PenetrationInput& input)
{
    input.joint3_torque_nm = config.joint3_reference_torque_nm
        + config.joint3_linear_coefficient * axial_force_n
        + config.joint3_quadratic_coefficient * axial_force_n * axial_force_n;
    input.joint4_torque_nm = config.joint4_reference_torque_nm
        + config.joint4_linear_coefficient * axial_force_n;
}

}  // namespace

int main()
{
    AxialForceEstimator::Config estimator_config;
    estimator_config.joint3_reference_torque_nm = 0.80;
    estimator_config.joint4_reference_torque_nm = -1.20;

    AdaptivePenetrationController::Config controller_config;
    AdaptivePenetrationController controller(controller_config, estimator_config);

    PenetrationInput input;
    input.tcp_z_m = 0.300;
    const double sample_period_s = 1.0 / controller_config.sample_frequency_hz;

    PenetrationOutput output;
    int sample_index = 0;
    for (; sample_index < 10000; ++sample_index) {
        double axial_force_n = 0.2;
        if (sample_index >= 20 && sample_index < 40) axial_force_n = 5.5;

        if (output.state == PenetrationState::AdaptivePenetration) {
            axial_force_n = std::min(
                60.0,
                8.0 + 420.0 * output.penetration_depth_m);
        }

        if (output.state == PenetrationState::StoppingNormal ||
            output.state == PenetrationState::HoldForSensing ||
            output.state == PenetrationState::RetractToReference ||
            output.state == PenetrationState::RetractToSafeHeight ||
            output.state == PenetrationState::Completed) {
            axial_force_n = 20.0;
        }

        setTorquesFromAxialForce(axial_force_n, estimator_config, input);

        // Minimal deterministic plant model used to exercise the controller state sequence.
        if (output.action == MotionAction::FeedDown) {
            input.tcp_z_m -= output.feed_speed_mps * sample_period_s;
        }

        if (output.state == PenetrationState::StoppingNormal &&
            sample_index % 4 == 0) {
            input.arm_is_stationary = true;
        } else if (output.state != PenetrationState::StoppingNormal) {
            input.arm_is_stationary = false;
        }

        if (output.state == PenetrationState::HoldForSensing &&
            output.sensing_requested) {
            input.sensing_complete = true;
        }

        if (output.state == PenetrationState::RetractToReference) {
            input.tcp_z_m = output.z_ref_m;
            input.at_penetration_reference = true;
        } else {
            input.at_penetration_reference = false;
        }

        input.at_safe_height =
            (output.state == PenetrationState::RetractToSafeHeight);

        output = controller.update(input);
        if (output.state == PenetrationState::Completed) break;
    }

    std::cout << std::fixed << std::setprecision(6)
              << "completed: "
              << (output.state == PenetrationState::Completed ? "yes" : "no") << '\n'
              << "samples: " << sample_index + 1 << '\n'
              << "touchdown_confirm_samples: " << output.touchdown_confirm_count << '\n'
              << "H_trigger_m: " << output.trigger_depth_m << '\n'
              << "H_end_m: " << output.terminal_depth_m << '\n'
              << "Delta_H_stop_m: " << output.stopping_response_deviation_m << '\n'
              << "peak_estimated_force_N: " << output.peak_estimated_force_n << '\n';

    return output.state == PenetrationState::Completed ? 0 : 1;
}
