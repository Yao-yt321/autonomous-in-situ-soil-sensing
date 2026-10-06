#include "adaptive_penetration/adaptive_penetration_controller.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace adaptive_penetration;

namespace {

int failures = 0;

void check(bool condition, const std::string& name)
{
    if (!condition) {
        ++failures;
        std::cerr << "[FAIL] " << name << '\n';
    } else {
        std::cout << "[PASS] " << name << '\n';
    }
}

bool near(double a, double b, double tolerance)
{
    return std::fabs(a - b) <= tolerance;
}

void torquesFromForce(double force_n,
                      const AxialForceEstimator::Config& config,
                      PenetrationInput& input)
{
    input.joint3_torque_nm = config.joint3_reference_torque_nm
        + config.joint3_linear_coefficient * force_n
        + config.joint3_quadratic_coefficient * force_n * force_n;
    input.joint4_torque_nm = config.joint4_reference_torque_nm
        + config.joint4_linear_coefficient * force_n;
}

PenetrationOutput advanceToAdaptivePenetration(
    AdaptivePenetrationController& controller,
    const AxialForceEstimator::Config& force_config,
    PenetrationInput& input)
{
    for (int i = 0; i < 3; ++i) {
        torquesFromForce(5.0, force_config, input);
        controller.update(input);
    }

    PenetrationOutput output = controller.update(input);
    while (output.state == PenetrationState::TouchdownCandidate) {
        output = controller.update(input);
    }
    return output;
}

}  // namespace

int main()
{
    bool missing_reference_rejected = false;
    try {
        AxialForceEstimator::Config incomplete_config;
        AxialForceEstimator invalid_estimator(incomplete_config);
        (void)invalid_estimator;
    } catch (const std::invalid_argument&) {
        missing_reference_rejected = true;
    }
    check(missing_reference_rejected,
          "reference torques must be supplied explicitly");

    AxialForceEstimator::Config force_config;
    force_config.joint3_reference_torque_nm = 0.8;
    force_config.joint4_reference_torque_nm = -1.2;

    AxialForceEstimator estimator(force_config);
    for (double force_n : {0.0, 5.0, 20.0, 66.7, 73.575}) {
        PenetrationInput input;
        torquesFromForce(force_n, force_config, input);
        const AxialForceEstimate estimate = estimator.estimate(
            input.joint3_torque_nm, input.joint4_torque_nm);
        check(near(estimate.joint3_force_n, force_n, 1e-8),
              "Joint-3 inverse calibration at " + std::to_string(force_n) + " N");
        check(near(estimate.joint4_force_n, force_n, 1e-8),
              "Joint-4 inverse calibration at " + std::to_string(force_n) + " N");
        check(near(estimate.fused_force_n, force_n, 1e-8),
              "covariance-weighted fusion at " + std::to_string(force_n) + " N");
    }

    AdaptivePenetrationController::Config controller_config;
    const std::size_t expected_touchdown_samples = static_cast<std::size_t>(
        std::ceil(controller_config.touchdown_confirmation_s
                  * controller_config.sample_frequency_hz));
    check(expected_touchdown_samples == 13,
          "0.10 s touchdown confirmation at 125 Hz equals 13 samples");

    AdaptivePenetrationController controller(controller_config, force_config);
    PenetrationInput input;
    input.tcp_z_m = 0.300;

    for (int i = 0; i < 3; ++i) {
        torquesFromForce(0.2, force_config, input);
        const PenetrationOutput output = controller.update(input);
        check(output.state == PenetrationState::Approach,
              "pre-contact state remains approach");
        check(near(output.feed_speed_mps, 0.30, 1e-12),
              "pre-contact feed speed is 0.30 m/s");
    }

    torquesFromForce(6.0, force_config, input);
    controller.update(input);
    controller.update(input);
    PenetrationOutput output = controller.update(input);
    check(output.touchdown_reference_valid,
          "first sustained threshold crossing latches penetration reference");
    check(output.state == PenetrationState::TouchdownCandidate,
          "touchdown enters confirmation state");
    check(near(output.z_ref_m, input.tcp_z_m, 1e-12),
          "Zref equals TCP Z at first threshold crossing");
    check(near(output.feed_speed_mps, 0.10, 1e-12),
          "touchdown-candidate speed is 0.10 m/s");

    while (output.state == PenetrationState::TouchdownCandidate) {
        output = controller.update(input);
    }
    check(output.state == PenetrationState::AdaptivePenetration,
          "sustained touchdown transitions to adaptive penetration");
    check(output.touchdown_confirm_count >= expected_touchdown_samples,
          "touchdown confirmation sample count is satisfied");

    AdaptivePenetrationController controller2(controller_config, force_config);
    PenetrationInput input2;
    input2.tcp_z_m = 0.300;
    PenetrationOutput output2 = advanceToAdaptivePenetration(
        controller2, force_config, input2);
    check(output2.state == PenetrationState::AdaptivePenetration,
          "helper reaches adaptive penetration state");
    check(near(output2.feed_speed_mps, 0.10, 1e-8),
          "Eq. (30) gives maximum feed speed near touchdown force");

    for (int i = 0; i < 3; ++i) {
        torquesFromForce(66.0, force_config, input2);
        output2 = controller2.update(input2);
    }
    check(output2.state == PenetrationState::AdaptivePenetration,
          "force below Fmax does not terminate penetration");
    check(output2.feed_speed_mps >= 0.03 && output2.feed_speed_mps < 0.031,
          "Eq. (30) approaches minimum feed speed near Fmax");

    input2.tcp_z_m = output2.z_ref_m - 0.1001;
    torquesFromForce(20.0, force_config, input2);
    for (int i = 0; i < 3; ++i) {
        output2 = controller2.update(input2);
    }
    check(output2.state == PenetrationState::StoppingNormal,
          "target depth triggers normal completion");
    check(output2.trigger_valid && !output2.safety_trigger,
          "normal completion is recorded as a non-safety trigger");

    torquesFromForce(70.0, force_config, input2);
    for (int i = 0; i < 3; ++i) {
        output2 = controller2.update(input2);
    }
    check(output2.state == PenetrationState::StoppingSafety,
          "safety condition escalates normal stopping to safety stopping");
    check(output2.safety_trigger,
          "safety priority is preserved during stopping");

    AdaptivePenetrationController controller3(controller_config, force_config);
    PenetrationInput input3;
    input3.tcp_z_m = 0.300;
    PenetrationOutput output3 = advanceToAdaptivePenetration(
        controller3, force_config, input3);
    input3.tcp_z_m = output3.z_ref_m - (controller_config.max_depth_m + 1e-6);
    torquesFromForce(20.0, force_config, input3);
    output3 = controller3.update(input3);
    check(output3.state == PenetrationState::StoppingSafety,
          "Hmax has safety priority over normal depth completion");
    check(output3.safety_trigger,
          "maximum-depth termination is recorded as safety termination");

    if (failures == 0) {
        std::cout << "All adaptive-penetration tests passed.\n";
        return 0;
    }

    std::cerr << failures << " test(s) failed.\n";
    return 1;
}
