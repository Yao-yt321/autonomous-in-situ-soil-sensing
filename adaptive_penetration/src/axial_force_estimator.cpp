#include "adaptive_penetration/axial_force_estimator.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace adaptive_penetration {

AxialForceEstimator::AxialForceEstimator(const Config& config) : config_(config) {
    if (!std::isfinite(config_.joint3_reference_torque_nm) ||
        !std::isfinite(config_.joint4_reference_torque_nm)) {
        throw std::invalid_argument(
            "Joint 3 and Joint 4 reference torques must be provided explicitly.");
    }
    if (!(config_.joint3_linear_coefficient > 0.0) ||
        !(config_.joint3_quadratic_coefficient > 0.0)) {
        throw std::invalid_argument(
            "Joint 3 calibration coefficients must define a positive monotonic mapping.");
    }
    if (!(config_.joint4_linear_coefficient < 0.0)) {
        throw std::invalid_argument(
            "Joint 4 calibration coefficient must be negative for the published mapping.");
    }
    if (std::abs((config_.joint3_weight + config_.joint4_weight) - 1.0) > 1e-9 ||
        config_.joint3_weight < 0.0 || config_.joint4_weight < 0.0) {
        throw std::invalid_argument("Fusion weights must be nonnegative and sum to one.");
    }
    if (!(config_.calibration_force_max_n > 0.0)) {
        throw std::invalid_argument("Calibration force limit must be positive.");
    }
}

AxialForceEstimate AxialForceEstimator::estimate(double joint3_torque_nm,
                                                double joint4_torque_nm) const
{
    if (!std::isfinite(joint3_torque_nm) || !std::isfinite(joint4_torque_nm)) {
        throw std::invalid_argument("Joint torque input must be finite.");
    }

    const double joint3_torque_increment_nm =
        joint3_torque_nm - config_.joint3_reference_torque_nm;
    const double joint4_torque_increment_nm =
        joint4_torque_nm - config_.joint4_reference_torque_nm;

    // Inverse of Eq. (22) for Joint 3. The paper specifies selecting the
    // nonnegative real root. Negative/no-contact noise is projected to F=0.
    const double quadratic_coefficient = config_.joint3_quadratic_coefficient;
    const double linear_coefficient = config_.joint3_linear_coefficient;
    const double discriminant = linear_coefficient * linear_coefficient
        + 4.0 * quadratic_coefficient * joint3_torque_increment_nm;
    double joint3_force_n = 0.0;
    if (discriminant >= 0.0) {
        const double nonnegative_root =
            (-linear_coefficient + std::sqrt(discriminant))
            / (2.0 * quadratic_coefficient);
        if (nonnegative_root > 0.0) joint3_force_n = nonnegative_root;
    }

    // Joint 4: dtau4 = -0.017294 F.
    double joint4_force_n =
        joint4_torque_increment_nm / config_.joint4_linear_coefficient;
    if (joint4_force_n < 0.0) joint4_force_n = 0.0;

    const double fused_force_n = config_.joint3_weight * joint3_force_n +
        config_.joint4_weight * joint4_force_n;

    AxialForceEstimate out;
    out.joint3_force_n = joint3_force_n;
    out.joint4_force_n = joint4_force_n;
    out.fused_force_n = fused_force_n;
    out.within_calibration_range =
        (joint3_force_n <= config_.calibration_force_max_n &&
         joint4_force_n <= config_.calibration_force_max_n &&
         fused_force_n <= config_.calibration_force_max_n);
    return out;
}

}  // namespace adaptive_penetration
