#pragma once

#include <limits>
#include <stdexcept>

namespace adaptive_penetration {

struct AxialForceEstimate {
    double joint3_force_n = 0.0;
    double joint4_force_n = 0.0;
    double fused_force_n = 0.0;
    bool within_calibration_range = true;
};

class AxialForceEstimator {
public:
    struct Config {
        // Reference torques are measured with the complete sensor assembly mounted,
        // at the penetration pose, without external contact (paper Eq. 20/23).
        double joint3_reference_torque_nm = std::numeric_limits<double>::quiet_NaN();
        double joint4_reference_torque_nm = std::numeric_limits<double>::quiet_NaN();

        // Eq. (22):
        // dTau3 = 0.038872 F + 0.000347 F^2
        // dTau4 = -0.017294 F
        double joint3_linear_coefficient = 0.038872;
        double joint3_quadratic_coefficient = 0.000347;
        double joint4_linear_coefficient = -0.017294;

        // Eq. (25)
        double joint3_weight = 0.453;
        double joint4_weight = 0.547;

        double calibration_force_max_n = 73.575;
    };

    explicit AxialForceEstimator(const Config& config);

    AxialForceEstimate estimate(double joint3_torque_nm,
                                double joint4_torque_nm) const;
    const Config& config() const { return config_; }

private:
    Config config_;
};

}  // namespace adaptive_penetration
