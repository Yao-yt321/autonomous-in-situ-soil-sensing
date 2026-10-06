#pragma once

#include "adaptive_penetration/axial_force_estimator.h"
#include "adaptive_penetration/penetration_types.h"

#include <cstddef>
#include <deque>

namespace adaptive_penetration {

class AdaptivePenetrationController {
public:
    struct Config {
        double sample_frequency_hz = 125.0;  // Section 2.3.2
        std::size_t moving_average_window = 3;  // Eq. (26)

        double touchdown_threshold_n = 5.0;
        double touchdown_confirmation_s = 0.10;

        double approach_speed_mps = 0.30;
        double touchdown_candidate_speed_mps = 0.10;
        double feed_speed_max_mps = 0.10;
        double feed_speed_min_mps = 0.03;

        double target_depth_m = 0.100;  // 10.0 cm
        double max_depth_m = 0.105;     // 10.5 cm
        double max_axial_force_n = 66.7;

        double sensing_hold_duration_s = 5.0;
        double retract_to_reference_speed_mps = 0.10;
        double retract_to_safe_height_speed_mps = 0.30;
    };

    AdaptivePenetrationController(const Config& config,
                                  const AxialForceEstimator::Config& estimator_config);

    void reset();
    PenetrationOutput update(const PenetrationInput& input);

    const Config& config() const { return config_; }
    const AxialForceEstimator& estimator() const { return estimator_; }

private:
    double movingAverage(double force_sample_n, bool& window_ready);
    double adaptiveFeedSpeed(double smoothed_force_n) const;
    void latchTrigger(bool safety_trigger, double depth_m);
    PenetrationOutput makeOutput(const AxialForceEstimate& estimate,
                                 double smoothed_force_n,
                                 bool filter_ready,
                                 double depth_m) const;

    Config config_;
    AxialForceEstimator estimator_;
    std::deque<double> force_window_;

    PenetrationState state_ = PenetrationState::Approach;
    std::size_t touchdown_required_samples_ = 1;
    std::size_t touchdown_count_ = 0;

    bool z_ref_valid_ = false;
    double z_ref_m_ = 0.0;

    bool trigger_valid_ = false;
    bool safety_trigger_ = false;
    double trigger_depth_m_ = 0.0;
    double terminal_depth_m_ = 0.0;
    bool terminal_depth_latched_ = false;
    double peak_force_n_ = 0.0;

    std::size_t hold_required_samples_ = 0;
    std::size_t hold_count_ = 0;
};

}  // namespace adaptive_penetration
