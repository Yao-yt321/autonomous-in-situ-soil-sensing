#include "adaptive_penetration/adaptive_penetration_controller.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace adaptive_penetration {

AdaptivePenetrationController::AdaptivePenetrationController(
    const Config& config,
    const AxialForceEstimator::Config& estimator_config)
    : config_(config), estimator_(estimator_config) {
    if (!(config_.sample_frequency_hz > 0.0))
        throw std::invalid_argument("sample_frequency_hz must be positive");
    if (config_.moving_average_window == 0)
        throw std::invalid_argument("moving_average_window must be at least one");
    if (!(config_.feed_speed_min_mps > 0.0 &&
          config_.feed_speed_min_mps <= config_.feed_speed_max_mps))
        throw std::invalid_argument("invalid adaptive feed-speed bounds");
    if (!(config_.touchdown_threshold_n < config_.max_axial_force_n))
        throw std::invalid_argument("touchdown force must be below maximum force");
    if (!(config_.target_depth_m > 0.0 && config_.target_depth_m < config_.max_depth_m))
        throw std::invalid_argument("target depth must be positive and below max depth");

    touchdown_required_samples_ = static_cast<std::size_t>(
        std::ceil(config_.touchdown_confirmation_s * config_.sample_frequency_hz));
    if (touchdown_required_samples_ == 0) touchdown_required_samples_ = 1;

    hold_required_samples_ = static_cast<std::size_t>(
        std::ceil(config_.sensing_hold_duration_s * config_.sample_frequency_hz));
    reset();
}

void AdaptivePenetrationController::reset() {
    force_window_.clear();
    state_ = PenetrationState::Approach;
    touchdown_count_ = 0;
    z_ref_valid_ = false;
    z_ref_m_ = 0.0;
    trigger_valid_ = false;
    safety_trigger_ = false;
    trigger_depth_m_ = 0.0;
    terminal_depth_m_ = 0.0;
    terminal_depth_latched_ = false;
    peak_force_n_ = 0.0;
    hold_count_ = 0;
}

double AdaptivePenetrationController::movingAverage(double force_sample_n,
                                                    bool& window_ready)
{
    force_window_.push_back(force_sample_n);
    while (force_window_.size() > config_.moving_average_window)
        force_window_.pop_front();

    double sum_force_n = 0.0;
    for (double force_sample : force_window_) sum_force_n += force_sample;
    window_ready = (force_window_.size() == config_.moving_average_window);
    return sum_force_n / static_cast<double>(force_window_.size());
}

double AdaptivePenetrationController::adaptiveFeedSpeed(double smoothed_force_n) const {
    // Eq. (30): linear decrease from vp,max at Ftouch to vp,min at Fmax,
    // followed by saturation to [vp,min, vp,max].
    const double ratio = (smoothed_force_n - config_.touchdown_threshold_n) /
                         (config_.max_axial_force_n - config_.touchdown_threshold_n);
    const double unclamped_speed_mps = config_.feed_speed_max_mps -
        (config_.feed_speed_max_mps - config_.feed_speed_min_mps) * ratio;
    return std::max(config_.feed_speed_min_mps,
                    std::min(config_.feed_speed_max_mps, unclamped_speed_mps));
}

void AdaptivePenetrationController::latchTrigger(bool safety_trigger,
                                                 double depth_m)
{
    if (!trigger_valid_) {
        trigger_valid_ = true;
        safety_trigger_ = safety_trigger;
        trigger_depth_m_ = depth_m;
    } else if (safety_trigger) {
        // Safety has priority if it is triggered while normal stopping is in progress.
        safety_trigger_ = true;
    }
}

PenetrationOutput AdaptivePenetrationController::makeOutput(
    const AxialForceEstimate& estimate,
    double smoothed_force_n,
    bool filter_ready,
    double depth_m) const
{
    PenetrationOutput output;
    output.state = state_;
    output.joint3_force_n = estimate.joint3_force_n;
    output.joint4_force_n = estimate.joint4_force_n;
    output.fused_force_n = estimate.fused_force_n;
    output.smoothed_force_n = smoothed_force_n;
    output.force_filter_ready = filter_ready;
    output.force_within_calibration_range = estimate.within_calibration_range;
    output.touchdown_reference_valid = z_ref_valid_;
    output.z_ref_m = z_ref_m_;
    output.penetration_depth_m = depth_m;
    output.trigger_valid = trigger_valid_;
    output.safety_trigger = safety_trigger_;
    output.trigger_depth_m = trigger_depth_m_;
    output.terminal_depth_m = terminal_depth_m_;
    output.stopping_response_deviation_m =
        terminal_depth_latched_ ? terminal_depth_m_ - trigger_depth_m_ : 0.0;
    output.peak_estimated_force_n = peak_force_n_;
    output.touchdown_confirm_count = touchdown_count_;

    switch (state_) {
    case PenetrationState::Approach:
        output.action = MotionAction::FeedDown;
        output.feed_speed_mps = config_.approach_speed_mps;
        break;
    case PenetrationState::TouchdownCandidate:
        output.action = MotionAction::FeedDown;
        output.feed_speed_mps = config_.touchdown_candidate_speed_mps;
        break;
    case PenetrationState::AdaptivePenetration:
        output.action = MotionAction::FeedDown;
        output.feed_speed_mps = adaptiveFeedSpeed(smoothed_force_n);
        break;
    case PenetrationState::StoppingNormal:
    case PenetrationState::StoppingSafety:
        output.action = MotionAction::Stop;
        output.feed_speed_mps = 0.0;
        break;
    case PenetrationState::HoldForSensing:
        output.action = MotionAction::Hold;
        output.feed_speed_mps = 0.0;
        output.sensing_requested = (hold_count_ >= hold_required_samples_);
        break;
    case PenetrationState::RetractToReference:
        output.action = MotionAction::RetractToReference;
        output.feed_speed_mps = config_.retract_to_reference_speed_mps;
        break;
    case PenetrationState::RetractToSafeHeight:
        output.action = MotionAction::RetractToSafeHeight;
        output.feed_speed_mps = config_.retract_to_safe_height_speed_mps;
        break;
    case PenetrationState::Completed:
        output.action = MotionAction::None;
        output.feed_speed_mps = 0.0;
        break;
    }
    return output;
}

PenetrationOutput AdaptivePenetrationController::update(const PenetrationInput& input) {
    if (!std::isfinite(input.tcp_z_m))
        throw std::invalid_argument("TCP Z input must be finite");

    const AxialForceEstimate estimate = estimator_.estimate(
        input.joint3_torque_nm, input.joint4_torque_nm);
    bool filter_ready = false;
    const double smoothed_force_n = movingAverage(estimate.fused_force_n, filter_ready);
    peak_force_n_ = std::max(peak_force_n_, smoothed_force_n);

    double depth = 0.0;
    if (z_ref_valid_) depth = std::max(0.0, z_ref_m_ - input.tcp_z_m);

    switch (state_) {
    case PenetrationState::Approach:
        if (filter_ready && smoothed_force_n >= config_.touchdown_threshold_n) {
            // kc: first threshold crossing. Eq. (28) uses TCP Z at kc as Zref.
            z_ref_valid_ = true;
            z_ref_m_ = input.tcp_z_m;
            depth = 0.0;
            touchdown_count_ = 1;
            state_ = (touchdown_count_ >= touchdown_required_samples_)
                ? PenetrationState::AdaptivePenetration
                : PenetrationState::TouchdownCandidate;
        }
        break;

    case PenetrationState::TouchdownCandidate:
        if (!filter_ready || smoothed_force_n < config_.touchdown_threshold_n) {
            // Eq. (27) not sustained: return to approach and wait for a new kc.
            touchdown_count_ = 0;
            z_ref_valid_ = false;
            z_ref_m_ = 0.0;
            depth = 0.0;
            state_ = PenetrationState::Approach;
        } else {
            ++touchdown_count_;
            if (touchdown_count_ >= touchdown_required_samples_)
                state_ = PenetrationState::AdaptivePenetration;
        }
        break;

    case PenetrationState::AdaptivePenetration:
        // Eq. (31): safety has priority.
        if (smoothed_force_n >= config_.max_axial_force_n || depth >= config_.max_depth_m) {
            latchTrigger(true, depth);
            state_ = PenetrationState::StoppingSafety;
        } else if (depth >= config_.target_depth_m) {
            latchTrigger(false, depth);
            state_ = PenetrationState::StoppingNormal;
        }
        break;

    case PenetrationState::StoppingNormal:
        // The paper continues monitoring force/depth during stopping and switches
        // to Ssafe if a safety constraint becomes active.
        if (smoothed_force_n >= config_.max_axial_force_n || depth >= config_.max_depth_m) {
            latchTrigger(true, depth);
            state_ = PenetrationState::StoppingSafety;
        } else if (input.arm_is_stationary) {
            terminal_depth_m_ = depth;
            terminal_depth_latched_ = true;
            hold_count_ = 0;
            state_ = PenetrationState::HoldForSensing;
        }
        break;

    case PenetrationState::StoppingSafety:
        if (input.arm_is_stationary) {
            terminal_depth_m_ = depth;
            terminal_depth_latched_ = true;
            state_ = PenetrationState::RetractToSafeHeight;
        }
        break;

    case PenetrationState::HoldForSensing:
        if (hold_count_ < hold_required_samples_) ++hold_count_;
        if (hold_count_ >= hold_required_samples_ && input.sensing_complete)
            state_ = PenetrationState::RetractToReference;
        break;

    case PenetrationState::RetractToReference:
        if (input.at_penetration_reference) state_ = PenetrationState::RetractToSafeHeight;
        break;

    case PenetrationState::RetractToSafeHeight:
        if (input.at_safe_height) state_ = PenetrationState::Completed;
        break;

    case PenetrationState::Completed:
        break;
    }

    // Recompute depth after a new touchdown reference was latched.
    if (z_ref_valid_) depth = std::max(0.0, z_ref_m_ - input.tcp_z_m);
    else depth = 0.0;

    return makeOutput(estimate, smoothed_force_n, filter_ready, depth);
}

}  // namespace adaptive_penetration
