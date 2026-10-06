#pragma once

#include <cstddef>
#include <string>

namespace adaptive_penetration {

enum class PenetrationState {
    Approach,
    TouchdownCandidate,
    AdaptivePenetration,
    StoppingNormal,
    StoppingSafety,
    HoldForSensing,
    RetractToReference,
    RetractToSafeHeight,
    Completed
};

enum class MotionAction {
    FeedDown,
    Stop,
    Hold,
    RetractToReference,
    RetractToSafeHeight,
    None
};

struct PenetrationInput {
    double joint3_torque_nm = 0.0;
    double joint4_torque_nm = 0.0;
    double tcp_z_m = 0.0;          // base-frame TCP Z, positive upward
    bool arm_is_stationary = false;
    bool sensing_complete = false;
    bool at_penetration_reference = false;
    bool at_safe_height = false;
};

struct PenetrationOutput {
    PenetrationState state = PenetrationState::Approach;
    MotionAction action = MotionAction::FeedDown;

    double joint3_force_n = 0.0;
    double joint4_force_n = 0.0;
    double fused_force_n = 0.0;
    double smoothed_force_n = 0.0;
    bool force_filter_ready = false;
    bool force_within_calibration_range = true;

    bool touchdown_reference_valid = false;
    double z_ref_m = 0.0;
    double penetration_depth_m = 0.0;
    double feed_speed_mps = 0.0;   // magnitude; direction is conveyed by action

    bool trigger_valid = false;
    bool safety_trigger = false;
    double trigger_depth_m = 0.0;
    double terminal_depth_m = 0.0;
    double stopping_response_deviation_m = 0.0;
    double peak_estimated_force_n = 0.0;

    bool sensing_requested = false;
    std::size_t touchdown_confirm_count = 0;
};

inline const char* toString(PenetrationState state)
{
    switch (state) {
    case PenetrationState::Approach: return "approach";
    case PenetrationState::TouchdownCandidate: return "touchdown_candidate";
    case PenetrationState::AdaptivePenetration: return "adaptive_penetration";
    case PenetrationState::StoppingNormal: return "stopping_normal";
    case PenetrationState::StoppingSafety: return "stopping_safety";
    case PenetrationState::HoldForSensing: return "hold_for_sensing";
    case PenetrationState::RetractToReference: return "retract_to_reference";
    case PenetrationState::RetractToSafeHeight: return "retract_to_safe_height";
    case PenetrationState::Completed: return "completed";
    }
    return "unknown";
}

inline const char* toString(MotionAction action)
{
    switch (action) {
    case MotionAction::FeedDown: return "feed_down";
    case MotionAction::Stop: return "stop";
    case MotionAction::Hold: return "hold";
    case MotionAction::RetractToReference: return "retract_to_reference";
    case MotionAction::RetractToSafeHeight: return "retract_to_safe_height";
    case MotionAction::None: return "none";
    }
    return "unknown";
}

}  // namespace adaptive_penetration
