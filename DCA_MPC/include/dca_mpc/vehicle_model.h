#pragma once

#include "dca_mpc/mpc_types.h"

namespace dca_mpc {

class VehicleModel {
public:
    struct Parameters {
        double wheelbase;   // L, m
        double sample_time; // Ts, s

        Parameters() : wheelbase(0.68), sample_time(0.10) {}
        Parameters(double L, double Ts) : wheelbase(L), sample_time(Ts) {}
    };

    VehicleModel();
    explicit VehicleModel(const Parameters& parameters);

    const Parameters& parameters() const { return params_; }

    // Eq. (1): continuous-time kinematic model.
    Pose2D derivative(const Pose2D& state, const ControlInput& input) const;

    // Eqs. (2)-(3): forward-Euler discretization.
    Pose2D step(const Pose2D& state, const ControlInput& input) const;

    // Eqs. (4)-(6): LTV model linearized about a reference state/control pair.
    LinearizedModel linearize(const Pose2D& reference_state,
                              const ControlInput& reference_input) const;

    // Paper steering constraint: delta_r = -delta_f.
    static ControlInput enforceOppositeSteering(double speed_mps,
                                                double front_steering_rad)
    {
        ControlInput input;
        input.v = speed_mps;
        input.delta_f = front_steering_rad;
        input.delta_r = -front_steering_rad;
        return input;
    }

private:
    Parameters params_;
};

} // namespace dca_mpc
