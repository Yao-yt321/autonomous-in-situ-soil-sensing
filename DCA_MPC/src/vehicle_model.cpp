#include "dca_mpc/vehicle_model.h"

#include <cmath>
#include <stdexcept>

namespace dca_mpc {

VehicleModel::VehicleModel()
    : VehicleModel(Parameters())
{
}

VehicleModel::VehicleModel(const Parameters& parameters)
    : params_(parameters)
{
    if (params_.wheelbase <= 0.0) {
        throw std::invalid_argument("wheelbase must be positive");
    }
    if (params_.sample_time <= 0.0) {
        throw std::invalid_argument("sample_time must be positive");
    }
}

Pose2D VehicleModel::derivative(const Pose2D& state,
                                const ControlInput& input) const
{
    Pose2D dx;
    dx.x = input.v * std::cos(state.yaw);
    dx.y = input.v * std::sin(state.yaw);
    dx.yaw = input.v * (std::tan(input.delta_f) - std::tan(input.delta_r))
             / params_.wheelbase;
    return dx;
}

Pose2D VehicleModel::step(const Pose2D& state,
                          const ControlInput& input) const
{
    const Pose2D dx = derivative(state, input);

    Pose2D next;
    next.x = state.x + params_.sample_time * dx.x;
    next.y = state.y + params_.sample_time * dx.y;
    next.yaw = wrapToPi(state.yaw + params_.sample_time * dx.yaw);
    return next;
}

LinearizedModel VehicleModel::linearize(const Pose2D& reference_state,
                                        const ControlInput& reference_input) const
{
    const double phi = reference_state.yaw;
    const double v = reference_input.v;
    const double df = reference_input.delta_f;
    const double dr = reference_input.delta_r;
    const double L = params_.wheelbase;
    const double Ts = params_.sample_time;

    Matrix3x3 dfdx;
    dfdx(0,2) = -v * std::sin(phi);
    dfdx(1,2) =  v * std::cos(phi);

    Matrix3x3 dfdu;
    dfdu(0,0) = std::cos(phi);
    dfdu(1,0) = std::sin(phi);
    dfdu(2,0) = (std::tan(df) - std::tan(dr)) / L;

    const double cos_df = std::cos(df);
    const double cos_dr = std::cos(dr);
    dfdu(2,1) =  v / (L * cos_df * cos_df);
    dfdu(2,2) = -v / (L * cos_dr * cos_dr);

    LinearizedModel model;
    model.A = Matrix3x3::identity();
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
            model.A(r,c) += Ts * dfdx(r,c);
            model.B(r,c)  = Ts * dfdu(r,c);
        }
    }
    return model;
}

} // namespace dca_mpc
