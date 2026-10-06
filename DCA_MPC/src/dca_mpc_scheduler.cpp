#include "dca_mpc/dca_mpc_scheduler.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dca_mpc {

DcaMpcScheduler::Parameters::Parameters()
    : wheelbase(0.68),
      alpha(3.2),
      straight_threshold(0.01),
      gentle_sharp_threshold(0.20),
      curvature_limit(0.40),
      far_distance(1.20),
      near_distance(0.40),
      stopping_weight_distance(0.20),
      lookahead_straight(1.00),
      lookahead_gentle(0.80),
      lookahead_sharp(0.50),
      lookahead_near(0.40),
      lookahead_final(0.30),
      far_reference_speed(1.00),
      near_reference_speed(0.40)
{
}

DcaMpcScheduler::DcaMpcScheduler()
    : DcaMpcScheduler(Parameters())
{
}

DcaMpcScheduler::DcaMpcScheduler(const Parameters& parameters)
    : params_(parameters)
{
    if (params_.wheelbase <= 0.0 || params_.alpha < 0.0) {
        throw std::invalid_argument("invalid vehicle/curvature parameters");
    }
    if (!(0.0 < params_.straight_threshold &&
          params_.straight_threshold < params_.gentle_sharp_threshold &&
          params_.gentle_sharp_threshold <= params_.curvature_limit)) {
        throw std::invalid_argument("invalid curvature thresholds");
    }
    if (!(0.0 < params_.stopping_weight_distance &&
          params_.stopping_weight_distance < params_.near_distance &&
          params_.near_distance < params_.far_distance)) {
        throw std::invalid_argument("invalid target-distance thresholds");
    }
    if (params_.far_reference_speed <= 0.0 ||
        params_.near_reference_speed <= 0.0 ||
        params_.near_reference_speed > params_.far_reference_speed) {
        throw std::invalid_argument("invalid reference-speed parameters");
    }
}

Matrix3x3 DcaMpcScheduler::diagonal(double diagonal_0,
                                     double diagonal_1,
                                     double diagonal_2)
{
    Matrix3x3 matrix;
    matrix(0,0) = diagonal_0;
    matrix(1,1) = diagonal_1;
    matrix(2,2) = diagonal_2;
    return matrix;
}

double DcaMpcScheduler::targetDistance(const Pose2D& robot_pose,
                                       const Pose2D& target_point) const
{
    const double dx = robot_pose.x - target_point.x;
    const double dy = robot_pose.y - target_point.y;
    return std::sqrt(dx * dx + dy * dy);
}

PathType DcaMpcScheduler::classify(double curvature) const
{
    const double absolute_curvature = std::fabs(curvature);
    if (absolute_curvature <= params_.straight_threshold) return PathType::Straight;
    if (absolute_curvature <= params_.gentle_sharp_threshold) return PathType::GentleCurve;
    return PathType::SharpCurve;
}

double DcaMpcScheduler::curvatureAttenuation(double curvature) const
{
    return std::exp(-params_.alpha * std::fabs(curvature));
}

double DcaMpcScheduler::lookaheadDistance(PathType path_type,
                                              double target_distance) const
{
    // Table 1: target distance takes priority once D_g,k < 1.2 m.
    if (target_distance < params_.near_distance) {
        return params_.lookahead_final;
    }
    if (target_distance < params_.far_distance) {
        return params_.lookahead_near;
    }

    switch (path_type) {
    case PathType::Straight:
        return params_.lookahead_straight;
    case PathType::GentleCurve:
        return params_.lookahead_gentle;
    case PathType::SharpCurve:
        return params_.lookahead_sharp;
    }
    return params_.lookahead_straight;
}

double DcaMpcScheduler::referenceSpeed(double curvature,
                                    double target_distance) const
{
    // Eq. (14). D_g,k is a distance and is therefore nonnegative.
    const double Dg = std::max(0.0, target_distance);
    const double attenuation = curvatureAttenuation(curvature);

    double base_speed = 0.0;
    if (Dg >= params_.far_distance) {
        base_speed = params_.far_reference_speed;
    } else if (Dg >= params_.near_distance) {
        const double ratio = (Dg - params_.near_distance) /
                             (params_.far_distance - params_.near_distance);
        base_speed = params_.near_reference_speed +
                     (params_.far_reference_speed - params_.near_reference_speed) * ratio;
    } else {
        base_speed = params_.near_reference_speed *
                     (Dg / params_.near_distance);
    }

    return base_speed * attenuation;
}

MpcOperatingCondition DcaMpcScheduler::operatingCondition(double curvature,
                                                        double target_distance) const
{
    // Table 2: stopping condition has priority at D_g,k <= 0.2 m.
    if (target_distance <= params_.stopping_weight_distance) {
        return MpcOperatingCondition::Stopping;
    }
    if (std::fabs(curvature) <= params_.straight_threshold) {
        return MpcOperatingCondition::Straight;
    }
    return MpcOperatingCondition::Curved;
}

void DcaMpcScheduler::assignWeights(MpcOperatingCondition condition,
                                 Matrix3x3& Q,
                                 Matrix3x3& R,
                                 Matrix3x3& S) const
{
    // Table 2 and the fixed smoothing matrix stated below the table.
    switch (condition) {
    case MpcOperatingCondition::Straight:
        Q = diagonal(30.0, 50.0, 30.0);
        R = diagonal(1.0, 1.0, 1.0);
        break;
    case MpcOperatingCondition::Curved:
        Q = diagonal(50.0, 80.0, 100.0);
        R = diagonal(2.0, 2.0, 1.0);
        break;
    case MpcOperatingCondition::Stopping:
        Q = diagonal(100.0, 100.0, 80.0);
        R = diagonal(5.0, 5.0, 3.0);
        break;
    }
    S = diagonal(1.0, 30.0, 30.0);
}

AdaptiveSettings DcaMpcScheduler::evaluate(const Pose2D& robot_pose,
                                            const Pose2D& target_point,
                                            double current_curvature) const
{
    AdaptiveSettings settings;
    settings.target_distance = targetDistance(robot_pose, target_point);
    settings.current_curvature = current_curvature;
    settings.path_type = classify(current_curvature);
    settings.condition = operatingCondition(
        current_curvature,
        settings.target_distance);
    settings.lookahead_distance = lookaheadDistance(
        settings.path_type,
        settings.target_distance);
    settings.reference_speed = referenceSpeed(
        current_curvature,
        settings.target_distance);
    settings.speed_upper_bound = params_.far_reference_speed
        * curvatureAttenuation(current_curvature);
    settings.stopping_weights_active =
        (settings.condition == MpcOperatingCondition::Stopping);
    settings.curvature_exceeds_limit =
        (std::fabs(current_curvature) > params_.curvature_limit);

    assignWeights(settings.condition, settings.Q, settings.R, settings.S);
    return settings;
}

ControlInput DcaMpcScheduler::referenceControl(const AdaptiveSettings& settings,
                                            double lookahead_curvature) const
{
    // Eq. (16): equal and opposite front/rear reference steering.
    const double delta_f = std::atan(params_.wheelbase * lookahead_curvature / 2.0);

    ControlInput reference_control;
    reference_control.v = std::min(
        settings.reference_speed,
        settings.speed_upper_bound);
    reference_control.delta_f = delta_f;
    reference_control.delta_r = -delta_f;
    return reference_control;
}

} // namespace dca_mpc
