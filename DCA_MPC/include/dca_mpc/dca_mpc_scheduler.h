#pragma once

#include "dca_mpc/mpc_types.h"

namespace dca_mpc {

enum class MpcOperatingCondition {
    Straight,
    Curved,
    Stopping
};

struct AdaptiveSettings {
    double target_distance;          // D_g,k, m
    double current_curvature;        // kappa_k, 1/m (signed)
    PathType path_type;              // straight / gentle / sharp
    MpcOperatingCondition condition; // straight / curved / stopping for Q,R

    double lookahead_distance;       // L_d, m
    double reference_speed;          // v_ref,k, m/s
    double speed_upper_bound;        // v_max(kappa_k), m/s

    Matrix3x3 Q;
    Matrix3x3 R;
    Matrix3x3 S;

    bool stopping_weights_active;
    bool curvature_exceeds_limit;

    AdaptiveSettings()
        : target_distance(0.0), current_curvature(0.0),
          path_type(PathType::Straight),
          condition(MpcOperatingCondition::Straight),
          lookahead_distance(1.0), reference_speed(0.0),
          speed_upper_bound(0.0), stopping_weights_active(false),
          curvature_exceeds_limit(false) {}
};

class DcaMpcScheduler {
public:
    struct Parameters {
        // Vehicle / curvature parameters from the manuscript.
        double wheelbase;                  // L, m
        double alpha;                      // curvature attenuation coefficient, m
        double straight_threshold;         // 0.01 1/m
        double gentle_sharp_threshold;     // kappa_th = 0.2 1/m
        double curvature_limit;            // kappa_lim = 0.4 1/m

        // Target-distance thresholds.
        double far_distance;               // 1.2 m
        double near_distance;              // 0.4 m
        double stopping_weight_distance;   // 0.2 m

        // Table 1 look-ahead distances.
        double lookahead_straight;         // 1.0 m
        double lookahead_gentle;           // 0.8 m
        double lookahead_sharp;            // 0.5 m
        double lookahead_near;              // 0.4 m for 0.4 <= D_g < 1.2
        double lookahead_final;             // 0.3 m for D_g < 0.4

        // Eq. (14) speed schedule before curvature attenuation.
        double far_reference_speed;         // 1.0 m/s
        double near_reference_speed;        // 0.4 m/s at D_g = 0.4 m

        Parameters();
    };

    DcaMpcScheduler();
    explicit DcaMpcScheduler(const Parameters& parameters);

    const Parameters& parameters() const { return params_; }

    // Eq. (13): Euclidean distance from robot to current target detection point.
    double targetDistance(const Pose2D& robot_pose,
                          const Pose2D& target_point) const;

    // Implements the manuscript's dual-factor adaptation for L_d, v_ref,k, Q and R.
    AdaptiveSettings evaluate(const Pose2D& robot_pose,
                              const Pose2D& target_point,
                              double current_curvature) const;

    // Eq. (16): reference front/rear steering from curvature at the look-ahead point.
    ControlInput referenceControl(const AdaptiveSettings& settings,
                                  double lookahead_curvature) const;

private:
    static Matrix3x3 diagonal(double diagonal_0,
                              double diagonal_1,
                              double diagonal_2);
    PathType classify(double curvature) const;
    double curvatureAttenuation(double curvature) const;
    double lookaheadDistance(PathType path_type,
                             double target_distance) const;
    double referenceSpeed(double curvature, double target_distance) const;
    MpcOperatingCondition operatingCondition(double curvature,
                                             double target_distance) const;
    void assignWeights(MpcOperatingCondition condition,
                       Matrix3x3& Q,
                       Matrix3x3& R,
                       Matrix3x3& S) const;

    Parameters params_;
};

} // namespace dca_mpc
