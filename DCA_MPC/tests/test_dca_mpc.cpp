#include "dca_mpc/ltv_mpc.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

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

bool near(double a, double b, double tol = 1e-8)
{
    return std::fabs(a - b) <= tol;
}

} // namespace

int main()
{
    using namespace dca_mpc;

    // Scheduler boundary checks against Table 1, Table 2 and Eq. (14).
    DcaMpcScheduler scheduler;
    Pose2D robot{0.0, 0.0, 0.0};

    AdaptiveSettings far_straight = scheduler.evaluate(robot, Pose2D{2.0, 0.0, 0.0}, 0.0);
    check(far_straight.path_type == PathType::Straight, "straight classification");
    check(near(far_straight.lookahead_distance, 1.0), "Table 1 straight Ld=1.0");
    check(near(far_straight.reference_speed, 1.0), "Eq.14 far straight vref=1.0");
    check(near(far_straight.Q(0,0), 30.0) && near(far_straight.R(0,0), 1.0),
          "Table 2 straight weights");

    AdaptiveSettings far_gentle = scheduler.evaluate(robot, Pose2D{2.0, 0.0, 0.0}, 0.10);
    check(far_gentle.path_type == PathType::GentleCurve, "gentle classification");
    check(near(far_gentle.lookahead_distance, 0.8), "Table 1 gentle Ld=0.8");
    check(near(far_gentle.reference_speed, std::exp(-3.2 * 0.10), 1e-10),
          "Eq.14 curvature attenuation");

    AdaptiveSettings far_sharp = scheduler.evaluate(robot, Pose2D{2.0, 0.0, 0.0}, 0.30);
    check(far_sharp.path_type == PathType::SharpCurve, "sharp classification");
    check(near(far_sharp.lookahead_distance, 0.5), "Table 1 sharp Ld=0.5");

    AdaptiveSettings mid = scheduler.evaluate(robot, Pose2D{1.0, 0.0, 0.0}, 0.30);
    check(near(mid.lookahead_distance, 0.4), "Table 1 target-distance priority at 1.0 m");

    AdaptiveSettings final = scheduler.evaluate(robot, Pose2D{0.30, 0.0, 0.0}, 0.0);
    check(near(final.lookahead_distance, 0.3), "Table 1 final Ld=0.3");
    check(near(final.reference_speed, 0.30, 1e-10), "Eq.14 final approach speed");

    AdaptiveSettings stop = scheduler.evaluate(robot, Pose2D{0.20, 0.0, 0.0}, 0.0);
    check(stop.condition == MpcOperatingCondition::Stopping, "stopping weights at Dg<=0.2");
    check(near(stop.Q(0,0), 100.0) && near(stop.R(0,0), 5.0), "Table 2 stopping weights");

    AdaptiveSettings at_target = scheduler.evaluate(robot, Pose2D{0.0, 0.0, 0.0}, 0.0);
    check(near(at_target.reference_speed, 0.0), "Eq.14 zero speed at target");

    ControlInput uref = scheduler.referenceControl(far_gentle, 0.10);
    check(near(uref.delta_f + uref.delta_r, 0.0, 1e-12), "Eq.16 equal/opposite steering");
    check(near(uref.delta_f, std::atan(0.68 * 0.10 / 2.0), 1e-12), "Eq.16 reference steering value");

    // Kinematic Jacobian check using finite differences.
    VehicleModel model;
    Pose2D xr{1.0, 2.0, 0.4};
    ControlInput ur;
    ur.v = 0.7;
    ur.delta_f = 0.08;
    ur.delta_r = -0.08;
    const LinearizedModel lm = model.linearize(xr, ur);
    const double eps = 1e-6;
    for (int c = 0; c < 3; ++c) {
        Pose2D xp = xr, xm = xr;
        if (c == 0) { xp.x += eps; xm.x -= eps; }
        if (c == 1) { xp.y += eps; xm.y -= eps; }
        if (c == 2) { xp.yaw += eps; xm.yaw -= eps; }
        Pose2D fp = model.step(xp, ur);
        Pose2D fm = model.step(xm, ur);
        const double num[3] = {(fp.x-fm.x)/(2*eps), (fp.y-fm.y)/(2*eps),
                               wrapToPi(fp.yaw-fm.yaw)/(2*eps)};
        check(std::fabs(num[0]-lm.A(0,c)) < 2e-5 &&
              std::fabs(num[1]-lm.A(1,c)) < 2e-5 &&
              std::fabs(num[2]-lm.A(2,c)) < 2e-5,
              "Eq.5-6 state Jacobian column " + std::to_string(c));
    }

    // Input Jacobian B from Eq. (6), checked independently by finite differences.
    for (int c = 0; c < 3; ++c) {
        ControlInput up = ur, um = ur;
        if (c == 0) { up.v += eps; um.v -= eps; }
        if (c == 1) { up.delta_f += eps; um.delta_f -= eps; }
        if (c == 2) { up.delta_r += eps; um.delta_r -= eps; }
        const Pose2D fp = model.step(xr, up);
        const Pose2D fm = model.step(xr, um);
        const double num[3] = {(fp.x-fm.x)/(2*eps), (fp.y-fm.y)/(2*eps),
                               wrapToPi(fp.yaw-fm.yaw)/(2*eps)};
        check(std::fabs(num[0]-lm.B(0,c)) < 2e-5 &&
              std::fabs(num[1]-lm.B(1,c)) < 2e-5 &&
              std::fabs(num[2]-lm.B(2,c)) < 2e-5,
              "Eq.6 input Jacobian column " + std::to_string(c));
    }

    // One-step MPC regression and paper constraints.
    std::vector<Pose2D> reference_points = {
        {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.10, 0.0},
        {3.0, 0.30, 0.0}, {4.0, 0.60, 0.0}, {5.0, 1.00, 0.0},
        {6.0, 1.45, 0.0}
    };
    ReferencePath path;
    path.setPoints(reference_points);
    check(path.curvatureWithinLimit(0.40), "reference path within kappa_lim");

    LtvMpcController controller;
    ControlInput previous;
    previous.v = 0.65;
    const MpcStepResult result = controller.compute(Pose2D{3.10,0.34,0.22},
                                                    previous,
                                                    Pose2D{5.0,1.0,0.0},
                                                    path);
    check(result.solved, "default QP settings converge");
    check(!result.fallback_used, "no fallback on solved QP");
    check(result.command.v >= -1e-6 && result.command.v <= 1.0 + 1e-6,
          "absolute speed bound");
    check(std::fabs(result.command.delta_f) <= 8.0*kPi/180.0 + 1e-6 &&
          std::fabs(result.command.delta_r) <= 8.0*kPi/180.0 + 1e-6,
          "absolute steering bounds");
    check(std::fabs(result.command.delta_f + result.command.delta_r) <= 1e-10,
          "four-wheel steering equality");
    check(std::fabs(result.command.v - previous.v) <= 0.20 + 5e-4,
          "control-increment speed bound");
    check(std::fabs(result.command.delta_f - previous.delta_f) <= 15.0*kPi/180.0 + 5e-4,
          "Delta-delta_f bound");
    check(result.lookahead.s <= result.target_arc_length + 1e-12,
          "look-ahead reference does not advance beyond active target");

    // A target behind the current path projection must be rejected before optimization.
    bool target_behind_rejected = false;
    try {
        controller.compute(Pose2D{4.50, 0.80, 0.20}, previous,
                           Pose2D{2.0, 0.10, 0.0}, path);
    } catch (const std::invalid_argument&) {
        target_behind_rejected = true;
    }
    check(target_behind_rejected, "reject active target behind current path projection");

    // Reject a path that violates the manuscript's kappa_lim precondition.
    ReferencePath tight;
    tight.setPoints({Pose2D{0,0,0}, Pose2D{0.1,0,0}, Pose2D{0.1,0.1,0}});
    check(!tight.curvatureWithinLimit(0.40), "detect path above kappa_lim");
    bool rejected = false;
    try {
        controller.compute(Pose2D{0.0,0.0,0.0}, ControlInput(), Pose2D{0.1,0.1,0.0}, tight);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    check(rejected, "controller rejects path above kappa_lim");


    // Exact scheduler boundaries from the manuscript tables.
    AdaptiveSettings at_far = scheduler.evaluate(robot, Pose2D{1.20, 0.0, 0.0}, 0.0);
    check(near(at_far.lookahead_distance, 1.0), "Table 1 Dg=1.2 stays in path-type branch");
    check(near(at_far.reference_speed, 1.0), "Eq.14 Dg=1.2 continuity");

    AdaptiveSettings at_near = scheduler.evaluate(robot, Pose2D{0.40, 0.0, 0.0}, 0.0);
    check(near(at_near.lookahead_distance, 0.4), "Table 1 Dg=0.4 uses middle branch");
    check(near(at_near.reference_speed, 0.4), "Eq.14 Dg=0.4 continuity");

    AdaptiveSettings at_stop_boundary = scheduler.evaluate(robot, Pose2D{0.200000, 0.0, 0.0}, 0.0);
    AdaptiveSettings above_stop_boundary = scheduler.evaluate(robot, Pose2D{0.200001, 0.0, 0.0}, 0.0);
    check(at_stop_boundary.condition == MpcOperatingCondition::Stopping,
          "Table 2 Dg=0.2 activates stopping weights");
    check(above_stop_boundary.condition == MpcOperatingCondition::Straight,
          "Table 2 Dg>0.2 leaves stopping weights");

    AdaptiveSettings at_straight_boundary = scheduler.evaluate(robot, Pose2D{2.0,0.0,0.0}, 0.01);
    AdaptiveSettings above_straight_boundary = scheduler.evaluate(robot, Pose2D{2.0,0.0,0.0}, 0.010001);
    check(at_straight_boundary.path_type == PathType::Straight,
          "path-type boundary |kappa|=0.01 is straight");
    check(above_straight_boundary.path_type == PathType::GentleCurve,
          "path-type boundary |kappa|>0.01 is gentle");

    AdaptiveSettings at_gentle_boundary = scheduler.evaluate(robot, Pose2D{2.0,0.0,0.0}, 0.20);
    AdaptiveSettings above_gentle_boundary = scheduler.evaluate(robot, Pose2D{2.0,0.0,0.0}, 0.200001);
    check(at_gentle_boundary.path_type == PathType::GentleCurve,
          "path-type boundary |kappa|=0.2 is gentle");
    check(above_gentle_boundary.path_type == PathType::SharpCurve,
          "path-type boundary |kappa|>0.2 is sharp");

    // Eq. (14) monotonicity: larger curvature or smaller target distance must not increase v_ref.
    const double v_far_k0 = scheduler.evaluate(robot, Pose2D{2.0,0.0,0.0}, 0.0).reference_speed;
    const double v_far_k1 = scheduler.evaluate(robot, Pose2D{2.0,0.0,0.0}, 0.10).reference_speed;
    const double v_far_k2 = scheduler.evaluate(robot, Pose2D{2.0,0.0,0.0}, 0.30).reference_speed;
    check(v_far_k0 > v_far_k1 && v_far_k1 > v_far_k2,
          "Eq.14 reference speed decreases with |kappa|");

    const double v_d12 = scheduler.evaluate(robot, Pose2D{1.20,0.0,0.0}, 0.10).reference_speed;
    const double v_d08 = scheduler.evaluate(robot, Pose2D{0.80,0.0,0.0}, 0.10).reference_speed;
    const double v_d03 = scheduler.evaluate(robot, Pose2D{0.30,0.0,0.0}, 0.10).reference_speed;
    check(v_d12 > v_d08 && v_d08 > v_d03,
          "Eq.14 reference speed decreases toward target");

    // Signed-curvature symmetry for Eq. (16).
    const ControlInput uref_left = scheduler.referenceControl(far_gentle, +0.10);
    const ControlInput uref_right = scheduler.referenceControl(far_gentle, -0.10);
    check(near(uref_left.delta_f, -uref_right.delta_f, 1e-12) &&
          near(uref_left.delta_r, -uref_right.delta_r, 1e-12),
          "Eq.16 signed-curvature steering symmetry");

    // Local-path coordinate transformation used before Eq. (7).
    const StateErrorLocal e0 = toLocalPathError(Pose2D{1.0, 2.0, 0.20},
                                               Pose2D{1.0, 1.0, 0.0});
    check(near(e0.longitudinal, 0.0) && near(e0.lateral, 1.0) && near(e0.heading, 0.20),
          "local-path error for zero reference heading");
    const StateErrorLocal e90 = toLocalPathError(Pose2D{0.0, 1.0, kPi/2.0},
                                                Pose2D{0.0, 0.0, kPi/2.0});
    check(near(e90.longitudinal, 1.0, 1e-10) && near(e90.lateral, 0.0, 1e-10),
          "local-path error rotates with reference heading");

    // Curvature sign and tail behavior for Eq. (12).
    ReferencePath left_arc;
    left_arc.setPoints({Pose2D{0,0,0}, Pose2D{1,0,0}, Pose2D{1,1,0}, Pose2D{0,1,0}});
    check(left_arc.at(0).curvature > 0.0, "Eq.12 positive curvature for left turn");
    check(near(left_arc.at(left_arc.size()-1).curvature,
               left_arc.at(left_arc.size()-2).curvature, 1e-12),
          "Eq.12 tail reuses last valid three-point curvature");

    ReferencePath right_arc;
    right_arc.setPoints({Pose2D{0,0,0}, Pose2D{1,0,0}, Pose2D{1,-1,0}, Pose2D{0,-1,0}});
    check(right_arc.at(0).curvature < 0.0, "Eq.12 negative curvature for right turn");

    if (failures == 0) {
        std::cout << "All DCA-MPC tests passed.\n";
        return 0;
    }
    std::cerr << failures << " test(s) failed.\n";
    return 1;
}
