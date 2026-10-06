#include "dca_mpc/ltv_mpc.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

int main()
{
    using namespace dca_mpc;

    const std::vector<Pose2D> reference_points = {
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {2.0, 0.10, 0.0},
        {3.0, 0.30, 0.0},
        {4.0, 0.60, 0.0},
        {5.0, 1.00, 0.0},
        {6.0, 1.45, 0.0}
    };

    ReferencePath reference_path;
    reference_path.setPoints(reference_points);

    const Pose2D robot_pose{3.10, 0.34, 0.22};
    const Pose2D target_point{5.00, 1.00, 0.0};

    ControlInput previous_control;
    previous_control.v = 0.65;

    // Uses the default DCA-MPC settings, including Np = 6 and Nc = 2.
    LtvMpcController controller;
    const MpcStepResult result = controller.compute(
        robot_pose,
        previous_control,
        target_point,
        reference_path);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "QP solved: " << (result.solved ? "yes" : "no") << '\n';
    std::cout << "iterations: " << result.qp_iterations << '\n';
    std::cout << "D_g,k: " << result.adaptive.target_distance << " m\n";
    std::cout << "kappa_k: " << result.adaptive.current_curvature << " 1/m\n";
    std::cout << "L_d: " << result.adaptive.lookahead_distance << " m\n";
    std::cout << "v_ref,k: " << result.adaptive.reference_speed << " m/s\n";
    std::cout << "command v: " << result.command.v << " m/s\n";
    std::cout << "command delta_f: "
              << result.command.delta_f * 180.0 / kPi << " deg\n";
    std::cout << "command delta_r: "
              << result.command.delta_r * 180.0 / kPi << " deg\n";
    std::cout << "primal residual: " << result.primal_residual << '\n';
    std::cout << "dual residual: " << result.dual_residual << '\n';

    const bool constraints_ok =
        result.solved &&
        result.command.v >= -1e-4 &&
        result.command.v <= 1.0 + 1e-4 &&
        std::fabs(result.command.delta_f) <= 8.0 * kPi / 180.0 + 1e-4 &&
        std::fabs(result.command.delta_r) <= 8.0 * kPi / 180.0 + 1e-4 &&
        std::fabs(result.command.delta_f + result.command.delta_r) <= 1e-8;

    std::cout << "constraint check: " << (constraints_ok ? "pass" : "fail") << '\n';
    return constraints_ok ? 0 : 2;
}
