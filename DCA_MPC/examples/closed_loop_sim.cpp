#include "dca_mpc/ltv_mpc.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

int main(int argc, char** argv)
{
    using namespace dca_mpc;

    const char* output_path = (argc > 1) ? argv[1] : "closed_loop_trace.csv";

    // Smooth reference path with varying but admissible curvature. This is a
    // numerical integration check, not a reproduction of a field trial.
    std::vector<Pose2D> reference_points;
    for (int point_index = 0; point_index <= 80; ++point_index) {
        const double x_m = 0.075 * point_index;
        const double y_m = 0.20 * std::sin(0.70 * x_m);
        reference_points.emplace_back(x_m, y_m, 0.0);
    }

    ReferencePath reference_path;
    reference_path.setPoints(reference_points);
    if (!reference_path.curvatureWithinLimit(0.40)) {
        std::cerr << "example path violates kappa_lim\n";
        return 3;
    }

    Pose2D robot_pose{0.0, 0.12, 0.02};
    const Pose2D target_point{
        reference_points.back().x,
        reference_points.back().y,
        0.0};
    ControlInput previous_control;

    LtvMpcController controller;
    VehicleModel simulated_vehicle;

    std::ofstream trace_file(output_path);
    if (!trace_file) {
        std::cerr << "cannot open output CSV\n";
        return 4;
    }
    trace_file
        << "step,time_s,x_m,y_m,yaw_rad,Dg_m,kappa_1pm,Ld_m,vref_mps,"
           "v_cmd_mps,delta_f_rad,delta_r_rad,qp_solved,qp_iterations,"
           "primal_residual,dual_residual\n";

    const int maximum_steps = 300;
    bool target_reached = false;
    double minimum_target_distance_m = 1e9;

    for (int step_index = 0; step_index < maximum_steps; ++step_index) {
        MpcStepResult result;
        try {
            result = controller.compute(
                robot_pose,
                previous_control,
                target_point,
                reference_path);
        } catch (const std::exception& error) {
            std::cerr << "controller error at step " << step_index
                      << ": " << error.what() << '\n';
            return 5;
        }

        if (!result.solved) {
            std::cerr << "QP did not converge at step " << step_index << '\n';
            return 6;
        }

        minimum_target_distance_m = std::min(
            minimum_target_distance_m,
            result.adaptive.target_distance);

        trace_file << step_index << ',' << step_index * 0.10 << ','
                   << robot_pose.x << ',' << robot_pose.y << ',' << robot_pose.yaw << ','
                   << result.adaptive.target_distance << ','
                   << result.adaptive.current_curvature << ','
                   << result.adaptive.lookahead_distance << ','
                   << result.adaptive.reference_speed << ','
                   << result.command.v << ',' << result.command.delta_f << ','
                   << result.command.delta_r << ',' << (result.solved ? 1 : 0) << ','
                   << result.qp_iterations << ',' << result.primal_residual << ','
                   << result.dual_residual << '\n';

        // Simulation-only loop termination. It is not part of the DCA-MPC law.
        if (result.adaptive.target_distance <= 0.05 && result.command.v <= 0.03) {
            target_reached = true;
            break;
        }

        robot_pose = simulated_vehicle.step(robot_pose, result.command);
        previous_control = result.command;
    }

    std::cout << std::fixed << std::setprecision(6)
              << "max |kappa|: " << reference_path.maxAbsCurvature() << " 1/m\n"
              << "minimum target distance: " << minimum_target_distance_m << " m\n"
              << "target reached: " << (target_reached ? "yes" : "no") << '\n'
              << "trace: " << output_path << '\n';

    return target_reached ? 0 : 7;
}
