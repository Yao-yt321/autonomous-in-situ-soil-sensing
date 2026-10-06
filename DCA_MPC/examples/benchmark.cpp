#include "dca_mpc/ltv_mpc.h"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <vector>

int main()
{
    using namespace dca_mpc;
    using Clock = std::chrono::steady_clock;

    const std::vector<Pose2D> reference_points = {
        {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.10, 0.0},
        {3.0, 0.30, 0.0}, {4.0, 0.60, 0.0}, {5.0, 1.00, 0.0},
        {6.0, 1.45, 0.0}
    };

    ReferencePath reference_path;
    reference_path.setPoints(reference_points);

    const Pose2D robot_pose{3.10, 0.34, 0.22};
    const Pose2D target_point{5.0, 1.0, 0.0};
    ControlInput previous_control;
    previous_control.v = 0.65;

    LtvMpcController controller;
    const int repeat_count = 200;
    double total_time_ms = 0.0;
    double maximum_time_ms = 0.0;
    int maximum_qp_iterations = 0;

    for (int run_index = 0; run_index < repeat_count; ++run_index) {
        const auto start_time = Clock::now();
        const MpcStepResult result = controller.compute(
            robot_pose,
            previous_control,
            target_point,
            reference_path);
        const auto end_time = Clock::now();
        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(end_time - start_time).count();

        if (!result.solved) {
            std::cerr << "QP failed during benchmark at iteration "
                      << run_index << '\n';
            return 2;
        }

        total_time_ms += elapsed_ms;
        maximum_time_ms = std::max(maximum_time_ms, elapsed_ms);
        maximum_qp_iterations = std::max(
            maximum_qp_iterations,
            result.qp_iterations);
        previous_control = result.command;
    }

    std::cout << std::fixed << std::setprecision(3)
              << "calls: " << repeat_count << '\n'
              << "average_ms: " << total_time_ms / repeat_count << '\n'
              << "max_ms: " << maximum_time_ms << '\n'
              << "max_qp_iterations: " << maximum_qp_iterations << '\n';
    return 0;
}
