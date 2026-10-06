#include "dca_mpc/ltv_mpc.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct ScenarioResult {
    bool all_steps_solved = true;
    bool target_reached = false;
    int executed_steps = 0;
    double maximum_absolute_curvature = 0.0;
    double minimum_target_distance_m = 1e9;
    double maximum_absolute_steering_deg = 0.0;
    int maximum_qp_iterations = 0;
};

ScenarioResult runToTarget(
    const std::vector<dca_mpc::Pose2D>& reference_points,
    dca_mpc::Pose2D initial_robot_pose)
{
    using namespace dca_mpc;

    ReferencePath reference_path;
    reference_path.setPoints(reference_points);
    if (!reference_path.curvatureWithinLimit(0.40)) {
        throw std::runtime_error("generated path violates kappa_lim");
    }

    const Pose2D target_point{
        reference_points.back().x,
        reference_points.back().y,
        0.0};
    ControlInput previous_control;
    LtvMpcController controller;
    VehicleModel simulated_vehicle;

    ScenarioResult scenario_result;
    scenario_result.maximum_absolute_curvature =
        reference_path.maxAbsCurvature();

    Pose2D robot_pose = initial_robot_pose;
    for (int step_index = 0; step_index < 500; ++step_index) {
        const MpcStepResult result = controller.compute(
            robot_pose,
            previous_control,
            target_point,
            reference_path);

        scenario_result.executed_steps = step_index + 1;
        scenario_result.minimum_target_distance_m = std::min(
            scenario_result.minimum_target_distance_m,
            result.adaptive.target_distance);
        scenario_result.maximum_qp_iterations = std::max(
            scenario_result.maximum_qp_iterations,
            result.qp_iterations);
        scenario_result.maximum_absolute_steering_deg = std::max(
            scenario_result.maximum_absolute_steering_deg,
            std::fabs(result.command.delta_f) * 180.0 / kPi);

        if (!result.solved) {
            scenario_result.all_steps_solved = false;
            break;
        }
        if (result.adaptive.target_distance <= 0.05 && result.command.v <= 0.03) {
            scenario_result.target_reached = true;
            break;
        }

        robot_pose = simulated_vehicle.step(robot_pose, result.command);
        previous_control = result.command;
    }

    return scenario_result;
}

ScenarioResult runTrackingWindow(
    const std::vector<dca_mpc::Pose2D>& reference_points,
    dca_mpc::Pose2D initial_robot_pose,
    int window_steps)
{
    using namespace dca_mpc;

    ReferencePath reference_path;
    reference_path.setPoints(reference_points);
    if (!reference_path.curvatureWithinLimit(0.40)) {
        throw std::runtime_error("generated path violates kappa_lim");
    }

    const Pose2D target_point{
        reference_points.back().x,
        reference_points.back().y,
        0.0};
    ControlInput previous_control;
    previous_control.v = 0.30;
    LtvMpcController controller;
    VehicleModel simulated_vehicle;

    ScenarioResult scenario_result;
    scenario_result.maximum_absolute_curvature =
        reference_path.maxAbsCurvature();

    Pose2D robot_pose = initial_robot_pose;
    for (int step_index = 0; step_index < window_steps; ++step_index) {
        const MpcStepResult result = controller.compute(
            robot_pose,
            previous_control,
            target_point,
            reference_path);

        scenario_result.executed_steps = step_index + 1;
        scenario_result.minimum_target_distance_m = std::min(
            scenario_result.minimum_target_distance_m,
            result.adaptive.target_distance);
        scenario_result.maximum_qp_iterations = std::max(
            scenario_result.maximum_qp_iterations,
            result.qp_iterations);
        scenario_result.maximum_absolute_steering_deg = std::max(
            scenario_result.maximum_absolute_steering_deg,
            std::fabs(result.command.delta_f) * 180.0 / kPi);

        if (!result.solved) {
            scenario_result.all_steps_solved = false;
            break;
        }

        robot_pose = simulated_vehicle.step(robot_pose, result.command);
        previous_control = result.command;
    }

    return scenario_result;
}

std::vector<dca_mpc::Pose2D> makeStraightPath()
{
    std::vector<dca_mpc::Pose2D> reference_points;
    for (int point_index = 0; point_index <= 60; ++point_index) {
        reference_points.emplace_back(0.10 * point_index, 0.0, 0.0);
    }
    return reference_points;
}

std::vector<dca_mpc::Pose2D> makeGentleCurvePath()
{
    std::vector<dca_mpc::Pose2D> reference_points;
    for (int point_index = 0; point_index <= 80; ++point_index) {
        const double x_m = 0.075 * point_index;
        const double y_m = 0.20 * std::sin(0.70 * x_m);
        reference_points.emplace_back(x_m, y_m, 0.0);
    }
    return reference_points;
}

std::vector<dca_mpc::Pose2D> makeSharpArcPath(double curvature)
{
    using namespace dca_mpc;

    const double turn_sign = curvature >= 0.0 ? 1.0 : -1.0;
    const double absolute_curvature = std::fabs(curvature);
    const double radius_m = 1.0 / absolute_curvature;
    const double terminal_angle_rad = 1.20;
    const int point_count = 100;

    std::vector<Pose2D> reference_points;
    reference_points.reserve(static_cast<std::size_t>(point_count + 1));
    for (int point_index = 0; point_index <= point_count; ++point_index) {
        const double angle_rad = terminal_angle_rad
            * static_cast<double>(point_index) / point_count;
        reference_points.emplace_back(
            radius_m * std::sin(angle_rad),
            turn_sign * radius_m * (1.0 - std::cos(angle_rad)),
            0.0);
    }
    return reference_points;
}

void printResult(const std::string& scenario_name,
                 const ScenarioResult& result,
                 const char* mode)
{
    std::cout << scenario_name
              << ": mode=" << mode
              << ", solved_all=" << (result.all_steps_solved ? "yes" : "no")
              << ", reached=" << (result.target_reached ? "yes" : "n/a")
              << ", steps=" << result.executed_steps
              << ", max|kappa|=" << result.maximum_absolute_curvature
              << ", min_Dg=" << result.minimum_target_distance_m
              << ", max|delta_f|_deg=" << result.maximum_absolute_steering_deg
              << ", max_qp_iterations=" << result.maximum_qp_iterations
              << '\n';
}

}  // namespace

int main()
{
    using namespace dca_mpc;
    std::cout << std::fixed << std::setprecision(6);

    bool all_scenarios_passed = true;

    const ScenarioResult straight_result = runToTarget(
        makeStraightPath(),
        Pose2D{0.0, 0.10, 0.02});
    printResult("straight", straight_result, "tracking+stopping");
    all_scenarios_passed = all_scenarios_passed
        && straight_result.all_steps_solved
        && straight_result.target_reached;

    const ScenarioResult gentle_result = runToTarget(
        makeGentleCurvePath(),
        Pose2D{0.0, 0.12, 0.02});
    printResult("gentle", gentle_result, "tracking+stopping");
    all_scenarios_passed = all_scenarios_passed
        && gentle_result.all_steps_solved
        && gentle_result.target_reached;

    const ScenarioResult sharp_left_result = runTrackingWindow(
        makeSharpArcPath(+0.30),
        Pose2D{0.0, 0.0, 0.0},
        40);
    printResult("sharp_left", sharp_left_result, "tracking-window");
    all_scenarios_passed = all_scenarios_passed
        && sharp_left_result.all_steps_solved
        && sharp_left_result.maximum_absolute_curvature > 0.20
        && sharp_left_result.maximum_absolute_steering_deg <= 8.0 + 0.01;

    const ScenarioResult sharp_right_result = runTrackingWindow(
        makeSharpArcPath(-0.30),
        Pose2D{0.0, 0.0, 0.0},
        40);
    printResult("sharp_right", sharp_right_result, "tracking-window");
    all_scenarios_passed = all_scenarios_passed
        && sharp_right_result.all_steps_solved
        && sharp_right_result.maximum_absolute_curvature > 0.20
        && sharp_right_result.maximum_absolute_steering_deg <= 8.0 + 0.01;

    return all_scenarios_passed ? 0 : 2;
}
