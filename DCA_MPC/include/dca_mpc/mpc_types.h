#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace dca_mpc {

constexpr double kPi = 3.14159265358979323846;

inline double wrapToPi(double angle)
{
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
}

struct Pose2D {
    double x;      // m
    double y;      // m
    double yaw;    // rad, counterclockwise positive

    Pose2D() : x(0.0), y(0.0), yaw(0.0) {}
    Pose2D(double x_in, double y_in, double yaw_in)
        : x(x_in), y(y_in), yaw(yaw_in) {}
};

struct ControlInput {
    double v;          // m/s
    double delta_f;    // rad, left steering positive
    double delta_r;    // rad, left steering positive

    ControlInput() : v(0.0), delta_f(0.0), delta_r(0.0) {}
};

struct StateErrorLocal {
    double longitudinal;  // m, along reference-path tangent
    double lateral;       // m, left of path positive
    double heading;       // rad

    StateErrorLocal() : longitudinal(0.0), lateral(0.0), heading(0.0) {}
};

struct ReferencePoint {
    double x;
    double y;
    double heading;      // rad
    double curvature;    // 1/m, signed
    double s;            // cumulative arc length, m

    ReferencePoint() : x(0.0), y(0.0), heading(0.0), curvature(0.0), s(0.0) {}
};

struct PathProjection {
    bool valid;
    std::size_t segment_index;   // projection lies on [i, i+1]
    double segment_ratio;        // 0..1
    double x;
    double y;
    double s;
    double distance;

    PathProjection()
        : valid(false), segment_index(0), segment_ratio(0.0),
          x(0.0), y(0.0), s(0.0),
          distance(std::numeric_limits<double>::infinity()) {}
};

enum class PathType {
    Straight,
    GentleCurve,
    SharpCurve
};

// Small fixed matrices keep the controller independent of external linear-algebra libraries.
// Row-major storage.
struct Matrix3x3 {
    std::array<double, 9> data{};

    double& operator()(std::size_t r, std::size_t c) { return data[r * 3 + c]; }
    double operator()(std::size_t r, std::size_t c) const { return data[r * 3 + c]; }

    static Matrix3x3 identity()
    {
        Matrix3x3 m;
        m(0,0) = 1.0;
        m(1,1) = 1.0;
        m(2,2) = 1.0;
        return m;
    }
};

struct LinearizedModel {
    Matrix3x3 A;
    Matrix3x3 B;
};

inline StateErrorLocal toLocalPathError(const Pose2D& actual,
                                        const Pose2D& reference)
{
    const double dx = actual.x - reference.x;
    const double dy = actual.y - reference.y;
    const double c = std::cos(reference.yaw);
    const double s = std::sin(reference.yaw);

    StateErrorLocal e;
    e.longitudinal =  c * dx + s * dy;
    e.lateral      = -s * dx + c * dy;
    e.heading      = wrapToPi(actual.yaw - reference.yaw);
    return e;
}

} // namespace dca_mpc
