#include "dca_mpc/reference_path.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace dca_mpc {

namespace {
constexpr double kEps = 1e-12;
}

double ReferencePath::euclidean(double x1, double y1, double x2, double y2)
{
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

void ReferencePath::setPoints(const std::vector<Pose2D>& input_points)
{
    if (input_points.size() < 2) {
        throw std::invalid_argument("ReferencePath requires at least two points");
    }

    points_.clear();
    points_.reserve(input_points.size());

    // Reject non-finite or duplicate consecutive points because Eq. (12) contains
    // segment lengths in the denominator.
    for (std::size_t i = 0; i < input_points.size(); ++i) {
        if (!std::isfinite(input_points[i].x) || !std::isfinite(input_points[i].y)) {
            throw std::invalid_argument("ReferencePath contains non-finite coordinates");
        }
        if (i > 0 && euclidean(input_points[i-1].x, input_points[i-1].y, input_points[i].x, input_points[i].y) <= 1e-9) {
            throw std::invalid_argument("ReferencePath contains duplicate consecutive points");
        }

        ReferencePoint p;
        p.x = input_points[i].x;
        p.y = input_points[i].y;
        points_.push_back(p);
    }

    // Eq. (11): heading from adjacent path points; last point inherits the previous heading.
    for (std::size_t i = 0; i + 1 < points_.size(); ++i) {
        const double dx = points_[i+1].x - points_[i].x;
        const double dy = points_[i+1].y - points_[i].y;
        points_[i].heading = std::atan2(dy, dx);
    }
    points_.back().heading = points_[points_.size() - 2].heading;

    // Cumulative arc length.
    points_[0].s = 0.0;
    for (std::size_t i = 1; i < points_.size(); ++i) {
        points_[i].s = points_[i-1].s +
                       euclidean(points_[i-1].x, points_[i-1].y,
                                 points_[i].x, points_[i].y);
    }

    // Eq. (12): forward three-point curvature.
    if (points_.size() == 2) {
        points_[0].curvature = 0.0;
        points_[1].curvature = 0.0;
        return;
    }

    for (std::size_t i = 0; i + 2 < points_.size(); ++i) {
        points_[i].curvature = signedCurvature(points_[i], points_[i+1], points_[i+2]);
    }

    // The manuscript specifies that the final three valid reference points are used
    // when i_k + 2 > n. Therefore the last valid three-point curvature is reused.
    const double tail_kappa = signedCurvature(points_[points_.size()-3],
                                              points_[points_.size()-2],
                                              points_[points_.size()-1]);
    points_[points_.size()-2].curvature = tail_kappa;
    points_[points_.size()-1].curvature = tail_kappa;
}

double ReferencePath::signedCurvature(const ReferencePoint& p0,
                                      const ReferencePoint& p1,
                                      const ReferencePoint& p2)
{
    const double d01 = euclidean(p0.x, p0.y, p1.x, p1.y);
    const double d12 = euclidean(p1.x, p1.y, p2.x, p2.y);
    const double d02 = euclidean(p0.x, p0.y, p2.x, p2.y);
    const double denom = d01 * d12 * d02;

    if (denom <= kEps) {
        return 0.0;
    }

    const double cross = (p1.x - p0.x) * (p2.y - p0.y)
                       - (p1.y - p0.y) * (p2.x - p0.x);
    return 2.0 * cross / denom;
}


double ReferencePath::maxAbsCurvature() const
{
    double max_kappa = 0.0;
    for (const ReferencePoint& p : points_) {
        max_kappa = std::max(max_kappa, std::fabs(p.curvature));
    }
    return max_kappa;
}

bool ReferencePath::curvatureWithinLimit(double curvature_limit,
                                         double tolerance) const
{
    if (curvature_limit < 0.0 || tolerance < 0.0) {
        throw std::invalid_argument("curvature limit and tolerance must be nonnegative");
    }
    return maxAbsCurvature() <= curvature_limit + tolerance;
}

PathProjection ReferencePath::project(double x, double y) const
{
    PathProjection best;
    if (points_.size() < 2) return best;

    for (std::size_t i = 0; i + 1 < points_.size(); ++i) {
        const double x0 = points_[i].x;
        const double y0 = points_[i].y;
        const double dx = points_[i+1].x - x0;
        const double dy = points_[i+1].y - y0;
        const double len2 = dx * dx + dy * dy;
        if (len2 <= kEps) continue;

        double t = ((x - x0) * dx + (y - y0) * dy) / len2;
        t = std::max(0.0, std::min(1.0, t));

        const double px = x0 + t * dx;
        const double py = y0 + t * dy;
        const double dist = euclidean(x, y, px, py);

        if (dist < best.distance) {
            best.valid = true;
            best.segment_index = i;
            best.segment_ratio = t;
            best.x = px;
            best.y = py;
            best.distance = dist;
            best.s = points_[i].s + t * (points_[i+1].s - points_[i].s);
        }
    }
    return best;
}

ReferencePoint ReferencePath::interpolateByArcLength(double s_query) const
{
    if (points_.empty()) {
        throw std::runtime_error("ReferencePath is empty");
    }
    if (points_.size() == 1 || s_query <= 0.0) return points_.front();
    if (s_query >= points_.back().s) return points_.back();

    auto it = std::upper_bound(points_.begin(), points_.end(), s_query,
        [](double s, const ReferencePoint& p) { return s < p.s; });

    const std::size_t i1 = static_cast<std::size_t>(it - points_.begin());
    const std::size_t i0 = i1 - 1;
    const ReferencePoint& a = points_[i0];
    const ReferencePoint& b = points_[i1];

    const double ds = b.s - a.s;
    const double t = (ds <= kEps) ? 0.0 : (s_query - a.s) / ds;

    ReferencePoint out;
    out.s = s_query;
    out.x = a.x + t * (b.x - a.x);
    out.y = a.y + t * (b.y - a.y);

    const double d_heading = wrapToPi(b.heading - a.heading);
    out.heading = wrapToPi(a.heading + t * d_heading);
    out.curvature = a.curvature + t * (b.curvature - a.curvature);
    return out;
}

std::size_t ReferencePath::matchedIndex(const PathProjection& p) const
{
    if (!p.valid || points_.empty()) {
        throw std::invalid_argument("invalid path projection");
    }
    if (p.segment_index + 1 >= points_.size()) {
        return points_.size() - 1;
    }
    return (p.segment_ratio < 0.5) ? p.segment_index : p.segment_index + 1;
}

PathType ReferencePath::classifyCurvature(double curvature,
                                          double straight_threshold,
                                          double gentle_sharp_threshold)
{
    const double a = std::fabs(curvature);
    if (a <= straight_threshold) return PathType::Straight;
    if (a <= gentle_sharp_threshold) return PathType::GentleCurve;
    return PathType::SharpCurve;
}

} // namespace dca_mpc
