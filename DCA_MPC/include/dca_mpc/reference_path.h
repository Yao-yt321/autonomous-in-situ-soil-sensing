#pragma once

#include "dca_mpc/mpc_types.h"

#include <vector>

namespace dca_mpc {

class ReferencePath {
public:
    ReferencePath() = default;

    // Build the reference path from planar points (meters).
    // Headings follow Eq. (11); curvature follows Eq. (12).
    void setPoints(const std::vector<Pose2D>& points_xy_only);

    bool empty() const { return points_.empty(); }
    std::size_t size() const { return points_.size(); }
    double length() const { return points_.empty() ? 0.0 : points_.back().s; }

    // Maximum absolute discrete curvature computed using Eq. (12).
    double maxAbsCurvature() const;

    // The manuscript requires path segments above kappa_lim to be smoothed or
    // constrained before tracking. This helper validates that precondition.
    bool curvatureWithinLimit(double curvature_limit = 0.40,
                              double tolerance = 1e-9) const;

    const std::vector<ReferencePoint>& points() const { return points_; }
    const ReferencePoint& at(std::size_t i) const { return points_.at(i); }

    // Nearest projection on the polyline. This implements the paper's
    // "current path projection point" used for cumulative-arc-length lookup.
    PathProjection project(double x, double y) const;

    // Linear interpolation by cumulative arc length.
    ReferencePoint interpolateByArcLength(double s_query) const;

    // Current matched reference-point index used for kappa_k.
    // The nearest endpoint of the projection segment is selected.
    std::size_t matchedIndex(const PathProjection& projection) const;

    // Paper path classes used by the DCA scheduler.
    static PathType classifyCurvature(double curvature,
                                      double straight_threshold = 0.01,
                                      double gentle_sharp_threshold = 0.20);

private:
    static double euclidean(double x1, double y1, double x2, double y2);
    static double signedCurvature(const ReferencePoint& p0,
                                  const ReferencePoint& p1,
                                  const ReferencePoint& p2);

    std::vector<ReferencePoint> points_;
};

} // namespace dca_mpc
