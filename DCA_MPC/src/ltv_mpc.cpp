#include "dca_mpc/ltv_mpc.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace dca_mpc {

namespace {

constexpr std::size_t kNx = 3;
constexpr std::size_t kNu = 3;

double deg2rad(double deg) { return deg * kPi / 180.0; }

std::vector<double> mat3Vec(const Matrix3x3& A, const std::vector<double>& x)
{
    if (x.size() != 3) throw std::invalid_argument("mat3Vec size mismatch");
    std::vector<double> y(3, 0.0);
    for (std::size_t r = 0; r < 3; ++r)
        for (std::size_t c = 0; c < 3; ++c)
            y[r] += A(r,c) * x[c];
    return y;
}

DenseMatrix mat3TimesDense(const Matrix3x3& A, const DenseMatrix& M)
{
    if (M.rows() != 3) throw std::invalid_argument("mat3TimesDense size mismatch");
    DenseMatrix out(3, M.cols(), 0.0);
    for (std::size_t r = 0; r < 3; ++r)
        for (std::size_t c = 0; c < M.cols(); ++c)
            for (std::size_t k = 0; k < 3; ++k)
                out(r,c) += A(r,k) * M(k,c);
    return out;
}

void addBToControlBlock(DenseMatrix& M, const Matrix3x3& B, std::size_t block)
{
    const std::size_t col0 = block * kNu;
    for (std::size_t r = 0; r < 3; ++r)
        for (std::size_t c = 0; c < 3; ++c)
            M(r, col0 + c) += B(r,c);
}

std::vector<double> subtractBRef(const std::vector<double>& c,
                                 const Matrix3x3& B,
                                 const ControlInput& uref)
{
    std::vector<double> u = {uref.v, uref.delta_f, uref.delta_r};
    std::vector<double> Bu = mat3Vec(B, u);
    std::vector<double> out = c;
    for (std::size_t i = 0; i < 3; ++i) out[i] -= Bu[i];
    return out;
}

// T maps global [dx,dy,dphi] into the local path coordinates used by Q.
Matrix3x3 localTransform(double heading)
{
    Matrix3x3 T;
    const double c = std::cos(heading);
    const double s = std::sin(heading);
    T(0,0) = c;  T(0,1) = s;
    T(1,0) = -s; T(1,1) = c;
    T(2,2) = 1.0;
    return T;
}

Matrix3x3 transpose3(const Matrix3x3& A)
{
    Matrix3x3 T;
    for (std::size_t r = 0; r < 3; ++r)
        for (std::size_t c = 0; c < 3; ++c)
            T(r,c) = A(c,r);
    return T;
}

Matrix3x3 multiply3(const Matrix3x3& A, const Matrix3x3& B)
{
    Matrix3x3 C;
    for (std::size_t r = 0; r < 3; ++r)
        for (std::size_t c = 0; c < 3; ++c)
            for (std::size_t k = 0; k < 3; ++k)
                C(r,c) += A(r,k) * B(k,c);
    return C;
}

Matrix3x3 localWeight(const Matrix3x3& Q, double heading)
{
    const Matrix3x3 T = localTransform(heading);
    return multiply3(transpose3(T), multiply3(Q, T));
}

void addStateCost(DenseMatrix& P,
                  std::vector<double>& q,
                  const DenseMatrix& M,
                  const std::vector<double>& c,
                  const Matrix3x3& W)
{
    const std::size_t n = M.cols();
    // P += 2 M' W M
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            double v = 0.0;
            for (std::size_t r = 0; r < 3; ++r)
                for (std::size_t s = 0; s < 3; ++s)
                    v += M(r,i) * W(r,s) * M(s,j);
            P(i,j) += 2.0 * v;
        }
    }
    // q += 2 M' W c
    for (std::size_t i = 0; i < n; ++i) {
        double v = 0.0;
        for (std::size_t r = 0; r < 3; ++r)
            for (std::size_t s = 0; s < 3; ++s)
                v += M(r,i) * W(r,s) * c[s];
        q[i] += 2.0 * v;
    }
}

void addQuadraticBlock(DenseMatrix& P, std::size_t block, const Matrix3x3& W, double scale)
{
    const std::size_t c0 = block * 3;
    for (std::size_t r = 0; r < 3; ++r)
        for (std::size_t c = 0; c < 3; ++c)
            P(c0 + r, c0 + c) += scale * W(r,c);
}

void addLinearBlock(std::vector<double>& q,
                    std::size_t block,
                    const Matrix3x3& W,
                    const std::vector<double>& a,
                    double scale)
{
    const std::size_t c0 = block * 3;
    for (std::size_t r = 0; r < 3; ++r) {
        double v = 0.0;
        for (std::size_t c = 0; c < 3; ++c) v += W(r,c) * a[c];
        q[c0 + r] += scale * v;
    }
}

} // namespace

MpcParameters::MpcParameters()
    : prediction_horizon(6), control_horizon(2),
      min_speed(0.0), max_speed(1.0), max_steering(deg2rad(8.0)),
      max_delta_speed(0.20), max_delta_steering(deg2rad(15.0)) {}

MpcStepResult::MpcStepResult()
    : solved(false), qp_iterations(0), primal_residual(0.0), dual_residual(0.0),
      projection_arc_length(0.0), target_arc_length(0.0), fallback_used(false) {}

LtvMpcController::LtvMpcController()
    : LtvMpcController(MpcParameters(), DenseQpSettings()) {}

LtvMpcController::LtvMpcController(const MpcParameters& params,
                                   const DenseQpSettings& qp_settings)
    : params_(params), model_(), scheduler_(), solver_(qp_settings)
{
    if (params_.prediction_horizon <= 0 || params_.control_horizon <= 0 ||
        params_.control_horizon > params_.prediction_horizon) {
        throw std::invalid_argument("invalid MPC horizons");
    }
    if (params_.control_horizon != 2) {
        throw std::invalid_argument("this implementation requires N_c = 2");
    }
}

void LtvMpcController::reset()
{
    warm_start_.clear();
}

std::vector<double> LtvMpcController::controlToVector(const ControlInput& u)
{
    return {u.v, u.delta_f, u.delta_r};
}

ControlInput LtvMpcController::vectorToControl(const std::vector<double>& x, std::size_t offset)
{
    if (offset + 2 >= x.size()) throw std::invalid_argument("vectorToControl offset out of range");
    ControlInput u;
    u.v = x[offset];
    u.delta_f = x[offset + 1];
    u.delta_r = x[offset + 2];
    return u;
}

LtvMpcController::HorizonData LtvMpcController::buildHorizon(
    const ReferencePath& path,
    const PathProjection& projection,
    const AdaptiveSettings& adaptive,
    double target_s) const
{
    HorizonData h;
    const int Np = params_.prediction_horizon;
    h.ref_states.reserve(static_cast<std::size_t>(Np + 1));
    h.ref_inputs.reserve(static_cast<std::size_t>(Np));
    h.models.reserve(static_cast<std::size_t>(Np));

    // The current target detection point is a stopping boundary for the active
    // path segment. The look-ahead/reference horizon must therefore not advance
    // beyond that target before the stop is completed.
    const double s0 = std::min(projection.s + adaptive.lookahead_distance, target_s);
    const double ds = adaptive.reference_speed * model_.parameters().sample_time;

    for (int j = 0; j <= Np; ++j) {
        const double sj = std::min(s0 + static_cast<double>(j) * ds, target_s);
        h.ref_states.push_back(path.interpolateByArcLength(sj));
    }

    for (int j = 0; j < Np; ++j) {
        ControlInput uref = scheduler_.referenceControl(adaptive, h.ref_states[static_cast<std::size_t>(j)].curvature);
        h.ref_inputs.push_back(uref);
        const ReferencePoint& rp = h.ref_states[static_cast<std::size_t>(j)];
        const Pose2D pose_ref{rp.x, rp.y, rp.heading};
        h.models.push_back(model_.linearize(pose_ref, uref));
    }
    return h;
}

DenseQpProblem LtvMpcController::buildQp(const Pose2D& robot,
                                         const ControlInput& previous_input,
                                         const HorizonData& horizon,
                                         const AdaptiveSettings& adaptive) const
{
    const int Np = params_.prediction_horizon;
    const int Nc = params_.control_horizon;
    const std::size_t nvar = static_cast<std::size_t>(Nc) * kNu; // 6 for N_c=2

    DenseQpProblem qp;
    qp.P = DenseMatrix(nvar, nvar, 0.0);
    qp.q.assign(nvar, 0.0);

    // Initial global state deviation relative to the current look-ahead reference.
    const ReferencePoint& r0 = horizon.ref_states.front();
    std::vector<double> c = {
        robot.x - r0.x,
        robot.y - r0.y,
        wrapToPi(robot.yaw - r0.heading)
    };
    DenseMatrix M(kNx, nvar, 0.0);

    // Condense the time-varying prediction model into affine functions of U0,U1.
    for (int j = 0; j < Np; ++j) {
        const LinearizedModel& lm = horizon.models[static_cast<std::size_t>(j)];
        const std::size_t block = static_cast<std::size_t>(std::min(j, Nc - 1));

        M = mat3TimesDense(lm.A, M);
        addBToControlBlock(M, lm.B, block);
        c = mat3Vec(lm.A, c);
        c = subtractBRef(c, lm.B, horizon.ref_inputs[static_cast<std::size_t>(j)]);
        c[2] = wrapToPi(c[2]);

        const ReferencePoint& rnext = horizon.ref_states[static_cast<std::size_t>(j + 1)];
        const Matrix3x3 W = localWeight(adaptive.Q, rnext.heading);
        addStateCost(qp.P, qp.q, M, c, W);
    }

    // Control-deviation cost for j = 0,...,N_c-1.
    for (int j = 0; j < Nc; ++j) {
        addQuadraticBlock(qp.P, static_cast<std::size_t>(j), adaptive.R, 2.0);
        const std::vector<double> uref = controlToVector(horizon.ref_inputs[static_cast<std::size_t>(j)]);
        addLinearBlock(qp.q, static_cast<std::size_t>(j), adaptive.R, uref, -2.0);
    }

    // Increment cost: Delta u_0 = u_0 - u_{k-1}; Delta u_1 = u_1 - u_0.
    const std::vector<double> uprev = controlToVector(previous_input);
    addQuadraticBlock(qp.P, 0, adaptive.S, 2.0);
    addLinearBlock(qp.q, 0, adaptive.S, uprev, -2.0);

    addQuadraticBlock(qp.P, 0, adaptive.S, 2.0);
    addQuadraticBlock(qp.P, 1, adaptive.S, 2.0);
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t cidx = 0; cidx < 3; ++cidx) {
            qp.P(r, 3 + cidx) += -2.0 * adaptive.S(r,cidx);
            qp.P(3 + r, cidx) += -2.0 * adaptive.S(r,cidx);
        }
    }

    // Small regularization improves numerical conditioning without changing the formulation materially.
    for (std::size_t i = 0; i < nvar; ++i) qp.P(i,i) += 1e-9;

    // Constraint rows: absolute input bounds (6), first increment (3), second increment (3),
    // and equal/opposite steering equalities (2): total 14 rows.
    const std::size_t m = 14;
    qp.A = DenseMatrix(m, nvar, 0.0);
    qp.l.assign(m, 0.0);
    qp.u.assign(m, 0.0);
    std::size_t row = 0;

    const double v_upper = std::min(params_.max_speed, adaptive.speed_upper_bound);

    // Absolute bounds for U0 and U1.
    for (int block = 0; block < Nc; ++block) {
        const std::size_t c0 = static_cast<std::size_t>(block) * 3;
        // v
        qp.A(row, c0) = 1.0;
        qp.l[row] = params_.min_speed;
        qp.u[row] = v_upper;
        ++row;
        // delta_f
        qp.A(row, c0 + 1) = 1.0;
        qp.l[row] = -params_.max_steering;
        qp.u[row] = params_.max_steering;
        ++row;
        // delta_r
        qp.A(row, c0 + 2) = 1.0;
        qp.l[row] = -params_.max_steering;
        qp.u[row] = params_.max_steering;
        ++row;
    }

    const double du_max[3] = {
        params_.max_delta_speed,
        params_.max_delta_steering,
        params_.max_delta_steering
    };
    const double prev[3] = { previous_input.v, previous_input.delta_f, previous_input.delta_r };

    // First increment: U0 - U_previous.
    for (std::size_t d = 0; d < 3; ++d) {
        qp.A(row, d) = 1.0;
        qp.l[row] = prev[d] - du_max[d];
        qp.u[row] = prev[d] + du_max[d];
        ++row;
    }

    // Second increment: U1 - U0.
    for (std::size_t d = 0; d < 3; ++d) {
        qp.A(row, d) = -1.0;
        qp.A(row, 3 + d) = 1.0;
        qp.l[row] = -du_max[d];
        qp.u[row] = du_max[d];
        ++row;
    }

    // Four-wheel steering constraint: delta_f + delta_r = 0.
    for (int block = 0; block < Nc; ++block) {
        const std::size_t c0 = static_cast<std::size_t>(block) * 3;
        qp.A(row, c0 + 1) = 1.0;
        qp.A(row, c0 + 2) = 1.0;
        qp.l[row] = 0.0;
        qp.u[row] = 0.0;
        ++row;
    }

    if (row != m) throw std::logic_error("internal QP constraint count mismatch");
    return qp;
}

MpcStepResult LtvMpcController::compute(const Pose2D& robot,
                                        const ControlInput& previous_input,
                                        const Pose2D& target,
                                        const ReferencePath& path)
{
    if (path.size() < 2) throw std::invalid_argument("reference path is empty");
    if (!std::isfinite(robot.x) || !std::isfinite(robot.y) || !std::isfinite(robot.yaw) ||
        !std::isfinite(target.x) || !std::isfinite(target.y) ||
        !std::isfinite(previous_input.v) || !std::isfinite(previous_input.delta_f) ||
        !std::isfinite(previous_input.delta_r)) {
        throw std::invalid_argument("DCA-MPC input contains a non-finite value");
    }

    if (!path.curvatureWithinLimit(scheduler_.parameters().curvature_limit)) {
        throw std::invalid_argument(
            "reference path exceeds kappa_lim = 0.4 1/m; smooth or constrain it before tracking");
    }

    const PathProjection projection = path.project(robot.x, robot.y);
    if (!projection.valid) throw std::runtime_error("failed to project robot onto reference path");

    const PathProjection target_projection = path.project(target.x, target.y);
    if (!target_projection.valid) throw std::runtime_error("failed to project target onto reference path");
    if (target_projection.s + 1e-6 < projection.s) {
        throw std::invalid_argument(
            "current target lies behind the robot path projection; switch the active target before computing DCA-MPC");
    }

    const std::size_t ik = path.matchedIndex(projection);
    const double kappa_k = path.at(ik).curvature;

    MpcStepResult out;
    out.projection_arc_length = projection.s;
    out.target_arc_length = target_projection.s;
    out.adaptive = scheduler_.evaluate(robot, target, kappa_k);

    const double target_s = target_projection.s;
    const double lookahead_s = std::min(projection.s + out.adaptive.lookahead_distance,
                                        target_s);
    out.lookahead = path.interpolateByArcLength(lookahead_s);

    const HorizonData horizon = buildHorizon(path, projection, out.adaptive, target_s);
    const DenseQpProblem qp = buildQp(robot, previous_input, horizon, out.adaptive);
    const DenseQpResult sol = solver_.solve(qp, warm_start_);

    out.solved = sol.solved;
    out.qp_iterations = sol.iterations;
    out.primal_residual = sol.primal_residual;
    out.dual_residual = sol.dual_residual;

    if (sol.solved && sol.x.size() >= 3) {
        out.command = vectorToControl(sol.x, 0);
        // Enforce exact steering symmetry at the API boundary to remove tiny numerical residuals.
        const double delta = 0.5 * (out.command.delta_f - out.command.delta_r);
        out.command.delta_f = delta;
        out.command.delta_r = -delta;
        warm_start_ = sol.x;
    } else {
        // Do not expose an unconverged ADMM iterate as an optimized control action.
        // The integration layer must decide how to handle a failed solve.
        out.command = previous_input;
        out.fallback_used = true;
        warm_start_.clear();
    }

    return out;
}

} // namespace dca_mpc
