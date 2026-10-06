#include "dca_mpc/dense_qp.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace dca_mpc {

DenseMatrix::DenseMatrix() : rows_(0), cols_(0) {}

DenseMatrix::DenseMatrix(std::size_t rows, std::size_t cols, double value)
    : rows_(rows), cols_(cols), data_(rows * cols, value) {}

double& DenseMatrix::operator()(std::size_t r, std::size_t c)
{
    return data_.at(r * cols_ + c);
}

double DenseMatrix::operator()(std::size_t r, std::size_t c) const
{
    return data_.at(r * cols_ + c);
}

void DenseMatrix::fill(double value)
{
    std::fill(data_.begin(), data_.end(), value);
}

DenseQpSettings::DenseQpSettings()
    : max_iterations(1200), rho(5.0), sigma(1e-6),
      absolute_tolerance(2e-5), relative_tolerance(2e-4) {}

DenseQpResult::DenseQpResult()
    : solved(false), iterations(0), primal_residual(0.0), dual_residual(0.0) {}

namespace {

std::vector<double> matVec(const DenseMatrix& A, const std::vector<double>& x)
{
    if (A.cols() != x.size()) throw std::invalid_argument("matVec size mismatch");
    std::vector<double> y(A.rows(), 0.0);
    for (std::size_t r = 0; r < A.rows(); ++r) {
        for (std::size_t c = 0; c < A.cols(); ++c) y[r] += A(r,c) * x[c];
    }
    return y;
}

std::vector<double> transposedMatVec(const DenseMatrix& A, const std::vector<double>& x)
{
    if (A.rows() != x.size()) throw std::invalid_argument("transposedMatVec size mismatch");
    std::vector<double> y(A.cols(), 0.0);
    for (std::size_t r = 0; r < A.rows(); ++r) {
        for (std::size_t c = 0; c < A.cols(); ++c) y[c] += A(r,c) * x[r];
    }
    return y;
}

double infNorm(const std::vector<double>& x)
{
    double n = 0.0;
    for (double v : x) n = std::max(n, std::fabs(v));
    return n;
}

// Cholesky factorization for symmetric positive-definite matrices.
bool cholesky(const DenseMatrix& A, DenseMatrix& L)
{
    if (A.rows() != A.cols()) return false;
    const std::size_t n = A.rows();
    L = DenseMatrix(n, n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j <= i; ++j) {
            double sum = A(i,j);
            for (std::size_t k = 0; k < j; ++k) sum -= L(i,k) * L(j,k);
            if (i == j) {
                if (sum <= 1e-14) return false;
                L(i,j) = std::sqrt(sum);
            } else {
                L(i,j) = sum / L(j,j);
            }
        }
    }
    return true;
}

std::vector<double> solveCholesky(const DenseMatrix& L, const std::vector<double>& b)
{
    const std::size_t n = L.rows();
    if (L.cols() != n || b.size() != n) throw std::invalid_argument("solveCholesky size mismatch");
    std::vector<double> y(n, 0.0), x(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        double sum = b[i];
        for (std::size_t k = 0; k < i; ++k) sum -= L(i,k) * y[k];
        y[i] = sum / L(i,i);
    }
    for (std::size_t ii = 0; ii < n; ++ii) {
        const std::size_t i = n - 1 - ii;
        double sum = y[i];
        for (std::size_t k = i + 1; k < n; ++k) sum -= L(k,i) * x[k];
        x[i] = sum / L(i,i);
    }
    return x;
}

double clamp(double v, double lo, double hi)
{
    return std::max(lo, std::min(hi, v));
}

} // namespace

DenseAdmmQpSolver::DenseAdmmQpSolver(const DenseQpSettings& settings)
    : settings_(settings)
{
    if (settings_.max_iterations <= 0 || settings_.rho <= 0.0 || settings_.sigma <= 0.0) {
        throw std::invalid_argument("invalid QP settings");
    }
}

DenseQpResult DenseAdmmQpSolver::solve(const DenseQpProblem& problem,
                                       const std::vector<double>& warm_start) const
{
    const std::size_t n = problem.q.size();
    const std::size_t m = problem.l.size();
    if (n == 0 || problem.P.rows() != n || problem.P.cols() != n ||
        problem.A.rows() != m || problem.A.cols() != n || problem.u.size() != m) {
        throw std::invalid_argument("invalid QP dimensions");
    }
    for (std::size_t i = 0; i < m; ++i) {
        if (!std::isfinite(problem.l[i]) || !std::isfinite(problem.u[i]) ||
            problem.l[i] > problem.u[i]) {
            throw std::invalid_argument("invalid QP constraint bounds");
        }
    }

    DenseMatrix K(n, n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            double ata = 0.0;
            for (std::size_t r = 0; r < m; ++r) ata += problem.A(r,i) * problem.A(r,j);
            K(i,j) = problem.P(i,j) + settings_.rho * ata;
        }
        K(i,i) += settings_.sigma;
    }

    DenseMatrix L;
    if (!cholesky(K, L)) {
        throw std::runtime_error("QP KKT matrix is not positive definite");
    }

    std::vector<double> x(n, 0.0);
    if (warm_start.size() == n) x = warm_start;
    std::vector<double> Ax = matVec(problem.A, x);
    std::vector<double> z(m, 0.0), y(m, 0.0);
    for (std::size_t i = 0; i < m; ++i) z[i] = clamp(Ax[i], problem.l[i], problem.u[i]);

    DenseQpResult result;
    std::vector<double> z_prev(m, 0.0);

    for (int iter = 1; iter <= settings_.max_iterations; ++iter) {
        // x-update: solve (P + sigma I + rho A'A)x = -q + A'(rho z - y) + sigma*x_old.
        std::vector<double> rhoz_minus_y(m, 0.0);
        for (std::size_t i = 0; i < m; ++i) rhoz_minus_y[i] = settings_.rho * z[i] - y[i];
        std::vector<double> rhs = transposedMatVec(problem.A, rhoz_minus_y);
        for (std::size_t i = 0; i < n; ++i) rhs[i] += -problem.q[i] + settings_.sigma * x[i];
        x = solveCholesky(L, rhs);

        Ax = matVec(problem.A, x);
        z_prev = z;

        // Projection onto the box [l,u].
        for (std::size_t i = 0; i < m; ++i) {
            const double shifted = Ax[i] + y[i] / settings_.rho;
            z[i] = clamp(shifted, problem.l[i], problem.u[i]);
        }

        // Dual update.
        std::vector<double> primal(m, 0.0);
        for (std::size_t i = 0; i < m; ++i) {
            primal[i] = Ax[i] - z[i];
            y[i] += settings_.rho * primal[i];
        }

        std::vector<double> dz(m, 0.0);
        for (std::size_t i = 0; i < m; ++i) dz[i] = z[i] - z_prev[i];
        std::vector<double> dual = transposedMatVec(problem.A, dz);
        for (double& v : dual) v *= settings_.rho;

        const double primal_res = infNorm(primal);
        const double dual_res = infNorm(dual);
        const double eps_primal = settings_.absolute_tolerance +
            settings_.relative_tolerance * std::max(infNorm(Ax), infNorm(z));
        const std::vector<double> aty = transposedMatVec(problem.A, y);
        const double eps_dual = settings_.absolute_tolerance +
            settings_.relative_tolerance * infNorm(aty);

        result.iterations = iter;
        result.primal_residual = primal_res;
        result.dual_residual = dual_res;
        if (primal_res <= eps_primal && dual_res <= eps_dual) {
            result.solved = true;
            break;
        }
    }

    result.x = x;
    return result;
}

} // namespace dca_mpc
