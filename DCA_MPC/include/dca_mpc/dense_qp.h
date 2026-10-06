#pragma once

#include <cstddef>
#include <vector>

namespace dca_mpc {

class DenseMatrix {
public:
    DenseMatrix();
    DenseMatrix(std::size_t rows, std::size_t cols, double value = 0.0);

    std::size_t rows() const { return rows_; }
    std::size_t cols() const { return cols_; }

    double& operator()(std::size_t r, std::size_t c);
    double operator()(std::size_t r, std::size_t c) const;

    void fill(double value);

private:
    std::size_t rows_;
    std::size_t cols_;
    std::vector<double> data_;
};

struct DenseQpProblem {
    // Minimize 0.5*x' P x + q' x subject to l <= A x <= u.
    DenseMatrix P;
    std::vector<double> q;
    DenseMatrix A;
    std::vector<double> l;
    std::vector<double> u;
};

struct DenseQpSettings {
    int max_iterations;
    double rho;
    double sigma;
    double absolute_tolerance;
    double relative_tolerance;

    DenseQpSettings();
};

struct DenseQpResult {
    bool solved;
    int iterations;
    double primal_residual;
    double dual_residual;
    std::vector<double> x;

    DenseQpResult();
};

// Lightweight dense ADMM solver for the very small MPC QP (6 decision variables
// for N_c = 2). It uses the standard l <= A x <= u form and introduces no
// platform-specific dependency, which keeps the research code portable.
class DenseAdmmQpSolver {
public:
    explicit DenseAdmmQpSolver(const DenseQpSettings& settings = DenseQpSettings());

    DenseQpResult solve(const DenseQpProblem& problem,
                        const std::vector<double>& warm_start = {}) const;

private:
    DenseQpSettings settings_;
};

} // namespace dca_mpc
