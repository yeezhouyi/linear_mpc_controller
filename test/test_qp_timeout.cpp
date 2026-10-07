// Runtime deadline and fallback contract for the OSQP backend.
#include <cmath>
#include <cstdio>

#include <Eigen/Dense>

#include "linear_mpc_controller/mpc/fallback_policy.hpp"
#include "linear_mpc_controller/mpc/qp_problem.hpp"

using namespace linear_mpc_controller;

int main()
{
  constexpr int n = 600;
  Eigen::MatrixXd H = 1.0e-3 * Eigen::MatrixXd::Identity(n, n);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(n);
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(n, n);
  for (int i = 0; i < n; ++i) {
    q(i) = (i % 2 == 0) ? 1.0 : -1.0;
    C(i, i) = 1.0;
    if (i + 1 < n) C(i, i + 1) = 0.5;
  }
  Eigen::VectorXd l = Eigen::VectorXd::Constant(n, -0.1);
  Eigen::VectorXd u = Eigen::VectorXd::Constant(n, 0.1);

  // The profiling-enabled Jazzy OSQP vendor exposes a real time_limit.  A
  // sub-nanosecond budget must stop before convergence, independent of the
  // host's normal solve speed.
  auto solver = makeDefaultSolver(100000, 1e-6, 1e-5, 1e-6);
  const auto sol = solver->solve(H, q, C, l, u, Eigen::VectorXd::Zero(n));
  if (sol.status != QpSolution::Status::kTimeout) {
    std::printf("FAIL expected kTimeout, got status=%d iterations=%d time_us=%.1f\n",
      static_cast<int>(sol.status), sol.iterations, sol.solve_time_us);
    return 1;
  }
  if (sol.iterations < 0 || !std::isfinite(sol.solve_time_us)) {
    std::printf("FAIL timeout diagnostics are invalid\n");
    return 1;
  }

  FallbackPolicy fallback(FallbackParams{});
  double v = 123.0, omega = 456.0;
  HealthState health_out = HealthState::OK;
  int stage = 0;
  fallback.apply(0.0, 0.0, HealthState::QP_TIMEOUT,
    v, omega, health_out, stage);
  if (health_out != HealthState::FALLBACK_ACTIVE || stage != 2 ||
      !std::isfinite(v) || !std::isfinite(omega) || omega != 0.0) {
    std::printf("FAIL timeout fallback output v=%.6f omega=%.6f health=%d stage=%d\n",
      v, omega, static_cast<int>(health_out), stage);
    return 1;
  }
  std::printf("PASS timeout status=%d iterations=%d time_us=%.1f fallback_stage=%d\n",
    static_cast<int>(sol.status), sol.iterations, sol.solve_time_us, stage);
  return 0;
}
