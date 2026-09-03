#include "linear_mpc_controller/mpc/linear_mpc.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace linear_mpc_controller
{

LinearMpcController::LinearMpcController(const MpcParams & params, std::vector<TrackPoint> traj)
: params_(params), traj_(std::move(traj)), fallback_(FallbackParams{
    params_.Ts, params_.v_min, params_.v_max, params_.omega_max,
    0.8, 10})
{
  solver_ = makeDefaultSolver(params_.qp_max_iter, params_.qp_abs_tol, params_.qp_rel_tol);
}

void LinearMpcController::setReference(std::vector<TrackPoint> traj)
{
  traj_ = std::move(traj);
  fallback_.reset();
  have_warm_ = false;
}

MpcCycleResult LinearMpcController::computeCycle(double px, double py, double yaw, double v, double omega)
{
  ++cycle_;
  MpcCycleResult res;
  if (traj_.size() < 2) {
    res.health = HealthState::NO_REFERENCE;
    res.reason = "no reference set";
    res.fallback_used = true;
    return res;
  }

  TrackPoint anchor;
  const Eigen::Vector4d x0 = frenetState(traj_, px, py, yaw, v, omega, params_.lookahead_m, anchor);
  res.e_used = x0;

  CondensedMpcProblem prob(params_, traj_, anchor.s);
  Eigen::MatrixXd H, C;
  Eigen::VectorXd q, l, u;
  prob.build(x0, H, q, C, l, u);

  QpSolution sol = solver_->solve(H, q, C, l, u, have_warm_ ? warm_ : Eigen::VectorXd());
  res.qp_status = sol.status;
  res.qp_iterations = sol.iterations;
  res.qp_time_us = sol.solve_time_us;

  if (sol.status == QpSolution::Status::kSolved ||
      sol.status == QpSolution::Status::kApproximate) {
    // shift warm start: drop first block, pad zeros
    warm_ = Eigen::VectorXd::Zero(2 * params_.N);
    if (2 * params_.N > 2) {
      warm_.head(2 * params_.N - 2) = sol.u.tail(2 * params_.N - 2);
    }
    have_warm_ = true;
    const double a0 = sol.u(0);
    const double alpha0 = sol.u(1);
    const double v_cmd = x0(kV) + params_.Ts * a0;
    const double w_cmd = x0(kOmega) + params_.Ts * alpha0;
    res.constraint_violation = maxConstraintViolation(x0, prob, sol.u);

    int stage = 0;
    HealthState h = HealthState::OK;
    fallback_.apply(v_cmd, w_cmd, HealthState::OK, res.v_cmd, res.omega_cmd, h, stage);
    res.health = h;
    res.fallback_used = stage >= 2;
    if (h != HealthState::OK) res.reason = "clamped/degraded by fallback";
    return res;
  }

  // solver failure -> fallback from zero (deterministic stop)
  int stage = 0;
  HealthState h = HealthState::QP_INFEASIBLE;
  fallback_.apply(0.0, 0.0, HealthState::QP_INFEASIBLE, res.v_cmd, res.omega_cmd, h, stage);
  res.health = h;
  res.reason = "qp failed";
  res.fallback_used = true;
  have_warm_ = false;
  return res;
}

double LinearMpcController::maxConstraintViolation(const Eigen::Vector4d & x0,
  const CondensedMpcProblem & prob, const Eigen::VectorXd & U) const
{
  const int N = params_.N;
  double viol = 0.0;
  Eigen::Vector4d x = x0;
  for (int k = 0; k < N; ++k) {
    x = prob.A_d()[k] * x + prob.B_d()[k] * U.segment<2>(2 * k);
    viol = std::max({viol, params_.v_min - x(kV), x(kV) - params_.v_max,
      std::fabs(x(kOmega)) - params_.omega_max});
  }
  return std::max(viol, 0.0);
}

}  // namespace linear_mpc_controller
