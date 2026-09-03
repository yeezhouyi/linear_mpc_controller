// Condensed QP problem builder + solver interface (U3).
//
// Mirrors the verified Python builder in mpc_core/mpc.py: dense condensed QP
// over the stacked input sequence with v / omega / accel hard constraints.
//
// Solver: default backend is OSQP (plan KTD12).  This file compiles without
// OSQP when LINEAR_MPC_HAVE_OSQP is not defined -- an UnavailableSolver is
// used so model/fallback tests still build; the OSQP-backed solver is wired
// in the WSL2/ros2 build (see CMakeLists).
#ifndef LINEAR_MPC_CONTROLLER__MPC__QP_PROBLEM_HPP_
#define LINEAR_MPC_CONTROLLER__MPC__QP_PROBLEM_HPP_

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "linear_mpc_controller/model/differential_drive_model.hpp"

namespace linear_mpc_controller
{

struct MpcParams
{
  double Ts = 0.05;
  int N = 25;
  Eigen::Matrix4d Q = Eigen::Vector4d(60.0, 10.0, 2.0, 1.0).asDiagonal();
  Eigen::Matrix4d Q_F = Eigen::Vector4d(120.0, 20.0, 4.0, 2.0).asDiagonal();
  Eigen::Matrix2d S = Eigen::Vector2d(0.5, 0.5).asDiagonal();
  double v_min = 0.0, v_max = 1.5;
  double omega_max = 2.0;
  double a_max = 1.0, alpha_max = 2.0;
  double lookahead_m = 0.0;
  int qp_max_iter = 1500;
  double qp_abs_tol = 1e-6, qp_rel_tol = 1e-5;
};

struct QpSolution
{
  enum class Status { kSolved, kApproximate, kFailed };
  Status status = Status::kFailed;
  Eigen::VectorXd u;         // stacked input sequence (2N)
  int iterations = 0;
  double pri_res = 0.0, dua_res = 0.0;
  double objective = 0.0;
  double solve_time_us = 0.0;
};

/// Abstract QP solver (same contract as the Python AdmmQp / C++ OSQP wrapper).
class QpSolver
{
public:
  virtual ~QpSolver() = default;
  /// Solve min 0.5 z'Hz + q'z  s.t.  l <= Cz <= u (dense).
  virtual QpSolution solve(const Eigen::MatrixXd & H, const Eigen::VectorXd & q,
    const Eigen::MatrixXd & C, const Eigen::VectorXd & l, const Eigen::VectorXd & u,
    const Eigen::VectorXd & warm_start) = 0;
};

/// Returns a solver.  Without OSQP: an UnavailableSolver (all solves FAILED).
/// With LINEAR_MPC_HAVE_OSQP: the OSQP-backed dense solver.
std::unique_ptr<QpSolver> makeDefaultSolver(int max_iter, double abs_tol, double rel_tol);

/// Condensed QP for one cycle (decision z = [a_0..a_{N-1}, alpha_0..]).
/// x0 is the current error state; refs are sampled every Ts at the preview arc.
class CondensedMpcProblem
{
public:
  CondensedMpcProblem(const MpcParams & p, const std::vector<TrackPoint> & traj, double base_arc);

  /// Fill H, q, C, l, u for the current state; returns variable count.
  int build(const Eigen::Vector4d & x0, Eigen::MatrixXd & H, Eigen::VectorXd & q,
    Eigen::MatrixXd & C, Eigen::VectorXd & l, Eigen::VectorXd & u) const;

  /// Reference state for step k (0-indexed over predicted states 1..N).
  Eigen::Vector4d referenceState(int k) const;

  int variableCount() const { return 2 * params_.N; }
  const std::vector<Eigen::Matrix4d> & A_d() const { return A_d_; }
  const std::vector<Eigen::Matrix<double, 4, 2>> & B_d() const { return B_d_; }

private:
  MpcParams params_;
  std::vector<Eigen::Matrix4d> A_d_;
  std::vector<Eigen::Matrix<double, 4, 2>> B_d_;
  std::vector<TrackPoint> anchors_;
};

}  // namespace linear_mpc_controller

#endif  // LINEAR_MPC_CONTROLLER__MPC__QP_PROBLEM_HPP_
