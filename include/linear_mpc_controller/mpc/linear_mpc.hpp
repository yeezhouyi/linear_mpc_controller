// Receding-horizon linear MPC controller (U3) -- orchestration only, mirrors
// mpc_core/mpc.py.  All math lives in differential_drive_model / qp_problem.
#ifndef LINEAR_MPC_CONTROLLER__MPC__LINEAR_MPC_HPP_
#define LINEAR_MPC_CONTROLLER__MPC__LINEAR_MPC_HPP_

#include <memory>
#include <string>
#include <vector>

#include "linear_mpc_controller/model/differential_drive_model.hpp"
#include "linear_mpc_controller/mpc/fallback_policy.hpp"
#include "linear_mpc_controller/mpc/qp_problem.hpp"

namespace linear_mpc_controller
{

struct MpcCycleResult
{
  double v_cmd = 0.0;
  double omega_cmd = 0.0;
  HealthState health = HealthState::OK;
  std::string reason;            // needs <string>
  QpSolution::Status qp_status = QpSolution::Status::kFailed;
  int qp_iterations = 0;
  double qp_time_us = 0.0;
  double constraint_violation = 0.0;
  bool fallback_used = false;
  Eigen::Vector4d e_used = Eigen::Vector4d::Zero();
};

class LinearMpcController
{
public:
  LinearMpcController(const MpcParams & params, std::vector<TrackPoint> traj);

  void setReference(std::vector<TrackPoint> traj);
  MpcCycleResult computeCycle(double px, double py, double yaw, double v, double omega);

private:
  double maxConstraintViolation(const Eigen::Vector4d & x0,
    const CondensedMpcProblem & prob, const Eigen::VectorXd & U) const;

  MpcParams params_;
  std::vector<TrackPoint> traj_;
  FallbackPolicy fallback_;
  std::unique_ptr<QpSolver> solver_;
  Eigen::VectorXd warm_;
  bool have_warm_ = false;
  long long cycle_ = 0;
};

}  // namespace linear_mpc_controller

#endif  // LINEAR_MPC_CONTROLLER__MPC__LINEAR_MPC_HPP_
