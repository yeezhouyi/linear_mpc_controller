// Receding-horizon linear MPC controller (U3) -- orchestration only, mirrors
// mpc_core/mpc.py.  All math lives in differential_drive_model / qp_problem.
#ifndef LINEAR_MPC_CONTROLLER__MPC__LINEAR_MPC_HPP_
#define LINEAR_MPC_CONTROLLER__MPC__LINEAR_MPC_HPP_

#include <memory>
#include <string>
#include <vector>

#include "linear_mpc_controller/model/differential_drive_model.hpp"
#include "linear_mpc_controller/model/projection_gate.hpp"
#include "linear_mpc_controller/model/projection_search.hpp"
#include "linear_mpc_controller/model/reacquire.hpp"
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
  // ---- A5.2/A5.3 accepted-arc diagnostics -----------------------------
  double accepted_arc = 0.0;      // A5.1 baseline after this cycle
  int projection_stage = 2;       // 0/1/2 window, 3 ambiguous (stop)
  int reject_run = 0;
  // ---- A4.2 reacquire protocol diagnostics ----------------------------
  bool in_probation = false;      // speed capped; completion must say no
  int reacquire_count = 0;        // lifetime reacquire events (never reset)
  bool reacquire_seeking = false; // decel-to-zero cycle, no QP
};

class LinearMpcController
{
public:
  LinearMpcController(const MpcParams & params, std::vector<TrackPoint> traj);

  void setReference(std::vector<TrackPoint> traj);
  MpcCycleResult computeCycle(double px, double py, double yaw, double v, double omega);

  // A5 diagnostics for tests/audits
  double acceptedArc() const { return gate_.accepted_arc; }
  int rejectRun() const { return reject_run_; }
  // A4.2 diagnostics for tests/audits
  bool inSeeking() const { return reacq_.inSeeking(); }
  bool inProbation() const { return reacq_.inProbation(); }
  int reacquireCount() const { return reacq_.reacquire_events; }
  const ReacquireState & reacquire() const { return reacq_; }

private:
  double maxConstraintViolation(const Eigen::Vector4d & x0,
    const CondensedMpcProblem & prob, const Eigen::VectorXd & U) const;
  /// (anchor, err) at the ACCEPTED arc (A5.0/A3.1): e_y/e_psi recomputed in
  /// the accepted arc's tangent frame, never at the raw candidate.
  void frenetErrorAtArc(double px, double py, double yaw, double v, double omega,
    TrackPoint & anchor, Eigen::Vector4d & err) const;

  MpcParams params_;
  std::vector<TrackPoint> traj_;
  FallbackPolicy fallback_;
  std::unique_ptr<QpSolver> solver_;
  Eigen::VectorXd warm_;
  bool have_warm_ = false;
  long long cycle_ = 0;
  // ---- A5.1 acceptance-gate state (reset with the reference, A5.1 note) --
  ProjectionGateState gate_;
  bool gate_initialized_ = false;
  int reject_run_ = 0;
  // ---- A4.2 reacquire protocol state (mirror of mpc.py) -----------------
  ReacquireState reacq_;
};

}  // namespace linear_mpc_controller

#endif  // LINEAR_MPC_CONTROLLER__MPC__LINEAR_MPC_HPP_
