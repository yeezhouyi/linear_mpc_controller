// B6B.2 QP-failure accounting, extracted from computeVelocityCommands so the
// abort policy is unit-testable without a live Nav2 controller node.
//
// Policy (verbatim mirror of the node block it replaces):
//   * a NORMAL-tracking cycle whose QP returns kFailed increments the run;
//     when the run exceeds qp_fail_max the caller must emergency-stop
//     (the node clears the plan and throws NoValidControl).
//   * a later kSolved cycle grants ONE step of recovery credit (decrement).
//   * kApproximate neither increments nor decrements.
//   * A4.2 SEEKING / PROBATION cycles are NEVER accountable: SEEKING runs
//     no QP at all (qp_status stays at the default kFailed), so counting it
//     would clear the controller after a few seeking cycles instead of
//     letting the protocol time out on its own (reacquire_timeout_steps);
//     PROBATION is a self-bounded observation window inside which the
//     controller never throws.  The node returns before this monitor is
//     fed; qpFailureAccountable() keeps that exclusion even if a future
//     refactor changes the call order.
#ifndef LINEAR_MPC_CONTROLLER__SAFETY__QP_FAIL_MONITOR_HPP_
#define LINEAR_MPC_CONTROLLER__SAFETY__QP_FAIL_MONITOR_HPP_

#include "linear_mpc_controller/mpc/linear_mpc.hpp"

namespace linear_mpc_controller
{

/// True when a cycle must be counted toward the QP-failure run.  SEEKING
/// and PROBATION cycles are excluded (see header comment).
inline bool qpFailureAccountable(const MpcCycleResult & res)
{
  return !res.reacquire_seeking && !res.in_probation;
}

/// Node-level QP-failure run counter (B6B.2).
class QpFailMonitor
{
public:
  explicit QpFailMonitor(int max_fails = 3) : max_fails_(max_fails) {}

  void reset() { run_ = 0; }
  void setMaxFails(int max_fails) { max_fails_ = max_fails; }

  int run() const { return run_; }
  int maxFails() const { return max_fails_; }
  bool aborted() const { return run_ > max_fails_; }

  /// Feed one cycle.  Non-accountable cycles are ignored entirely (no
  /// increment, no recovery credit).  Returns true only when THIS cycle
  /// pushed the run past max_fails_ -- the caller performs the emergency
  /// stop.  Mirrors the node's original ++ / recovery-credit / abort.
  bool record(const MpcCycleResult & res)
  {
    if (!qpFailureAccountable(res)) {
      return false;
    }
    if (res.qp_status == QpSolution::Status::kFailed) {
      ++run_;
      if (run_ > max_fails_) {
        return true;
      }
    } else if (run_ > 0 && res.qp_status == QpSolution::Status::kSolved) {
      --run_;   // recovery credit: a clean solve walks the run back down
    }
    return false;
  }

private:
  int max_fails_;
  int run_ = 0;
};

}  // namespace linear_mpc_controller

#endif  // LINEAR_MPC_CONTROLLER__SAFETY__QP_FAIL_MONITOR_HPP_
