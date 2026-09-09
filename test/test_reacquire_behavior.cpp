// A4.2 reacquire PROTOCOL behavior tests driven through the REAL controller
// and REAL OSQP solves (not source inspection).  Registered only when
// LINEAR_MPC_WITH_OSQP is ON.
//
// Claims under test (review P1-1):
//   1. SEEKING runs NO QP: every seeking cycle returns a zero command with
//      qp_iterations == 0 and the default kFailed status, and NONE of those
//      cycles is accountable toward the node's QP-failure run -- even a
//      whole seeking window of them never trips QpFailMonitor(max=3).  The
//      bounded exit is PROJECTION_LOST at reacquire_timeout_steps.
//   2. PROBATION DOES run the QP: real post-commit probation cycles solve
//      (qp_iterations > 0), the observation window lasts EXACTLY
//      probation_steps cycles, no exception is raised inside it, and every
//      shaped output is bounded (|v| <= v_probation, |omega| <= omega_max).
//   3. After the window the controller returns to NORMAL tracking on the
//      committed arc without any intervention.
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "linear_mpc_controller/mpc/linear_mpc.hpp"
#include "linear_mpc_controller/safety/qp_fail_monitor.hpp"
#include "linear_mpc_controller/safety/reacquire_command.hpp"

using linear_mpc_controller::HealthState;
using linear_mpc_controller::LinearMpcController;
using linear_mpc_controller::MpcCycleResult;
using linear_mpc_controller::MpcParams;
using linear_mpc_controller::QpFailMonitor;
using linear_mpc_controller::QpSolution;
using linear_mpc_controller::ReacquireCommandParams;
using linear_mpc_controller::TrackPoint;
using linear_mpc_controller::qpFailureAccountable;
using linear_mpc_controller::shapeReacquireCommand;

static int failures = 0;

static void check(bool ok, const char * what)
{
  if (!ok) {
    std::printf("FAIL %s\n", what);
    ++failures;
  }
}

static bool near(double a, double b, double tol = 1e-6)
{
  return std::fabs(a - b) < tol;
}

static std::vector<TrackPoint> straight(double length, double ds = 0.02)
{
  std::vector<TrackPoint> t;
  const int n = static_cast<int>(length / ds) + 1;
  for (int i = 0; i < n; ++i) {
    TrackPoint p;
    p.s = i * ds;
    p.x = i * ds;
    p.y = 0.0;
    p.yaw = 0.0;
    p.v = 0.5;
    t.push_back(p);
  }
  return t;
}

static ReacquireCommandParams nodeLikeParams(const MpcParams & params)
{
  // mirrors what computeVelocityCommands fills into ReacquireCommandParams
  ReacquireCommandParams rp;
  rp.v_max = params.v_max;
  rp.omega_max = params.omega_max;
  rp.a_max = params.a_max;
  rp.v_probation = params.v_probation;
  rp.terminal_stop_margin = 0.20;
  return rp;
}

// -- Scenario A ------------------------------------------------------------
// Teleport 3 m ahead of the accepted arc, let the A5.1 gate reject the jump
// into SEEKING, then keep the candidate UNSTABLE (alternating nearest
// segments) so the protocol never commits and must time out.  Every seeking
// cycle must be QP-free, zero-commanded and never QP-fail-accountable.
static void scenarioSeekingWindowNeverCountsAsQpFail()
{
  MpcParams params;
  params.qp_max_iter = 300;      // offline speed
  params.N = 15;
  LinearMpcController ctrl(params, straight(8.0));

  MpcCycleResult r0 = ctrl.computeCycle(1.0, 0.0, 0.0, 0.4, 0.0);
  check(
    r0.health == HealthState::OK && near(ctrl.acceptedArc(), 1.0, 0.03),
    "scenario A: anchor accepted at s=1.0");
  if (r0.health != HealthState::OK) {
    return;   // can't build the scenario; report and move on
  }

  QpFailMonitor mon(3);          // the node's qp_fail_max default
  bool seeking_seen = false;
  bool alt = false;
  int seek_ok = 0;
  bool lost = false;
  MpcCycleResult r;

  for (int i = 0; i < 80 && !lost; ++i) {
    const double x = alt ? ((i % 2 == 0) ? 4.0 : 6.0) : 4.0;
    r = ctrl.computeCycle(x, 0.0, 0.0, 0.4, 0.0);
    if (!r.reacquire_seeking) {
      // reject-phase cycles run the QP on the frozen baseline: mirror the
      // node and feed only accountable cycles to the monitor
      if (qpFailureAccountable(r)) {
        mon.record(r);
      }
      continue;
    }
    if (!seeking_seen) {
      seeking_seen = true;
      alt = true;   // from now on alternate so the run never stabilizes
    }
    if (r.health == HealthState::EMERGENCY_STOP) {
      lost = true;
      check(
        r.reason.find("PROJECTION_LOST") != std::string::npos,
        "scenario A: the bounded SEEKING exit is PROJECTION_LOST");
      check(
        r.qp_iterations == 0 && r.v_cmd == 0.0,
        "scenario A: the lost cycle is still QP-free and zero-commanded");
      if (qpFailureAccountable(r)) {
        mon.record(r);
      }
      continue;
    }
    ++seek_ok;
    check(r.health == HealthState::OK, "scenario A: seeking cycle stays OK");
    check(
      r.qp_iterations == 0 && r.qp_status == QpSolution::Status::kFailed,
      "scenario A: SEEKING runs no QP (0 iterations, default kFailed)");
    check(
      r.v_cmd == 0.0 && r.omega_cmd == 0.0,
      "scenario A: SEEKING commands decel-to-zero");
    check(
      !qpFailureAccountable(r),
      "scenario A: a SEEKING cycle must never be QP-fail-accountable");
    mon.record(r);   // must be ignored by the monitor
  }

  check(seeking_seen, "scenario A: persistent rejection entered SEEKING");
  check(
    seek_ok == params.reacquire_timeout_steps,
    "scenario A: exactly reacquire_timeout_steps zero-command seeking cycles");
  check(lost, "scenario A: the unstable run exits via the SEEKING timeout");
  check(
    mon.run() == 0 && !mon.aborted(),
    "scenario A: a whole SEEKING window of kFailed cycles never trips the "
    "QP-fail monitor");
}

// -- Scenario B ------------------------------------------------------------
// Park 3 m ahead on the track, reject into SEEKING, let the stable candidate
// COMMIT.  During the observation window the QP must really run every cycle,
// the window must last exactly probation_steps cycles, no exception may be
// raised, every shaped output must respect the probation caps, and the
// controller must return to NORMAL tracking on the committed arc.
static void scenarioProbationRunsQpBoundedAndRecovers()
{
  MpcParams params;
  params.qp_max_iter = 300;      // offline speed
  params.N = 15;
  LinearMpcController ctrl(params, straight(8.0));

  MpcCycleResult r0 = ctrl.computeCycle(1.0, 0.0, 0.0, 0.4, 0.0);
  check(
    r0.health == HealthState::OK && near(ctrl.acceptedArc(), 1.0, 0.03),
    "scenario B: anchor accepted at s=1.0");
  if (r0.health != HealthState::OK) {
    return;
  }

  const ReacquireCommandParams rp = nodeLikeParams(params);
  int in_prob_total = 0;         // commit cycle + probation tracking cycles
  int prob_normal = 0;           // probation cycles that really ran the QP
  int commit_cycles = 0;
  bool cap_binds = false;
  MpcCycleResult r;

  for (int i = 0; i < 80; ++i) {
    r = ctrl.computeCycle(4.0, 0.0, 0.0, 0.4, 0.0);
    if (!r.in_probation) {
      continue;
    }
    ++in_prob_total;
    if (r.reacquire_seeking) {
      ++commit_cycles;   // the atomic-commit cycle: still decel-to-zero
      check(
        r.v_cmd == 0.0 && r.qp_iterations == 0,
        "scenario B: the commit cycle is still a no-QP decel beat");
      continue;
    }
    ++prob_normal;
    // PROBATION really runs the QP on every observation cycle
    check(
      (r.qp_status == QpSolution::Status::kSolved ||
       r.qp_status == QpSolution::Status::kApproximate) &&
        r.qp_iterations > 0,
      "scenario B: a probation cycle must actually run and solve the QP");
    check(r.health == HealthState::OK, "scenario B: probation cycle is OK");
    // output bounds through the node's own shaper
    double vo = 0.0, wo = 0.0;
    shapeReacquireCommand(r, rp, 8.0, vo, wo);
    check(
      vo <= params.v_probation + 1e-9 && vo >= -params.v_probation - 1e-9,
      "scenario B: shaped |v| is bounded by v_probation during probation");
    check(
      wo <= params.omega_max + 1e-9 && wo >= -params.omega_max - 1e-9,
      "scenario B: shaped |omega| is bounded by omega_max");
    if (near(vo, params.v_probation, 1e-9)) {
      cap_binds = true;   // the tracker asks for more than 0.15 -> cap bites
    }
  }

  check(ctrl.reacquireCount() == 1, "scenario B: one reacquire event");
  check(
    in_prob_total == params.probation_steps,
    "scenario B: the observation window lasts exactly probation_steps cycles");
  check(
    prob_normal == params.probation_steps - 1,
    "scenario B: probation runs the QP on probation_steps - 1 cycles");
  check(commit_cycles == 1, "scenario B: exactly one atomic commit");
  check(cap_binds, "scenario B: the v_probation cap visibly binds");
  check(!ctrl.inProbation(), "scenario B: probation expired on its own");

  // back to NORMAL tracking on the committed arc, no intervention needed
  MpcCycleResult rf = ctrl.computeCycle(4.04, 0.0, 0.0, 0.4, 0.0);
  check(
    rf.health == HealthState::OK && !rf.in_probation && !rf.reacquire_seeking,
    "scenario B: the controller returns to NORMAL tracking");
  check(
    near(ctrl.acceptedArc(), 4.04, 1e-6),
    "scenario B: tracking advances from the committed baseline");
}

int main()
{
  scenarioSeekingWindowNeverCountsAsQpFail();
  scenarioProbationRunsQpBoundedAndRecovers();

  if (failures) {
    std::printf("%d A4.2 reacquire behavior check(s) failed\n", failures);
    return 1;
  }
  std::printf("all A4.2 reacquire behavior tests passed\n");
  return 0;
}
