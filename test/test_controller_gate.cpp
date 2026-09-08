// A5.1 C++ controller wiring tests (mirrors mpc_core/tests/test_a51_gate.py).
#include <cmath>
#include <cstdio>
#include <vector>

#include "linear_mpc_controller/mpc/linear_mpc.hpp"

using linear_mpc_controller::HealthState;
using linear_mpc_controller::LinearMpcController;
using linear_mpc_controller::MpcCycleResult;
using linear_mpc_controller::MpcParams;
using linear_mpc_controller::TrackPoint;

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

static bool approx(double a, double b, double tol = 1e-6)
{
  return std::fabs(a - b) < tol;
}

int main()
{
  MpcParams params;
  params.qp_max_iter = 300;      // offline speed
  params.N = 15;
  LinearMpcController ctrl(params, straight(8.0));

  // -- anchor cycle accepts the first projection ----------------------------
  MpcCycleResult r1 = ctrl.computeCycle(1.0, 0.0, 0.0, 0.4, 0.0);
  if (!(r1.health == HealthState::OK && approx(ctrl.acceptedArc(), 1.0, 0.03))) {
    std::printf("FAIL anchor: health=%d acc=%.6f\n", (int)r1.health, ctrl.acceptedArc());
    return 1;
  }
  // -- no-motion cycle: accept, budget stays zero ---------------------------
  MpcCycleResult r2 = ctrl.computeCycle(1.0, 0.0, 0.0, 0.4, 0.0);
  if (!(r2.health == HealthState::OK && approx(ctrl.acceptedArc(), 1.0, 0.03))) {
    std::printf("FAIL idle: acc=%.6f\n", ctrl.acceptedArc());
    return 1;
  }
  // -- 3 m pose teleport: rejected, accepted arc frozen, speed limited ------
  MpcCycleResult rj = ctrl.computeCycle(4.0, 0.0, 0.0, 0.4, 0.0);
  if (!(approx(ctrl.acceptedArc(), 1.0, 0.03))) {
    std::printf("FAIL jump accepted: acc=%.6f (want 1.0)\n", ctrl.acceptedArc());
    return 1;
  }
  if (!(rj.v_cmd <= params.v_probation + 1e-6)) {
    std::printf("FAIL reject speed cap: v_cmd=%.6f\n", rj.v_cmd);
    return 1;
  }
  // -- A4.2: persistent rejection enters SEEKING and commits the TRUE arc --
  // Mirrors mpc_core/tests/test_a51_gate.py::
  //   test_reacquire_recovers_committed_baseline_then_normal
  // The old C++ expectation ("persistent rejection -> PROJECTION_LOST
  // immediately") belonged to the pre-A4.2 subset: once the full protocol is
  // wired, a STABLE pose is not lost, it is re-acquired.
  bool saw_seeking = false;
  MpcCycleResult rpark;
  for (int i = 0; i < 60; ++i) {           // robot parked at 4.0
    rpark = ctrl.computeCycle(4.0, 0.0, 0.0, 0.4, 0.0);
    if (rpark.reason == "REACQUIRE_SEEKING") {
      saw_seeking = true;
    }
  }
  if (!saw_seeking) {
    std::printf("FAIL reacquire: never entered A4.2 seeking\n");
    return 1;
  }
  if (!approx(ctrl.acceptedArc(), 4.0, 1e-6)) {
    std::printf("FAIL reacquire commit: acc=%.6f (want 4.0)\n", ctrl.acceptedArc());
    return 1;
  }
  if (ctrl.inProbation()) {
    std::printf("FAIL reacquire: probation should have expired after 60 cycles\n");
    return 1;
  }
  if (ctrl.reacquireCount() < 1) {
    std::printf("FAIL reacquire: no reacquire event recorded\n");
    return 1;
  }
  // and tracking continues forward from the committed baseline
  MpcCycleResult rfwd = ctrl.computeCycle(4.04, 0.0, 0.0, 0.4, 0.0);
  if (!(rfwd.health == HealthState::OK && approx(ctrl.acceptedArc(), 4.04, 1e-6))) {
    std::printf("FAIL post-reacquire track: health=%d acc=%.6f\n",
      (int)rfwd.health, ctrl.acceptedArc());
    return 1;
  }
  // -- new reference resets the gate ----------------------------------------
  ctrl.setReference(straight(8.0));
  MpcCycleResult ra = ctrl.computeCycle(2.0, 0.0, 0.0, 0.4, 0.0);
  if (!(ra.health == HealthState::OK && approx(ctrl.acceptedArc(), 2.0, 0.03))) {
    std::printf("FAIL reset anchor: acc=%.6f\n", ctrl.acceptedArc());
    return 1;
  }
  std::printf("all LinearMpcController A5.1 gate tests passed\n");
  return 0;
}
