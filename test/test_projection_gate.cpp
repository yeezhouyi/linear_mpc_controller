// A5B.1 ProjectionGateState tests -- C++ side of the A7.3 guard semantics.
// Each case mirrors the corresponding Python test in
// mpc_core/tests/test_progress_gate.py (same numbers where geometry-free).
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "linear_mpc_controller/model/projection_gate.hpp"

using linear_mpc_controller::GateDecision;
using linear_mpc_controller::ProjectionGateState;

static double near_zero(double a, double b, double tol = 1e-9)
{
  return std::fabs(a - b) < tol;
}

int main()
{
  // -- first call only anchors the pose; standing still banks nothing -------
  {
    ProjectionGateState g(1.5, 0.05);
    GateDecision d0 = g.step(0.0, 0.0, 0.0, 0.0);
    assert(d0.accepted);
    for (int i = 0; i < 20; ++i) {
      assert(g.step(0.0, 0.0, 0.0, 0.0).accepted);
    }
    assert(near_zero(g.budget, 0.0));
    // a 0.10 m raw-arc jump while the robot stands still must be rejected
    GateDecision dj = g.step(0.0, 0.0, 0.10, 0.0);
    assert(!dj.accepted);
    assert(near_zero(g.accepted_arc, 0.0));
  }
  // -- honest straight tracking is never rejected (0.04 m/cycle @ 1.5,0.05) --
  {
    ProjectionGateState g(1.5, 0.05);
    g.step(0.0, 0.0, 0.0, 0.0);
    double x = 0.0;
    int rejected = 0;
    for (int i = 0; i < 200; ++i) {
      x += 0.04;
      rejected += g.step(x, 0.0, x, 0.0).accepted ? 0 : 1;
    }
    assert(rejected == 0);
    assert(near_zero(g.accepted_arc, x, 1e-6));
  }
  // -- motion against the tangent banks ZERO (wrong lane cannot self-grant) --
  {
    ProjectionGateState g(1.5, 0.05, 1.0);   // baselined on lane A (+x)
    g.step(1.0, 0.0, 1.0, 0.0);              // anchor pose on lane A
    // move along the antiparallel lane: x decreases while yaw tangent is +x
    GateDecision d = g.step(1.0 - 0.04, 0.3, 4.0, 0.0);
    assert(near_zero(d.physical_ds, 0.0));
    assert(!d.accepted);
    assert(near_zero(g.accepted_arc, 1.0));
  }
  // -- localisation jump is capped, not banked ------------------------------
  {
    ProjectionGateState g(1.5, 0.05);
    g.step(0.0, 0.0, 0.0, 0.0);
    GateDecision d = g.step(5.0, 0.0, 0.0, 0.0);   // 5 m pose teleport
    assert(near_zero(d.physical_ds, g.stepCap()));
    assert(near_zero(g.stepCap(), 1.5 * 0.05 + 0.015));
  }
  // -- budget capped and spent on accept ------------------------------------
  {
    ProjectionGateState g(1.5, 0.05);
    g.step(0.0, 0.0, 0.0, 0.0);
    double x = 0.0;
    for (int i = 0; i < 200; ++i) {
      x += 0.04;
      assert(!g.step(x, 0.0, 99.0, 0.0).accepted);   // always over the limit
    }
    assert(g.budget <= 0.30 + 1e-12);
    assert(near_zero(g.accepted_arc, 0.0));
    GateDecision d = g.step(x, 0.0, 0.10, 0.0);       // within margin+budget
    assert(d.accepted);
    assert(near_zero(g.budget, 0.0));
  }
  // -- baseline is forward only; backward drift never lowers it -------------
  {
    ProjectionGateState g(1.5, 0.05);
    g.step(0.0, 0.0, 0.0, 0.0);
    double x = 0.0;
    for (int i = 0; i < 30; ++i) {
      x += 0.04;
      assert(g.step(x, 0.0, x, 0.0).accepted);
    }
    const double peak = g.accepted_arc;
    assert(g.step(x, 0.0, peak - 0.5, 0.0).accepted);  // backward is not a jump
    assert(near_zero(g.accepted_arc, peak));
  }
  // -- rebaseline is forward only and spends the budget ---------------------
  {
    ProjectionGateState g(1.5, 0.05);
    g.step(0.0, 0.0, 0.0, 0.0);
    assert(!g.step(0.04, 0.0, 99.0, 0.0).accepted);    // rejected, budget banked
    assert(g.budget > 0.0);
    g.rebaseline(2.0);
    assert(near_zero(g.accepted_arc, 2.0));
    assert(near_zero(g.budget, 0.0));
    g.rebaseline(1.0);                                  // backward: no move
    assert(near_zero(g.accepted_arc, 2.0));
  }
  std::printf("all ProjectionGateState tests passed\n");
  return 0;
}
