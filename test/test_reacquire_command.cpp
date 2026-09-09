// A4.2 SEEKING / PROBATION output shaping unit tests (the node branch that
// used to live inline in computeVelocityCommands).  Claims under test:
//   * SEEKING commands are decel-to-zero (v_cmd == 0 from the core),
//     never inflated by the external speed limit;
//   * PROBATION commands are hard-capped to v_probation (0.15 m/s default),
//     with the external speed limit applied as an additional cap;
//   * omega is clamped to omega_max;
//   * the terminal stop on the ACCEPTED arc can bind below all other caps.
#include <cmath>
#include <cstdio>

#include "linear_mpc_controller/safety/reacquire_command.hpp"

using linear_mpc_controller::MpcCycleResult;
using linear_mpc_controller::ReacquireCommandParams;
using linear_mpc_controller::shapeReacquireCommand;

static int failures = 0;

static void check(bool ok, const char * what)
{
  if (!ok) {
    std::printf("FAIL %s\n", what);
    ++failures;
  }
}

static bool near(double a, double b, double tol = 1e-9)
{
  return std::fabs(a - b) < tol;
}

// Default params mirror the node: params_ defaults + terminal_stop_margin_.
static ReacquireCommandParams defaultParams()
{
  ReacquireCommandParams p;
  p.v_max = 1.5;
  p.omega_max = 2.0;
  p.a_max = 1.0;
  p.v_probation = 0.15;
  p.terminal_stop_margin = 0.20;
  return p;
}

int main()
{
  // -- 1) SEEKING: zero command, unchanged by the speed limit ---------------
  {
    MpcCycleResult res;
    res.reacquire_seeking = true;   // v_cmd/omega_cmd stay at the defaults
    res.accepted_arc = 4.0;
    ReacquireCommandParams p = defaultParams();
    double v = 99.0, w = 99.0;
    shapeReacquireCommand(res, p, 8.0, v, w);
    check(v == 0.0 && w == 0.0, "SEEKING must shape to a zero command");
    p.speed_limit_active = true;    // a limit can never raise the zero
    p.speed_limit = 0.5;
    shapeReacquireCommand(res, p, 8.0, v, w);
    check(v == 0.0 && w == 0.0, "SEEKING stays zero under a speed limit");
  }

  // -- 2) PROBATION: hard cap to v_probation --------------------------------
  {
    MpcCycleResult res;
    res.in_probation = true;
    res.v_cmd = 0.5;                 // the tracker wants full speed
    res.omega_cmd = 0.0;
    res.accepted_arc = 4.0;
    double v = 0.0, w = 0.0;
    shapeReacquireCommand(res, defaultParams(), 8.0, v, w);
    check(near(v, 0.15), "PROBATION must cap v_cmd to v_probation");
    check(w == 0.0, "PROBATION keeps the commanded omega");
  }

  // -- 3) PROBATION below the cap passes through ----------------------------
  {
    MpcCycleResult res;
    res.in_probation = true;
    res.v_cmd = 0.1;
    res.accepted_arc = 4.0;
    double v = 0.0, w = 0.0;
    shapeReacquireCommand(res, defaultParams(), 8.0, v, w);
    check(near(v, 0.1), "PROBATION passes through a command below the cap");
  }

  // -- 4) omega clamp to omega_max ------------------------------------------
  {
    MpcCycleResult res;
    res.in_probation = true;
    res.v_cmd = 0.1;
    res.omega_cmd = 5.0;
    res.accepted_arc = 4.0;
    double v = 0.0, w = 0.0;
    shapeReacquireCommand(res, defaultParams(), 8.0, v, w);
    check(near(w, 2.0), "omega must be clamped to omega_max");
    check(near(v, 0.1), "omega clamp must not disturb the v cap");
  }

  // -- 5) absolute speed limit caps SEEKING output --------------------------
  {
    MpcCycleResult res;
    res.reacquire_seeking = true;
    res.v_cmd = 0.6;                 // not really reachable, but the shaper
    res.accepted_arc = 4.0;          // must still bound it like the node did
    ReacquireCommandParams p = defaultParams();
    p.speed_limit_active = true;
    p.speed_limit = 0.5;             // absolute m/s limit
    double v = 0.0, w = 0.0;
    shapeReacquireCommand(res, p, 8.0, v, w);
    check(near(v, 0.5), "absolute speed limit caps a seeking command");
    p.speed_limit_percentage = true;
    p.speed_limit = 50.0;            // 50% of v_max = 0.75
    shapeReacquireCommand(res, p, 8.0, v, w);
    check(near(v, 0.6), "percentage limit (50%% of 1.5) allows 0.6");
  }

  // -- 6) terminal stop binds on the accepted arc ---------------------------
  {
    MpcCycleResult res;
    res.in_probation = true;
    res.v_cmd = 0.5;
    res.accepted_arc = 7.9;          // 0.1 m before the end of an 8 m path
    double v = 0.0, w = 0.0;
    shapeReacquireCommand(res, defaultParams(), 8.0, v, w);
    check(v == 0.0, "terminal stop must clamp to zero at the very end");
    res.accepted_arc = 7.79;         // remaining = 8 - 7.79 - 0.20 = 0.01:
    shapeReacquireCommand(res, defaultParams(), 8.0, v, w);   // clamp < 0.15
    const double v_term = std::sqrt(2.0 * 1.0 * 0.01);
    check(near(v, v_term, 1e-12),
      "terminal stop must bind to sqrt(2*a*remaining) below the probation cap");
  }

  // -- 7) symmetric negative clamp ------------------------------------------
  {
    MpcCycleResult res;
    res.in_probation = true;
    res.v_cmd = -0.5;
    res.omega_cmd = 0.0;
    res.accepted_arc = 4.0;
    double v = 0.0, w = 0.0;
    shapeReacquireCommand(res, defaultParams(), 8.0, v, w);
    check(near(v, -0.15), "PROBATION caps negative commands symmetrically");
  }

  if (failures) {
    std::printf("%d reacquire-command check(s) failed\n", failures);
    return 1;
  }
  std::printf("all A4.2 reacquire-command shaping tests passed\n");
  return 0;
}
