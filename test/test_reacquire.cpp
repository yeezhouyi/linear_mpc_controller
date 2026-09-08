// A4.2 reacquire protocol tests (C++ side).
// Mirrors the SEEKING / COMMIT / PROBATION semantics of mpc_core/mpc.py and
// closes stage-A seal condition #5: heading-compatible, ambiguity-rejecting,
// probation-speed-limited, no-completion-on-commit-cycle.
#include <cmath>
#include <cstdio>

#include "linear_mpc_controller/model/reacquire.hpp"

using linear_mpc_controller::ReacquireCycle;
using linear_mpc_controller::ReacquireState;

static int failures = 0;

static void check(bool ok, const char * what)
{
  if (!ok) {
    std::printf("FAIL %s\n", what);
    ++failures;
  }
}

int main()
{
  // -- 1) entry: NORMAL -> SEEKING ------------------------------------------
  {
    ReacquireState s;
    check(!s.inSeeking(), "fresh state must not be seeking");
    s.enterSeeking();
    check(s.inSeeking(), "enterSeeking must arm SEEKING");
    check(s.seeking_steps == 0 && s.stable_run == 0 && s.stable_seg == -1,
      "enterSeeking must clear the stability run");
  }

  // -- 2) ambiguity screen: stage 3 resets the stability run -----------------
  {
    ReacquireState s;
    s.enterSeeking();
    s.seek(7, 3.5, 2);          // clean candidate on segment 7
    check(s.stable_run == 1, "first clean candidate counts as 1");
    ReacquireCycle amb = s.seek(9, 4.0, 3);   // ambiguous -> reset
    check(s.stable_run == 0 && s.stable_seg == -1, "stage 3 must reset stability");
    check(!amb.committed, "an ambiguous candidate must never commit");
  }

  // -- 3) four screens: 3 consecutive same-segment beats -> atomic commit ----
  {
    ReacquireState s;
    s.enterSeeking();
    ReacquireCycle c1 = s.seek(4, 2.00, 2);
    check(!c1.committed, "beat 1 must not commit");
    ReacquireCycle c2 = s.seek(4, 2.01, 2);
    check(!c2.committed, "beat 2 must not commit");
    ReacquireCycle c3 = s.seek(4, 2.02, 2);
    check(c3.committed, "beat 3 must commit (reacquire_stable_steps=3)");
    check(std::fabs(c3.committed_arc - 2.02) < 1e-12,
      "commit must carry THIS cycle's arc, not a cached candidate");
    check(s.inProbation(), "commit must enter PROBATION");
    check(s.probation_left == s.p.probation_steps,
      "probation window must be armed to probation_steps");
    check(s.reacquire_events == 1, "one commit == one reacquire event");
    // the commit cycle itself is still a decel-to-zero cycle (no completion)
    check(c3.seeking, "the commit cycle must not be a normal tracking cycle");
    check(c3.in_probation, "the commit cycle reports probation");
  }

  // -- 4) a segment change restarts the stability run ------------------------
  {
    ReacquireState s;
    s.enterSeeking();
    s.seek(4, 2.0, 2);
    s.seek(4, 2.0, 2);
    ReacquireCycle c = s.seek(5, 2.2, 2);   // different segment
    check(!c.committed, "a segment change must restart the stability run");
    check(s.stable_run == 1, "run restarts at 1 on the new segment");
  }

  // -- 5) seeking timeout -> PROJECTION_LOST ---------------------------------
  {
    ReacquireState s;
    s.enterSeeking();
    ReacquireCycle last;
    bool lost = false;
    for (int i = 0; i < s.p.reacquire_timeout_steps + 2; ++i) {
      last = s.seek(4, 2.0, 3);     // never stable (ambiguous every beat)
      if (last.lost) {
        lost = true;
        break;
      }
    }
    check(lost, "seeking past the timeout must report PROJECTION_LOST");
    check(last.in_probation, "the lost path keeps the degraded flag set");
  }

  // -- 6) probation window expires back to NORMAL ----------------------------
  {
    ReacquireState s;
    s.enterSeeking();
    s.seek(1, 1.0, 2);
    s.seek(1, 1.0, 2);
    s.seek(1, 1.0, 2);              // commit -> probation
    check(s.inProbation(), "committed state is on probation");
    for (int i = 0; i < s.p.probation_steps; ++i) {
      const bool active = s.tickProbation();
      if (i < s.p.probation_steps - 1) {
        check(active, "probation must stay active for the whole window");
      }
    }
    check(!s.inProbation(), "probation must expire after probation_steps");
    check(!s.tickProbation(), "tickProbation is a no-op outside probation");
  }

  // -- 7) reset drops an in-flight reacquire but keeps the event counter -----
  {
    ReacquireState s;
    s.enterSeeking();
    s.seek(1, 1.0, 2);
    s.seek(1, 1.0, 2);
    s.seek(1, 1.0, 2);
    check(s.reacquire_events == 1, "one event recorded");
    s.reset();
    check(!s.inSeeking() && !s.inProbation(), "reset must leave SEEKING/PROBATION");
    check(s.reacquire_events == 1, "A4.2 step 7: the run keeps the mark");
    s.resetAll();
    check(s.reacquire_events == 0, "resetAll clears the counter");
  }

  if (failures) {
    std::printf("%d A4.2 reacquire check(s) failed\n", failures);
    return 1;
  }
  std::printf("all A4.2 reacquire protocol tests passed\n");
  return 0;
}
