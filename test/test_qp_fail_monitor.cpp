// B6B.2 QP-failure monitor unit tests.
//
// The node aborts (clears the plan + throws NoValidControl) only after a
// run of consecutive NORMAL-tracking kFailed cycles exceeds qp_fail_max.
// The central safety claim under test: A4.2 SEEKING / PROBATION cycles are
// NEVER accountable -- SEEKING runs no QP (its cycles carry the default
// kFailed status) and would otherwise clear the controller after a few
// beats, and PROBATION is a bounded observation window that never throws.
#include <cmath>
#include <cstdio>

#include "linear_mpc_controller/safety/qp_fail_monitor.hpp"

using linear_mpc_controller::MpcCycleResult;
using linear_mpc_controller::QpFailMonitor;
using linear_mpc_controller::QpSolution;
using linear_mpc_controller::qpFailureAccountable;

static int failures = 0;

static void check(bool ok, const char * what)
{
  if (!ok) {
    std::printf("FAIL %s\n", what);
    ++failures;
  }
}

static MpcCycleResult makeRes(QpSolution::Status st)
{
  MpcCycleResult r;
  r.qp_status = st;
  return r;
}

int main()
{
  // -- 1) fresh monitor: run 0, not aborted --------------------------------
  {
    QpFailMonitor m(3);
    check(m.run() == 0 && !m.aborted(), "fresh monitor has an empty run");
    check(m.maxFails() == 3, "max_fails is readable");
    m.setMaxFails(5);
    check(m.maxFails() == 5, "setMaxFails updates the threshold");
  }

  // -- 2) consecutive kFailed accountable cycles -> abort on (max+1)-th -----
  {
    QpFailMonitor m(3);
    bool tripped = false;
    for (int i = 1; i <= 4; ++i) {
      const bool t = m.record(makeRes(QpSolution::Status::kFailed));
      if (i < 4) {
        check(!t, "run below the threshold must not trip");
      }
      tripped = tripped || t;
    }
    check(tripped, "the 4th consecutive failure (max=3) must trip");
    check(m.run() == 4, "run counts every accountable failure");
    check(m.aborted(), "aborted() reflects run > max");
    m.reset();
    check(m.run() == 0 && !m.aborted(), "reset clears the run");
  }

  // -- 3) recovery credit: a later kSolved walks the run back down ----------
  {
    QpFailMonitor m(3);
    m.record(makeRes(QpSolution::Status::kFailed));    // run 1
    m.record(makeRes(QpSolution::Status::kFailed));    // run 2
    check(m.run() == 2, "two failures accumulate");
    m.record(makeRes(QpSolution::Status::kSolved));    // recovery -> run 1
    check(m.run() == 1, "a clean solve grants one step of recovery");
    m.record(makeRes(QpSolution::Status::kSolved));    // recovery -> run 0
    check(m.run() == 0, "second clean solve clears the run");
    m.record(makeRes(QpSolution::Status::kSolved));    // run already 0
    check(m.run() == 0, "recovery credit is capped at zero");
    check(!m.aborted(), "recovered run must not abort");
  }

  // -- 4) kApproximate neither increments nor decrements --------------------
  {
    QpFailMonitor m(3);
    m.record(makeRes(QpSolution::Status::kFailed));    // run 1
    m.record(makeRes(QpSolution::Status::kApproximate));
    check(m.run() == 1, "kApproximate must not add to the run");
    m.record(makeRes(QpSolution::Status::kSolved));
    check(m.run() == 0, "kSolved recovery still works after kApproximate");
  }

  // -- 5) SEEKING cycles carry the default kFailed but are NOT accountable --
  {
    QpFailMonitor m(3);
    MpcCycleResult seek = makeRes(QpSolution::Status::kFailed);
    seek.reacquire_seeking = true;   // the core returns before the QP runs
    check(!qpFailureAccountable(seek), "SEEKING cycles must be excluded");
    bool tripped = false;
    for (int i = 0; i < 20; ++i) {   // a whole seeking window (40 max)
      tripped = tripped || m.record(seek);
    }
    check(!tripped && m.run() == 0 && !m.aborted(),
      "20 kFailed SEEKING cycles must never trip the monitor");
  }

  // -- 6) PROBATION cycles are not accountable either -----------------------
  {
    QpFailMonitor m(3);
    MpcCycleResult prob = makeRes(QpSolution::Status::kFailed);
    prob.in_probation = true;
    check(!qpFailureAccountable(prob), "PROBATION cycles must be excluded");
    bool tripped = false;
    for (int i = 0; i < 20; ++i) {
      tripped = tripped || m.record(prob);
    }
    check(!tripped && m.run() == 0,
      "failing PROBATION cycles must never trip the monitor");
    // a probation kSolved must not grant recovery credit to a normal run
    m.record(makeRes(QpSolution::Status::kFailed));    // run 1
    m.record(makeRes(QpSolution::Status::kFailed));    // run 2
    m.record(prob);                                    // ignored
    check(m.run() == 2, "PROBATION cycles must not change a normal run");
  }

  // -- 7) mixed sequence mirrors the node end-to-end -------------------------
  {
    QpFailMonitor m(3);
    MpcCycleResult seek = makeRes(QpSolution::Status::kFailed);
    seek.reacquire_seeking = true;
    MpcCycleResult prob = makeRes(QpSolution::Status::kFailed);
    prob.in_probation = true;
    m.record(makeRes(QpSolution::Status::kFailed));    // run 1
    m.record(makeRes(QpSolution::Status::kFailed));    // run 2
    m.record(makeRes(QpSolution::Status::kFailed));    // run 3 (not tripped)
    check(!m.aborted(), "run 3 of max 3 must not abort yet");
    m.record(seek);                                    // ignored
    m.record(prob);                                    // ignored
    check(m.run() == 3, "seeking/probation beats in between are ignored");
    m.record(makeRes(QpSolution::Status::kSolved));    // run 2
    m.record(makeRes(QpSolution::Status::kFailed));    // run 3
    m.record(makeRes(QpSolution::Status::kFailed));    // run 4 -> tripped
    check(m.aborted(), "persistent failure resumes and trips on the 4th");
  }

  if (failures) {
    std::printf("%d QP-fail monitor check(s) failed\n", failures);
    return 1;
  }
  std::printf("all QP-fail monitor policy tests passed\n");
  return 0;
}
