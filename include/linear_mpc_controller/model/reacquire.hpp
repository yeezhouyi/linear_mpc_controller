// A5B / A4.2 C++ mirror of the Python reacquire protocol in mpc_core/mpc.py.
//
// The protocol is the safe answer to "the anchor went stale": instead of
// snapping to the global nearest point (which can re-attach to the wrong lane
// on folded/antiparallel paths) the controller decelerates, waits for a
// candidate that survives FOUR screens, commits atomically, then serves a
// bounded probation window at reduced speed.
//
//   NORMAL --(reject_run > max_reject_run)--> SEEKING --(stable >= 3 beats)-->
//   [atomic commit] --> PROBATION --(probation_steps)--> NORMAL
//   SEEKING --(> reacquire_timeout_steps)--> PROJECTION_LOST (hard stop)
//
// Screens applied to every seeking candidate (mirrors mpc.py lines 112-147):
//   1. distance gate      -- inside closestPointWindowed (reacquire_m)
//   2. heading compatible -- inside closestPointWindowed (heading_gate_rad)
//   3. ambiguity rejected -- stage == 3 resets the stability run
//   4. same segment for reacquire_stable_steps consecutive beats
//
// Constants are numeric copies of MpcParams (types.py:243-249).
#ifndef LINEAR_MPC_CONTROLLER__MODEL__REACQUIRE_HPP_
#define LINEAR_MPC_CONTROLLER__MODEL__REACQUIRE_HPP_

#include <cstdint>

namespace linear_mpc_controller
{

enum class ReacquireMode
{
  kNormal = 0,
  kSeeking = 1,
  kProbation = 2
};

struct ReacquireParams
{
  int max_reject_run = 5;             // consecutive rejects -> enter seeking
  double v_probation = 0.15;          // speed cap while on probation
  int probation_steps = 20;           // observation window after commit
  int reacquire_stable_steps = 3;     // consecutive same-seg beats to commit
  int reacquire_timeout_steps = 40;   // seeking timeout -> PROJECTION_LOST
};

/// Outcome of one cycle of the protocol, as seen by the caller.
struct ReacquireCycle
{
  bool seeking = false;        // true => emit the decel-to-zero command
  bool lost = false;           // seeking timed out -> PROJECTION_LOST
  bool committed = false;      // atomic commit happened on THIS cycle
  bool in_probation = false;   // probation active: speed cap, no completion
  int reacquire_count = 0;     // lifetime number of reacquire events
  double committed_arc = 0.0;  // new baseline when committed
};

struct ReacquireState
{
  ReacquireParams p;
  ReacquireMode mode = ReacquireMode::kNormal;
  int seeking_steps = 0;
  int stable_run = 0;
  int stable_seg = -1;
  int probation_left = 0;
  int reacquire_events = 0;

  void reset()
  {
    mode = ReacquireMode::kNormal;
    seeking_steps = 0;
    stable_run = 0;
    stable_seg = -1;
    probation_left = 0;
    // NOTE: reacquire_events is deliberately NOT reset -- "this run always
    // carries the mark" (A4.2 step 7); only setReference() clears it.
  }

  void resetAll()
  {
    reset();
    reacquire_events = 0;
  }

  bool inSeeking() const { return mode == ReacquireMode::kSeeking; }
  bool inProbation() const { return mode == ReacquireMode::kProbation; }

  /// Stop trusting the stale anchor (A4.2 entry condition).
  void enterSeeking()
  {
    mode = ReacquireMode::kSeeking;
    seeking_steps = 0;
    stable_run = 0;
    stable_seg = -1;
  }

  /// One SEEKING cycle.  ``seg/arc/stage`` come from a GLOBAL search with the
  /// heading gate ON (closestPointWindowed with s_prev < 0), which is exactly
  /// what mpc.py does while seeking.
  ReacquireCycle seek(int seg, double arc, int stage)
  {
    ReacquireCycle c;
    ++seeking_steps;
    if (seeking_steps > p.reacquire_timeout_steps) {
      c.seeking = true;
      c.lost = true;
      c.in_probation = true;   // Python marks in_probation on the lost path
      c.reacquire_count = reacquire_events;
      return c;
    }
    if (stage == 3) {
      stable_run = 0;
      stable_seg = -1;
    } else {
      if (seg == stable_seg) {
        ++stable_run;
      } else {
        stable_run = 1;
        stable_seg = seg;
      }
      if (stable_run >= p.reacquire_stable_steps) {
        // atomic commit (A4.2 steps 4/5): caller replaces the baseline with
        // committed_arc, zeroes the gate budget and drops the QP warm start.
        c.committed = true;
        c.committed_arc = arc;
        mode = ReacquireMode::kProbation;
        probation_left = p.probation_steps;
        ++reacquire_events;
      }
    }
    c.seeking = true;
    c.in_probation = (mode == ReacquireMode::kProbation);
    c.reacquire_count = reacquire_events;
    return c;
  }

  /// One PROBATION cycle.  Returns true while probation is still active; the
  /// completion predicate must return false for as long as this is true.
  bool tickProbation()
  {
    if (mode != ReacquireMode::kProbation) {
      return false;
    }
    --probation_left;
    if (probation_left <= 0) {
      mode = ReacquireMode::kNormal;
      return false;
    }
    return true;
  }
};

}  // namespace linear_mpc_controller

#endif  // LINEAR_MPC_CONTROLLER__MODEL__REACQUIRE_HPP_
