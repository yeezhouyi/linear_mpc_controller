// Pure unit tests for the R9 odom freshness gate (review fix 1).
// Interruption / frozen stamp / time-jump-backward are all covered here at
// the decision level; the node wires these inputs in linear_mpc_node.cpp.
#include <cstdio>

#include "linear_mpc_controller/safety/odom_staleness.hpp"

using namespace linear_mpc_controller;

int main()
{
  int fails = 0;
  auto chk = [&fails](bool cond, const char * name) {
    if (!cond) {
      std::printf("FAIL %s\n", name);
      ++fails;
    }
  };

  // fresh stamped odom -> command allowed
  chk(!odomIsStale(0.02, 0.5, true, false, 0.02), "fresh-stamped");
  // interruption: stamped odom stops advancing (age grows) -> stale
  chk(odomIsStale(0.70, 0.5, true, false, 0.05), "stamped-stale-old");
  // frozen: messages keep arriving but the stamp does not advance -> the
  // receipt gap stays small, the STAMP age is what trips
  chk(odomIsStale(0.60, 0.5, true, false, 0.02), "frozen-stamp");
  // unstamped driver: falls back to the receipt gap
  chk(odomIsStale(0.70, 0.5, false, false, 0.70), "unstamped-stale");
  chk(!odomIsStale(0.05, 0.5, false, false, 0.05), "unstamped-fresh");
  // time jump backward (flag set by monotonic check in the callback)
  chk(odomIsStale(0.02, 0.5, true, true, 0.02), "backwards-stamp");
  // future stamp beyond jitter tolerance -> stale
  chk(odomIsStale(-2.0, 0.5, true, false, 0.02), "future-jump");
  // small future jitter is tolerated
  chk(!odomIsStale(-0.01, 0.5, true, false, 0.02), "clock-jitter-ok");

  if (fails) {
    std::printf("odom_staleness: %d FAILURES\n", fails);
    return 1;
  }
  std::printf("odom_staleness: OK\n");
  return 0;
}
