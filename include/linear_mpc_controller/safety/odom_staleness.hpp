// Odom freshness gate for the standalone /odom consumer (R9).
//
// Pure (ROS-free) decision helper so the node logic can be unit-tested
// without a live clock: interruption / frozen stamp / time-jump-backward
// are all reduced to (age, since_recv, backwards).
#ifndef LINEAR_MPC_CONTROLLER__SAFETY__ODOM_STALENESS_HPP_
#define LINEAR_MPC_CONTROLLER__SAFETY__ODOM_STALENESS_HPP_

namespace linear_mpc_controller
{

/// A stamp slightly in the future is clock jitter; anything beyond is
/// treated as a jump backward (stale) -- do not command on a broken clock.
constexpr double kOdomClockTolS = 0.05;

/// Decide whether the last odometry state is too old to command on.
///
/// \param age_s       now - odom header.stamp (or now - receipt when the
///                    driver leaves the stamp at zero)
/// \param max_age_s   odom_max_age_s parameter
/// \param stamp_valid false when the last message carried a zero stamp
/// \param backwards   true when the latest stamp went BACKWARD vs previous
/// \param since_recv_s now - last odometry receipt time (always tracked)
inline bool odomIsStale(
  double age_s, double max_age_s, bool stamp_valid, bool backwards,
  double since_recv_s)
{
  if (backwards) {
    return true;                    // time jump backward
  }
  if (!stamp_valid) {
    return since_recv_s > max_age_s;  // unstamped driver: receipt gap
  }
  // Frozen stamps: now() advances while age_s grows past max_age_s.
  // Future stamps beyond jitter tolerance: clock jumped / restarted.
  return age_s > max_age_s || age_s < -kOdomClockTolS;
}

}  // namespace linear_mpc_controller

#endif  // LINEAR_MPC_CONTROLLER__SAFETY__ODOM_STALENESS_HPP_
