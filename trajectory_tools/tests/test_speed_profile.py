# --- Advanced round close-out: speed profile + production entry -----------

import numpy as np
import pytest

from trajectory_tools.adapter_pipeline import prepare_tracker_reference
from trajectory_tools.speed_profile import (
    accel_limited_profile,
    speed_profile_violations,
)


def _straight_s(n: int, ds: float = 0.05) -> np.ndarray:
    s = np.arange(n, dtype=float) * ds
    return s


def test_terminal_brake_brings_cruise_to_v_end_at_path_end():
    s = _straight_s(121)                     # 6.0 m at ds=0.05
    v_curve = np.full(121, 0.3)
    vp = accel_limited_profile(v_curve, s, a_accel=0.5, a_decel=0.5,
                               v_end=0.0)
    assert vp[-1] <= 1e-9                     # stops exactly at the end
    # Interior stays at cruise until the brake distance (v^2/2a = 0.09 m).
    brake_start = int(np.nonzero(vp < 0.3 - 1e-9)[0][0])
    assert s[brake_start] > 5.7                # braking only near the end
    assert np.all(vp <= v_curve + 1e-12)       # never raised above curve
    verdict = speed_profile_violations(vp, s, a_accel=0.5, a_decel=0.5,
                                       v_end=0.0)
    assert verdict["feasible"], verdict["reasons"]


def test_forward_ramp_limits_step_acceleration():
    s = _straight_s(101)                       # 5.0 m
    v_curve = np.zeros(101)
    v_curve[1:] = 0.5                          # absurd step at sample 0->1
    vp = accel_limited_profile(v_curve, s, a_accel=0.5, a_decel=0.5)
    # sample 1 must respect v1^2 <= v0^2 + 2 a ds -> sqrt(2*0.5*0.05)=0.224
    assert vp[1] <= 0.224 + 1e-9
    # ... and the full profile passes its own discrete checks.
    verdict = speed_profile_violations(vp, s, a_accel=0.5, a_decel=0.5)
    assert verdict["feasible"], verdict["reasons"]


def test_brake_limiting_holds_for_short_decel_distance():
    # v_end=0 with a tiny decel budget: the whole profile must collapse so
    # that every point can still brake to 0.
    s = _straight_s(41)                        # 2.0 m
    v_curve = np.full(41, 0.5)
    vp = accel_limited_profile(v_curve, s, a_accel=0.5, a_decel=0.2,
                               v_end=0.0)
    # max entry speed under 0.2 m/s^2 over 2.0 m: sqrt(2*0.2*2.0)=0.894>0.5,
    # so the start may stay at 0.5; the brake zone is v^2/(2a)=0.625 m.
    assert vp[0] == pytest.approx(0.5)
    assert np.all(vp <= v_curve + 1e-12)
    verdict = speed_profile_violations(vp, s, a_accel=0.5, a_decel=0.2,
                                       v_end=0.0)
    assert verdict["feasible"], verdict["reasons"]


def test_omega_cap_preserved_after_profiling():
    # v_curve already satisfies |kappa|*v <= omega_max; profiling only
    # lowers speeds, so the omega bound must still hold.
    s = _straight_s(101)
    kappa = np.full(101, 0.0)
    kappa[40:60] = 3.0                        # needs v <= 2.0/3.0 = 0.667
    v_curve = np.full(101, 0.5)               # legal: 3.0*0.5 = 1.5 <= 2.0
    vp = accel_limited_profile(v_curve, s, a_accel=0.5, a_decel=0.5,
                               v_end=0.0)
    assert np.max(np.abs(kappa) * vp) <= 2.0 * (1 + 1e-9)


def test_violation_checker_flags_step_and_over_brake():
    s = _straight_s(11)                        # 0.5 m
    # accel bound: sample 1 jumps 0 -> 0.5 over ds=0.05 (needs a=2.5)
    v_bad = np.array([0.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
    v1 = speed_profile_violations(v_bad, s, a_accel=0.5, a_decel=0.5)
    assert not v1["feasible"]
    assert "accel_bound" in v1["codes"]
    # brake bound: late drop 0.5 -> 0 over one ds is a 2.5 m/s^2 decel
    v_bad2 = v_bad.copy()
    v_bad2[-2] = 0.5
    v_bad2[-1] = 0.0
    v2 = speed_profile_violations(v_bad2, s, a_accel=0.5, a_decel=0.5)
    assert not v2["feasible"]
    assert "brake_bound" in v2["codes"]
    # terminal speed: last sample above v_end
    v3 = speed_profile_violations(np.full(11, 0.3), s, a_accel=0.5,
                                  a_decel=0.5, v_end=0.0)
    assert not v3["feasible"]
    assert "terminal_speed" in v3["codes"]
    # nonfinite
    v4 = np.full(11, 0.3)
    v4[5] = np.nan
    vn = speed_profile_violations(v4, s, a_accel=0.5, a_decel=0.5)
    assert not vn["feasible"]
    assert "nonfinite" in vn["codes"]


def _straight_xy(n: int, length: float) -> tuple:
    return np.linspace(0.0, length, n), np.zeros(n)


def test_prepare_tracker_reference_certifies_clean_and_brakes():
    x, y = _straight_xy(21, 2.0)
    traj, verdict = prepare_tracker_reference(x, y, v_default=0.3,
                                              v_max=0.5, ds=0.05)
    assert verdict["feasible"], verdict["reasons"]
    assert traj.v[-1] <= 1e-9                  # terminal deceleration on
    # processed reference satisfies the discrete constraint checks
    prof = speed_profile_violations(traj.v, traj.s, a_accel=0.5,
                                    a_decel=0.5, v_end=0.0)
    assert prof["feasible"], prof["reasons"]
    omega = np.abs(traj.kappa) * traj.v
    assert omega.max() <= 2.0 * (1 + 1e-9)


def test_prepare_tracker_reference_rejects_nonfinite_input():
    x = np.array([0.0, 1.0, 2.0, np.nan, 4.0])
    y = np.zeros(5)
    # NaN waypoint: resample_uniform returns the raw polyline unchanged
    # (total arc is non-finite), curvature/speed then carry NaN and the
    # production entry's certify gate MUST surface an infeasible verdict
    # with code=nonfinite instead of handing garbage onward.
    _, verdict = prepare_tracker_reference(x, y, v_default=0.3, ds=0.05)
    assert not verdict["feasible"]
    assert "nonfinite" in verdict["codes"]


def test_prepare_tracker_reference_repairs_duplicate_tail_platform():
    # Colocated tail (0 -> 1, then three points parked at x=1): arc-length
    # resample CONSUMES the flat plateau (interp re-parameterises the same
    # polyline), so the pipeline REPAIRS it -- "fixable gets fixed".  The
    # repaired reference ends at x=1 with a terminal brake to v_end=0.
    x = np.array([0.0, 1.0, 1.0, 1.0])
    y = np.zeros(4)
    traj, verdict = prepare_tracker_reference(x, y, v_default=0.3, ds=0.05)
    assert verdict["feasible"], verdict["reasons"]
    assert traj.x[-1] == pytest.approx(1.0)
    assert traj.v[-1] <= 1e-9
    # Raw-waypoint hygiene stays a SEPARATE gate for task-layer callers who
    # certify before any resampling (validate_input_path): a duplicate
    # segment there is refused with zero_length_segment, not silently kept.
    from trajectory_tools.reference_guard import certify_reference
    c = certify_reference(x, np.zeros(4), np.zeros(4), np.full(4, 0.3),
                          np.arange(4.0))
    assert not c["feasible"]
    assert "zero_length_segment" in c["codes"]
