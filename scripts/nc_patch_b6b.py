#!/usr/bin/env python3
"""B6B negative control: deterministically disable the plugin's TERMINAL-STOP
clamp, rebuild, and (on revert) restore the exact original source and rebuild.

Usage:
  nc_patch_b6b.py apply   # patch + print what changed; caller rebuilds
  nc_patch_b6b.py revert  # restore exact original; caller rebuilds
  nc_patch_b6b.py status  # print patch state

Only the NORMAL-branch terminal clamp is disabled (the site that matters for
the drive-past-the-end overshoot).  The reacquire/probation branch clamp is
left intact -- the NC scenario never enters seeking/probation, so the single
changed variable is exactly the terminal stop on the accepted arc.

State lives under /tmp (never inside the repo): a backup of the pristine
source and an "active" marker, so a crashed run can always be reverted.
"""
import argparse
import os
import sys

CPP = "/home/zhouyi/ros2_ws/src/linear_mpc_controller/ros2/nav2_mpc_controller.cpp"
BAK = "/tmp/nc_nav2_mpc_controller.cpp.bak"
MARK = "/tmp/nc_patch_active"
ANCHOR = "// ---- B6B.1: two independent speed limits, take the min"
OLD = "  v = std::clamp(v, -v_term, v_term);"
NEW = ("  v = std::clamp(v, -v_allow, v_allow);"
       "  // NC: terminal stop DISABLED (negative control)")


def read(p):
    with open(p, encoding="utf-8") as f:
        return f.read()


def write(p, s):
    with open(p, "w", encoding="utf-8") as f:
        f.write(s)


def status():
    active = os.path.exists(MARK)
    backed = os.path.exists(BAK)
    print(f"nc_patch state: active={active} backup_present={backed}")
    return 0 if not active else 1


def apply():
    if os.path.exists(MARK):
        print("ERROR: patch already active; revert first")
        return 1
    src = read(CPP)
    idx = src.find(ANCHOR)
    if idx < 0:
        print("ERROR: normal-branch anchor not found; source moved?")
        return 1
    tail = src[idx:]
    n = tail.count(OLD)
    if n != 1:
        print(f"ERROR: expected exactly 1 terminal-clamp site after anchor, "
              f"found {n}")
        return 1
    # backup pristine source exactly once
    if not os.path.exists(BAK):
        write(BAK, src)
    patched = src[:idx] + tail.replace(OLD, NEW, 1)
    write(CPP, patched)
    write(MARK, "1")
    print("nc_patch applied: terminal clamp -> plain v_allow clamp "
          "(1 site, anchor-unique)")
    return 0


def revert():
    if not os.path.exists(BAK):
        print("ERROR: no backup to restore from")
        return 1
    pristine = read(BAK)
    write(CPP, pristine)
    for p in (MARK,):
        if os.path.exists(p):
            os.remove(p)
    print("nc_patch reverted: source restored to pristine backup")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["apply", "revert", "status"])
    args = ap.parse_args()
    rc = {"apply": apply, "revert": revert, "status": status}[args.cmd]()
    sys.exit(rc)


if __name__ == "__main__":
    main()
