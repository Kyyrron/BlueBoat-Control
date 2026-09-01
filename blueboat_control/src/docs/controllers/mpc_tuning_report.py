#!/usr/bin/env python3
"""Score a recorded controller run for actuator aggression.

Turns "the MPC feels too aggressive" into numbers. Reads the .npy controller log
written by master_control (schema ['t','x','y','psi','x_d','y_d','psi_d','u1','u2'],
header in row 0, whole array coerced to strings by np.save) and reports:

  * how fast the reference was actually moving, and how fast the boat went
  * how much of the run sat on the +/-thrust_limit bound
  * along-track and cross-track error, separately -- a boat abreast of its target
    but far off to the side is failing differently from one that is behind
  * differential sign flips, which is what "chattering thrusters" looks like
    numerically

Read-only: recorded runs are primary record (CLAUDE.md section 6 / CM-7).

    python3 mpc_tuning_report.py <log.npy> [more.npy ...]
    python3 mpc_tuning_report.py ~/ros2_ws/data/MPC_data/*.npy
    python3 mpc_tuning_report.py --latest MPC        # newest run of that controller
"""

import glob
import os
import sys

import numpy as np

THR_LIM = 20.0            # master_control's thrust_limit default
SAT = 0.995 * THR_LIM     # "on the bound" allowing for solver tolerance
COLUMNS = ['t', 'x', 'y', 'psi', 'x_d', 'y_d', 'psi_d', 'u1', 'u2']


def load(path):
    """Return the numeric rows, dropping the string header row."""
    a = np.load(path, allow_pickle=True)
    if a.ndim != 2 or a.shape[1] != len(COLUMNS):
        raise ValueError(f'{path}: expected {len(COLUMNS)} columns, got {a.shape}')
    # The header is appended to the same list as the float rows, so np.save
    # coerced everything to strings -- cast back and drop row 0.
    return a[1:].astype(float)


def report(path):
    d = load(path)
    if len(d) < 20:
        print(f'{os.path.basename(path)}: only {len(d)} rows, skipping')
        return
    t, x, y, psi, xd, yd, psid, u1, u2 = d.T
    U = np.c_[u1, u2]
    diff = u1 - u2

    dt = np.diff(t)
    ok = dt > 1e-3                      # guard the finite differences
    v_ref = np.hypot(np.diff(xd), np.diff(yd))[ok] / dt[ok]
    v_boat = np.hypot(np.diff(x), np.diff(y))[ok] / dt[ok]

    ex, ey = x - xd, y - yd
    # Signed along/cross-track split about the reference heading. along > 0 means
    # the boat is BEHIND its target, which is the sign the governor throttles on.
    along = -(np.cos(psid) * ex + np.sin(psid) * ey)
    cross = -np.sin(psid) * ex + np.cos(psid) * ey
    radial = np.hypot(ex, ey)

    duration = t[-1] - t[0]
    any_sat = (np.abs(U) >= SAT).any(axis=1)
    both_sat = (np.abs(U) >= SAT).all(axis=1)
    flips = 100 * np.mean(np.diff(np.sign(diff)) != 0)

    print(f'== {os.path.basename(path)}')
    print(f'   {len(d)} ticks over {duration:.0f} s '
          f'({len(d) / duration:.1f} Hz, 20 Hz nominal)')
    print(f'   speed      ref  p50 {np.median(v_ref):.2f}  p90 {np.percentile(v_ref, 90):.2f} m/s'
          f'   |  boat p50 {np.median(v_boat):.2f}  p90 {np.percentile(v_boat, 90):.2f} m/s')
    print(f'   thrust     |u| mean {np.abs(U).mean():5.2f}  max {np.abs(U).max():5.2f} N'
          f'   |  surge mean {U.sum(1).mean():6.2f}  |diff| mean {np.abs(diff).mean():5.2f} N')
    print(f'   SATURATION one thruster {100 * any_sat.mean():5.1f} %'
          f'   |  both {100 * both_sat.mean():5.1f} %')
    print(f'   CHATTER    differential sign flips {flips:5.1f} per 100 ticks')
    print(f'   error      radial mean {radial.mean():5.2f} max {radial.max():6.2f} m'
          f'   |  along mean {along.mean():+6.2f}  cross mean {cross.mean():+6.2f} m')
    print(f'   governor   {100 * np.mean(along > 3.0):5.1f} % of ticks beyond gov_Lmax = 3.0 m')


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    if argv[0] == '--latest':
        ctrl = argv[1] if len(argv) > 1 else 'MPC'
        pattern = os.path.expanduser(f'~/ros2_ws/data/{ctrl}_data/*.npy')
        files = [f for f in sorted(glob.glob(pattern)) if os.path.getsize(f) > 0]
        if not files:
            print(f'no non-empty runs matching {pattern}')
            return 1
        paths = files[-1:]
    else:
        paths = [p for p in argv if os.path.getsize(p) > 0]

    for p in paths:
        try:
            report(p)
        except Exception as e:                       # a truncated run is not fatal
            print(f'{os.path.basename(p)}: {type(e).__name__}: {e}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
