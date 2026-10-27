#!/usr/bin/env python3
"""
NFR-3: the loop-closure error of an estimate (threshold in the thesis: < 0.1 m).

Thesis wording: "the difference between the estimate at the return to a
visited point and the estimate there at the first pass". Implemented as:

  A REVISIT is a sample t2 whose TRUE position comes within REVISIT_RADIUS
  (0.10 m) of the true position at an earlier time t1 with t2 - t1 >= MIN_GAP
  (30 s; this excludes simply driving on). For t2, t1 is taken from the FIRST
  pass near that point (the earliest group of such earlier samples, split
  where consecutive candidates are more than MIN_GAP apart), at its sample
  closest to the true position at t2. The error at t2 is

      e(t2) = | (p_est(t2) - p_est(t1)) - (p_true(t2) - p_true(t1)) |

  i.e. how much the ESTIMATED displacement between the two visits differs from
  the TRUE one. A constant offset of the estimate cancels; only the drift
  accumulated between the first pass and the return counts. The true
  displacement (up to 0.1 m) is subtracted, so the robot need not hit the
  point exactly.

Reported per run: e at the FINAL return to the start (t2 = the last sample
of the run, t1 on the first pass at the start - every route of the series
ends where it starts), and the maximum and mean of e over all revisits.

Positions are resampled at SAMPLE_RATE (5 Hz) on the ground-truth times,
the estimate interpolated to them (no alignment, same frame from t = 0).

Library use (final_metrics.py):  loop_errors(true_txy, est_txy) -> dict
Command line, with TUM files (e.g. from tf_to_tum.py / evo_traj --save_as_tum):
  ~/.venvs/evo/bin/python src/gnssd_tools/scripts/loop_closure_error.py ground_truth.tum est.tum
"""

import argparse

import numpy as np
from scipy.spatial import cKDTree

REVISIT_RADIUS = 0.10   # m
MIN_GAP = 30.0          # s
SAMPLE_RATE = 5.0       # Hz


def resample(true_txy, est_txy):
    """Resample to common 5 Hz times inside both spans; return times, true and estimated (x, y)."""
    start = max(true_txy[0, 0], est_txy[0, 0])
    end = min(true_txy[-1, 0], est_txy[-1, 0])
    t = np.arange(start, end, 1.0 / SAMPLE_RATE)
    true = np.column_stack([np.interp(t, true_txy[:, 0], true_txy[:, k]) for k in (1, 2)])
    est = np.column_stack([np.interp(t, est_txy[:, 0], est_txy[:, k]) for k in (1, 2)])
    return t, true, est


def first_pass_partner(t, true, i2, candidates):
    """Index t1 on the first pass among the earlier candidates of sample i2."""
    candidates = np.sort(candidates)
    split = np.nonzero(np.diff(t[candidates]) > MIN_GAP)[0]
    first = candidates[:split[0] + 1] if len(split) else candidates
    distances = np.hypot(*(true[first] - true[i2]).T)
    return first[np.argmin(distances)]


def loop_errors(true_txy, est_txy):
    """
    Loop-closure errors of one estimate.

    true_txy, est_txy: arrays of rows (t, x, y). Returns a dict with
    e_final (None if the run does not end near its start), e_max, e_mean,
    revisits (number of revisit samples) and the arrays t2, e.
    """
    t, true, est = resample(true_txy, est_txy)
    tree = cKDTree(true)
    t2_list, e_list = [], []
    for i2, neighbours in enumerate(tree.query_ball_point(true, REVISIT_RADIUS)):
        earlier = [i1 for i1 in neighbours if t[i2] - t[i1] >= MIN_GAP]
        if not earlier:
            continue
        i1 = first_pass_partner(t, true, i2, np.array(earlier))
        error = (est[i2] - est[i1]) - (true[i2] - true[i1])
        t2_list.append(t[i2])
        e_list.append(float(np.hypot(*error)))
    e = np.array(e_list)
    t2 = np.array(t2_list)
    e_final = float(e[-1]) if len(e) and t2[-1] == t[-1] else None
    return {'e_final': e_final,
            'e_max': float(e.max()) if len(e) else None,
            'e_mean': float(e.mean()) if len(e) else None,
            'revisits': int(len(e)), 't2': t2, 'e': e}


def main():
    parser = argparse.ArgumentParser(description='NFR-3 loop-closure error from TUM files')
    parser.add_argument('ground_truth', help='TUM file of the true trajectory')
    parser.add_argument('estimate', help='TUM file of the estimate')
    args = parser.parse_args()
    true = np.loadtxt(args.ground_truth)[:, :3]
    est = np.loadtxt(args.estimate)[:, :3]
    result = loop_errors(true, est)
    final = 'n/a' if result['e_final'] is None else f'{result["e_final"]:.3f} m'
    print(f'revisit samples {result["revisits"]}: e at the final return {final}, '
          f'max {result["e_max"]:.3f} m, mean {result["e_mean"]:.3f} m')


if __name__ == '__main__':
    main()
