#!/usr/bin/env python3
"""
Evaluate a lidar-dropout run: errors before, during and after the outage.

For the base layer and the corrected estimate (TUM files, matched to the
ground truth by the nearest stamp, NO alignment) it prints:
  * ATE RMSE / mean / max in three intervals: before the outage, during it,
    and after it until the end of the run;
  * the largest corrected error during the outage;
  * the re-convergence time: how long after the lidar comes back the
    corrected error needs to fall below `threshold` (default 0.1 m) and stay
    there for `hold` seconds (default 5 s). "0.0" = it never exceeded the
    threshold; "not recovered" = it is above the threshold at the end.
and draws both errors over time with the outage shaded.

Run with the Python of evo's venv:
  ~/.venvs/evo/bin/python src/gnssd_tools/scripts/dropout_analysis.py
      ground_truth.tum base.tum corrected.tum <t_start> <t_end> [--plot out.png]
  (one command line)
"""

import argparse

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402  (must come after matplotlib.use)
import numpy as np  # noqa: E402

MAX_TIME_DIFFERENCE = 0.02  # s


def errors_over_time(ground_truth, path):
    """Return (times, position errors) of a TUM estimate against the ground truth."""
    true_t, true_xy = ground_truth
    data = np.loadtxt(path)
    est_t, est_xy = data[:, 0], data[:, 1:3]
    index = np.clip(np.searchsorted(true_t, est_t), 1, len(true_t) - 1)
    previous_is_closer = (est_t - true_t[index - 1]) < (true_t[index] - est_t)
    index = index - previous_is_closer
    close = np.abs(true_t[index] - est_t) <= MAX_TIME_DIFFERENCE
    return est_t[close], np.hypot(*(est_xy[close] - true_xy[index[close]]).T)


def statistics(errors):
    if len(errors) == 0:
        return 'no data'
    return (f'RMSE {np.sqrt(np.mean(errors ** 2)):.3f} m, mean {errors.mean():.3f} m, '
            f'max {errors.max():.3f} m')


def reconvergence_time(times, errors, t_end, threshold, hold):
    """Seconds after t_end until the error is below `threshold` for `hold` seconds."""
    after = times >= t_end
    times, errors = times[after], errors[after]
    if len(errors) == 0:
        return 'no data'
    if errors.max() < threshold and np.all(errors < threshold):
        return '0.0 s (never above the threshold)'
    for i in range(len(times)):
        if errors[i] >= threshold:
            continue
        window = (times >= times[i]) & (times <= times[i] + hold)
        if times[-1] >= times[i] + hold and np.all(errors[window] < threshold):
            return f'{times[i] - t_end:.1f} s'
    return 'not recovered'


def main():
    parser = argparse.ArgumentParser(description='Errors before/during/after a lidar dropout')
    parser.add_argument('ground_truth')
    parser.add_argument('base')
    parser.add_argument('corrected')
    parser.add_argument('t_start', type=float, help='outage start, sim time (s)')
    parser.add_argument('t_end', type=float, help='outage end, sim time (s)')
    parser.add_argument('--threshold', type=float, default=0.1)
    parser.add_argument('--hold', type=float, default=5.0)
    parser.add_argument('--plot', default=None)
    args = parser.parse_args()

    true = np.loadtxt(args.ground_truth)
    ground_truth = (true[:, 0], true[:, 1:3])

    results = {}
    for label, path in (('base layer', args.base), ('corrected', args.corrected)):
        times, errors = errors_over_time(ground_truth, path)
        results[label] = (times, errors)
        before = errors[times < args.t_start]
        during = errors[(times >= args.t_start) & (times < args.t_end)]
        after = errors[times >= args.t_end]
        print(f'{label}:')
        print(f'  before  ({times[0]:.0f}-{args.t_start:.0f} s): {statistics(before)}')
        print(f'  during  ({args.t_start:.0f}-{args.t_end:.0f} s): {statistics(during)}')
        print(f'  after   ({args.t_end:.0f}-{times[-1]:.0f} s): {statistics(after)}')

    times, errors = results['corrected']
    during = errors[(times >= args.t_start) & (times < args.t_end)]
    print(f'corrected max error during the outage: {during.max():.3f} m')
    print(f'corrected re-convergence (< {args.threshold} m for {args.hold} s): '
          f'{reconvergence_time(times, errors, args.t_end, args.threshold, args.hold)}')

    if args.plot:
        figure, axes = plt.subplots(figsize=(11, 4.5))
        axes.axvspan(args.t_start, args.t_end, color='tab:red', alpha=0.15, label='lidar dropout')
        for label, (t, e) in results.items():
            axes.plot(t, e, linewidth=0.9, label=label)
        axes.axhline(args.threshold, color='grey', linestyle='--', linewidth=0.8,
                     label=f'{args.threshold} m')
        axes.set_xlabel('sim time, s')
        axes.set_ylabel('position error, m')
        axes.set_title('Position error vs ground truth (not aligned)')
        axes.grid(True)
        axes.legend()
        figure.tight_layout()
        figure.savefig(args.plot, dpi=130)
        print(f'plot saved to {args.plot}')


if __name__ == '__main__':
    main()
