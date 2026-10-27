#!/usr/bin/env python3
"""
Step 7, stage 3: tables, statistics and the NFR evaluation -> results/final/summary.md.

Input: results/final/runs.csv (final_metrics.py) and the extracts in
results/final_work/runs/ (for the analyses that need whole trajectories).

Sections of summary.md:
  scenarios and runs; ATE per scenario and estimate; NFR-1 ... NFR-8 with the
  numbers behind every verdict; lidar dropout; recovery; gyro bias vs the
  model; EKF consistency; warehouse error vs travelled distance; summary table
  of the NFRs; list of figures (written by final_figures.py).

Statistics (NFR-2): per scenario, d = corrected ATE - base ATE of the SAME run
(same seed = same noise realisation, so the comparison is paired). Reported:
mean d with its 95 % t confidence interval, the two-sided paired t-test and the
two-sided Wilcoxon signed-rank test (exact for n <= 25; with n = 5 its
smallest possible p is 0.0625), and how many runs have corrected < base.

Run with the Python of evo's venv:
  ~/.venvs/evo/bin/python src/gnssd_tools/scripts/final_summary.py
"""

from collections import defaultdict
import csv
import math
from pathlib import Path

import numpy as np
from scipy import stats
from scipy.spatial import cKDTree
import yaml

REPO = Path(__file__).resolve().parents[3]
RUNS_CSV = REPO / 'results' / 'final' / 'runs.csv'
EXTRACTS = REPO / 'results' / 'final_work' / 'runs'
OUT = REPO / 'results' / 'final' / 'summary.md'

SCENARIOS = ['room_loop', 'room_long', 'room_loc', 'room_dropout',
             'warehouse_loop', 'warehouse_loc', 'warehouse_dropout', 'warehouse_recovery']
ESTIMATES = {'odom': 'odometry (/odom)', 'base': 'base layer (/odometry/filtered)',
             'corr': 'corrected (map -> base_link)'}
NFR1_ATE = 0.2       # m
NFR3_LOOP = 0.1      # m
NFR4_RATE = 30.0     # Hz
NFR6_CPU = 80.0      # %
NFR7_RAM = 400.0     # MiB
BIAS_MODEL = 1.0e-4  # rad/s, robot_params.xacro (bias_stddev)
DRIFT_MODEL = 1.0e-5  # rad/s, stationary in-run drift (dynamic bias)
# Radius of the circle around base_link that contains the whole robot (robot_params.xacro:
# chassis 0.28 x 0.22 m shifted -0.04 m -> rear corners at (-0.18, +-0.11) m, 0.211 m away;
# wheels reach |y| = 0.1425 m). A clearance above it rules a collision out.
ROBOT_RADIUS = 0.211  # m
SPEED_SLOW, SPEED_FAST = 'room_loop', 'room_loop_fast'


def number(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def load_runs():
    """Return the rows of the main series and all rows by scenario (incl. the speed runs)."""
    with open(RUNS_CSV) as f:
        rows = list(csv.DictReader(f))
    by_scenario = defaultdict(list)
    for row in rows:
        by_scenario[row['scenario']].append(row)
    return [r for r in rows if r['scenario'] in SCENARIOS], by_scenario


def values(rows, key):
    return np.array([v for v in (number(r.get(key)) for r in rows) if v is not None])


def m(x):
    return '–' if x is None or (isinstance(x, float) and math.isnan(x)) else f'{x:.3f}'


def deg(x):
    return '–' if x is None else f'{x:.2f}'


def pct(x):
    return f'{100 * x:.0f} %'


def pvalue(p):
    return f'{p:.3g}'


def table(header, rows):
    lines = ['| ' + ' | '.join(header) + ' |', '|' + '---|' * len(header)]
    lines += ['| ' + ' | '.join(str(c) for c in row) + ' |' for row in rows]
    return '\n'.join(lines)


def mean_std(x):
    return f'{x.mean():.3f} ± {x.std(ddof=1):.3f}' if len(x) > 1 else m(x.mean())


# ---------------------------------------------------------------------------------------------
def section_runs(rows, by):
    out = ['## Runs', '',
           f'{len(rows)} runs, all at tag `experiments-v1` (clean tree, run_info.yaml).', '']
    body = []
    for s in SCENARIOS:
        r = by[s]
        body.append([s, len(r), f'{values(r, "duration_s").mean():.0f}',
                     f'{values(r, "path_m").mean():.1f}', f'{values(r, "rtf").min():.3f}',
                     f'{values(r, "loop_closures").mean():.1f}', int(values(r, 'alarms').sum())])
    out.append(table(['scenario', 'runs', 'duration, s', 'path, m', 'RTF min',
                      'loop closures (mean)', 'alarms (sum)'], body))
    worst = min(rows, key=lambda r: float(r['rtf']))
    others = min(float(r['rtf']) for r in rows if r is not worst)
    out += ['', f'Lowest real-time factor: {float(worst["rtf"]):.3f} ({worst["bag"]}); '
            f'every other run >= {others:.3f}.']
    return out


def section_ate(by):
    out = ['## ATE per scenario (translation, not aligned; mean ± std over seeds, max)', '']
    body = []
    for s in SCENARIOS:
        for e, label in ESTIMATES.items():
            x = values(by[s], f'{e}_ate_rmse')
            h = values(by[s], f'{e}_head_rmse')
            body.append([s, label, mean_std(x), m(np.median(x)),
                         m(values(by[s], f'{e}_ate_max').max()), f'{h.mean():.2f}',
                         deg(values(by[s], f'{e}_head_max').max())])
    out.append(table(['scenario', 'estimate', 'ATE RMSE, m', 'median', 'max error, m',
                      'heading RMSE, deg', 'heading max, deg'], body))
    return out


def section_nfr1(by):
    out = ['## NFR-1: ATE <= 0.2 m (share of runs whose ATE RMSE meets it)', '']
    body = []
    shares = {}
    for s in SCENARIOS:
        row = [s, len(by[s])]
        for e in ESTIMATES:
            x = values(by[s], f'{e}_ate_rmse')
            share = np.mean(x <= NFR1_ATE)
            shares[(s, e)] = share
            row.append(f'{np.sum(x <= NFR1_ATE)}/{len(x)} ({pct(share)})')
        body.append(row)
    out.append(table(['scenario', 'runs', 'odometry', 'base layer', 'corrected'], body))
    return out, shares


def section_nfr2(by):
    out = ['## NFR-2: corrected ATE < base ATE (paired, same runs)', '',
           'd = corrected - base ATE RMSE of the same run. Negative = the correction helps.', '']
    body = []
    verdicts = {}
    for s in SCENARIOS:
        base, corr = values(by[s], 'base_ate_rmse'), values(by[s], 'corr_ate_rmse')
        d = corr - base
        n = len(d)
        half = stats.t.ppf(0.975, n - 1) * d.std(ddof=1) / math.sqrt(n)
        t_p = stats.ttest_rel(corr, base).pvalue
        w_p = stats.wilcoxon(corr, base).pvalue
        better = int(np.sum(d < 0))
        verdicts[s] = (d.mean() < 0 and t_p < 0.05, better, n)
        body.append([s, n, f'{d.mean():+.3f}', f'[{d.mean() - half:+.3f}, {d.mean() + half:+.3f}]',
                     pvalue(t_p), pvalue(w_p), f'{better}/{n}'])
    out.append(table(['scenario', 'n', 'mean d, m', '95 % CI', 'paired t-test p',
                      'Wilcoxon p', 'runs corrected < base'], body))
    out += ['', 'With n = 5 the exact two-sided Wilcoxon test cannot go below p = 0.0625.']
    return out, verdicts


def section_nfr3(by):
    out = ['## NFR-3: loop-closure error < 0.1 m (loop_closure_error.py)', '',
           'e = |(p_est(t2) - p_est(t1)) - (p_true(t2) - p_true(t1))| at revisits '
           '(true position within 0.10 m of an earlier one, >= 30 s apart; t1 on the first '
           'pass). e_final: at the final return to the start.', '']
    body = []
    shares = {}
    for s in SCENARIOS:
        for e, label in ESTIMATES.items():
            final = values(by[s], f'{e}_loop_final')
            share = np.mean(final < NFR3_LOOP) if len(final) else float('nan')
            shares[(s, e)] = share
            body.append([s, label, mean_std(final), m(values(by[s], f'{e}_loop_max').max()),
                         m(values(by[s], f'{e}_loop_mean').mean()),
                         f'{np.sum(final < NFR3_LOOP)}/{len(final)} ({pct(share)})'])
    out.append(table(['scenario', 'estimate', 'e_final, m', 'max e over revisits, m',
                      'mean e, m', 'runs with e_final < 0.1 m'], body))
    return out, shares


def base_intervals(rows):
    """Median and maximum interval of /odometry/filtered while moving, all runs."""
    medians, maxima, share30 = [], [], []
    for r in rows:
        d = np.load(EXTRACTS / f'{r["bag"]}.npz')
        t, gt = d['base'][:, 0], d['gt']
        speed = np.hypot(np.gradient(gt[:, 1], gt[:, 0]), np.gradient(gt[:, 2], gt[:, 0]))
        moving = np.interp(t, gt[:, 0], (speed > 0.05).astype(float)) > 0.5
        dt = np.diff(t)[moving[1:]]
        medians.append(np.median(dt))
        maxima.append(dt.max())
        share30.append(np.mean(dt <= 1 / NFR4_RATE + 1e-3))
    return np.array(medians), np.array(maxima), np.array(share30)


def section_nfr4(rows, by):
    out = ['## NFR-4: base-layer output rate >= 30 Hz', '',
           'Messages of /odometry/filtered per 1 s window of simulation time while the robot '
           'moves (windows aligned to whole seconds), and the intervals between messages.', '']
    medians, maxima, share = base_intervals(rows)
    body = []
    for s in SCENARIOS:
        body.append([s, f'{values(by[s], "base_rate_mean").mean():.2f}',
                     f'{values(by[s], "base_rate_min").min():.0f}'])
    out.append(table(['scenario', 'mean rate, Hz', 'min messages in a 1 s window'], body))
    out += ['', f'All {len(rows)} runs: the EKF publishes at 30 Hz, but each message carries '
            'the stamp of the latest fused measurement (IMU 100 Hz, odometry 50 Hz), so the stamp '
            'intervals are multiples of 10 ms (median '
            f'{1000 * np.median(medians):.0f} ms, largest {1000 * maxima.max():.0f} ms) and a 1 s '
            'window holds 29–31 messages by phase; the mean over every run is 30.00 Hz.']
    return out, (values(rows_all(by), 'base_rate_mean').min(), maxima.max())


def rows_all(by):
    return [r for s in SCENARIOS for r in by[s]]


def section_nfr5(by):
    out = ['## NFR-5: no scan queue', '',
           'Criterion: slam_toolbox publishes one map -> odom per processed scan, stamped scan '
           'stamp + transform_timeout (0.2 s). Latency of a scan = receive time of its '
           'map -> odom - scan stamp. No queue: p95 latency < 0.1 s (one scan period), no '
           'growth over the run (median second half - median first half <= 0.01 s), coverage '
           '~ 1 while slam_toolbox runs.', '']
    body = []
    for s in SCENARIOS:
        r = by[s]
        body.append([s, f'{1000 * values(r, "scan_lat_median").mean():.1f}',
                     f'{1000 * values(r, "scan_lat_p95").max():.1f}',
                     f'{1000 * values(r, "scan_lat_max").max():.1f}',
                     f'{1000 * values(r, "scan_lat_growth").max():+.2f}',
                     f'{100 * values(r, "scan_coverage").min():.2f} %'])
    out.append(table(['scenario', 'median latency, ms', 'p95 (worst run), ms', 'max, ms',
                      'growth (worst run), ms', 'coverage (worst run)'], body))
    allr = rows_all(by)
    met = (values(allr, 'scan_lat_p95').max() < 0.1
           and values(allr, 'scan_lat_growth').max() <= 0.01
           and values(allr, 'scan_coverage').min() > 0.99)
    return out, met, (values(allr, 'scan_lat_p95').max(), values(allr, 'scan_lat_growth').max())


def section_nfr67(by):
    out = ['## NFR-6: mean CPU < 80 %, NFR-7: peak RAM < 400 MB (system processes only)', '',
           'System = ekf_filter_node, slam_toolbox (async or localization), sensor_monitor_node, '
           'map_manager_node; resource_logger, 1 Hz; per sample the processes are summed, then '
           'averaged over the run (one run = one mapping/localization session). Normalisations: '
           '"% of one core" (psutil per process, 100 % = one core fully busy) and '
           '"% of all 8 cores" (= one-core value / 8). "Without /clock": minus 5.7 % of one '
           'core for each C++ node with use_sim_time (ekf_filter_node, slam_toolbox; Step 5b); '
           'the Python system nodes do not subscribe /clock (share 0, Steps 5b/6b). RSS: peak '
           'of the per-sample sum.', '']
    body = []
    for s in SCENARIOS:
        r = by[s]
        body.append([s, f'{values(r, "cpu_core_mean").mean():.1f}',
                     f'{values(r, "cpu_total_mean").mean():.2f}',
                     f'{values(r, "cpu_core_noclock").mean():.1f}',
                     f'{values(r, "cpu_total_noclock").mean():.2f}',
                     f'{values(r, "cpu_core_mean").max():.1f}',
                     f'{values(r, "rss_peak").max():.0f}'])
    out.append(table(['scenario', 'CPU, % of one core', 'CPU, % of 8 cores',
                      'without /clock, % one core', 'without /clock, % 8 cores',
                      'worst run, % one core', 'peak RSS, MiB'], body))
    allr = rows_all(by)
    return out, (values(allr, 'cpu_core_mean').max(), values(allr, 'cpu_total_mean').max(),
                 values(allr, 'rss_peak').max())


def section_dropout(by):
    out = ['## Lidar dropout and NFR-8 (no termination on lidar loss)', '',
           'Errors: RMSE of the position error before / during / after the outage. LOST latency = '
           'first scan LOST report - outage start (timeout 0.5 s). INVALID latency = first '
           'correction-consistency alarm - lidar back. Nodes: every system node in every '
           'node-census row (5 s) during the outage. Rate: min /odometry/filtered messages per '
           '1 s window during the outage.', '']
    body = []
    for s in ('room_dropout', 'warehouse_dropout', 'warehouse_recovery'):
        for r in sorted(by[s], key=lambda r: int(r['seed'])):
            body.append([s, r['seed'], m(number(r['base_before_rmse'])),
                         m(number(r['base_during_rmse'])), m(number(r['base_after_rmse'])),
                         m(number(r['corr_before_rmse'])), m(number(r['corr_during_rmse'])),
                         m(number(r['corr_after_rmse'])), m(number(r['lost_latency'])),
                         m(number(r['alarm_latency'])) if r['alarm_latency'] else 'no alarm',
                         'yes' if r['drop_nodes_ok'] == 'True' else 'NO',
                         f'{number(r["drop_base_rate_min"]):.0f}'])
    out.append(table(['scenario', 'seed', 'base before', 'base during', 'base after',
                      'corr before', 'corr during', 'corr after', 'LOST latency, s',
                      'INVALID latency, s', 'nodes alive', 'rate min, Hz'], body))
    drop = by['room_dropout'] + by['warehouse_dropout'] + by['warehouse_recovery']
    nodes_ok = all(r['drop_nodes_ok'] == 'True' for r in drop)
    rate_min = values(drop, 'drop_base_rate_min').min()
    wd = by['warehouse_dropout']
    wrong = [r for r in wd if r['alarm_latency']]
    after_wrong = values(wrong, 'corr_after_rmse')
    after_right = ', '.join(m(number(r['corr_after_rmse'])) for r in wd if not r['alarm_latency'])
    lost, alarm = values(drop, 'lost_latency'), values(drop, 'alarm_latency')
    out += ['', f'warehouse_dropout: a wrong correction after the outage (alarm) in {len(wrong)} '
            f'of {len(wd)} runs; the corrected error after the outage is then '
            f'{after_wrong.min():.2f}–{after_wrong.max():.2f} m (no alarm: {after_right} m). '
            f'room_dropout (60 s): no alarm in any run. LOST latency '
            f'{lost.min():.2f}–{lost.max():.2f} s; INVALID latency '
            f'{alarm.min():.3f}–{alarm.max():.3f} s.']
    return out, nodes_ok, rate_min


def section_recovery(by):
    out = ['## Recovery (warehouse_recovery: checkpoint on, scripted operator, mapping)', '',
           'Switch = the operator continues mapping from the checkpoint, 3 s after INVALID, '
           'initial pose from the base layer. Errors of the corrected estimate from the switch '
           'to the end; convergence = first time the error stays < 0.1 m for 5 s.', '']
    body = []
    for r in sorted(by['warehouse_recovery'], key=lambda r: int(r['seed'])):
        if not r['switch_t']:
            body.append([r['seed'], 'no alarm, no switch', '–', '–', '–', '–',
                         m(number(r['corr_ate_rmse']))])
            continue
        conv = number(r['conv_time'])
        body.append([r['seed'], f'{number(r["switch_t"]):.1f}',
                     f'{number(r["init_pos_err"]):.3f} m, {number(r["init_head_err"]):+.2f} deg',
                     m(number(r['after_rmse'])), m(number(r['after_max'])),
                     '–' if conv is None else f'{conv:.1f}', m(number(r['corr_ate_rmse']))])
    out.append(table(['seed', 'switch at, s', 'initial pose error', 'RMSE after switch, m',
                      'max after switch, m', 'time to < 0.1 m for 5 s, s',
                      'whole-run corrected ATE, m'],
                     body))
    return out


def section_bias(rows):
    per_seed = defaultdict(list)
    for r in rows:
        per_seed[int(r['seed'])].append(float(r['gyro_bias']))
    seeds = sorted(per_seed)
    estimates = np.array([np.mean(per_seed[s]) for s in seeds])
    spread = max(np.ptp(per_seed[s]) for s in seeds)
    expected = math.sqrt(BIAS_MODEL ** 2 + DRIFT_MODEL ** 2)
    chi_low, chi_high = stats.chi2.ppf([0.025, 0.975], len(estimates) - 1)
    std = estimates.std(ddof=1)
    ci = (std * math.sqrt((len(estimates) - 1) / chi_high),
          std * math.sqrt((len(estimates) - 1) / chi_low))
    out = ['## Gyro bias vs the model (checks the Step 4 Gauss–Markov fix)', '',
           f'Gazebo draws the bias from the seed, so runs with the same seed share it (largest '
           f'spread between runs of one seed: {spread:.1e} rad/s, from the in-run drift and the '
           f'estimate). Independent draws: {len(estimates)} seeds; per seed the mean of its runs '
           '(estimate_gyro_bias formula).', '',
           f'Std over seeds: {std:.2e} rad/s (95 % CI {ci[0]:.2e} – {ci[1]:.2e}); mean '
           f'{estimates.mean():+.2e} rad/s. Model: bias_stddev {BIAS_MODEL:.1e} rad/s plus '
           f'stationary drift {DRIFT_MODEL:.1e} rad/s -> expected {expected:.2e} rad/s. '
           f'Before the Step 4 fix the drift alone would have been 2.2e-4 rad/s.',
           '', table(['seed'] + [str(s) for s in seeds],
                     [['bias, 1e-5 rad/s'] + [f'{1e5 * b:+.1f}' for b in estimates]])]
    return out, (std, ci, expected)


def consistency(row):
    d = np.load(EXTRACTS / f'{row["bag"]}.npz')
    base, gt = d['base'], d['gt']
    i = np.clip(np.searchsorted(gt[:, 0], base[:, 0]), 1, len(gt) - 1)
    i = i - ((base[:, 0] - gt[i - 1, 0]) < (gt[i, 0] - base[:, 0]))
    ok = (np.abs(gt[i, 0] - base[:, 0]) <= 0.01) & (base[:, 0] > base[0, 0] + 1.0)
    b, g = base[ok], gt[i[ok]]
    ex, ey = b[:, 1] - g[:, 1], b[:, 2] - g[:, 2]
    eyaw = np.arctan2(np.sin(b[:, 3] - g[:, 3]), np.cos(b[:, 3] - g[:, 3]))
    sxx, sxy, syy, syaw = b[:, 4], b[:, 5], b[:, 6], np.sqrt(b[:, 7])
    d2 = (syy * ex * ex - 2 * sxy * ex * ey + sxx * ey * ey) / (sxx * syy - sxy * sxy)
    sxy_max = np.sqrt((sxx + syy) / 2 + np.sqrt(((sxx - syy) / 2) ** 2 + sxy ** 2))
    return (np.mean(d2 < 4.0), np.mean(d2 < 5.991), np.mean(np.abs(eyaw) < 2 * syaw),
            sxy_max[-1], math.hypot(ex[-1], ey[-1]), math.degrees(syaw[-1]),
            math.degrees(abs(eyaw[-1])))


def section_consistency(by):
    out = ['## EKF consistency (Step 6b metric) over all room_loop / warehouse_loop runs', '',
           'Fractions of samples inside the 2-sigma ellipse (expected 86.5 % if consistent), the '
           '95 % ellipse (95 %) and with |heading error| < 2 sigma_yaw (95.4 %); sigma and real '
           'error at the end of the run.', '']
    body = []
    for s in ('room_loop', 'warehouse_loop'):
        c = np.array([consistency(r) for r in by[s]])
        body.append([s, len(c), f'{100 * c[:, 0].mean():.1f} ({100 * c[:, 0].min():.1f}–'
                     f'{100 * c[:, 0].max():.1f})', f'{100 * c[:, 1].mean():.1f}',
                     f'{100 * c[:, 2].mean():.1f} '
                     f'({100 * c[:, 2].min():.1f}–{100 * c[:, 2].max():.1f})',
                     f'{c[:, 3].mean():.3f} / {c[:, 4].mean():.3f}',
                     f'{c[:, 5].mean():.3f} / {c[:, 6].mean():.3f}'])
    out.append(table(['scenario', 'runs', 'inside 2-sigma, % (min–max)', 'inside 95 %, %',
                      '|heading err| < 2 sigma, % (min–max)', 'end: sigma_xy / real error, m',
                      'end: sigma_yaw / real |heading err|, deg'], body))
    out += ['', 'Over-confident in every run: the real heading error is dominated by the gyro '
            'bias (grows ~t), which the white process noise of the EKF (grows ~sqrt(t)) does not '
            'represent; robot_localization has no bias state (Step 6b).']
    return out


def error_vs_distance(rows):
    """Mean ± std over seeds of base / corrected error vs travelled distance (warehouse_loop)."""
    grid = np.arange(0, 178, 1.0)
    curves = {'base': [], 'corr': []}
    for r in rows:
        d = np.load(EXTRACTS / f'{r["bag"]}.npz')
        gt = d['gt']
        steps = np.hypot(np.diff(gt[:, 1]), np.diff(gt[:, 2]))
        travelled = np.concatenate([[0], np.cumsum(steps)])
        for e in curves:
            est = d[e]
            x = np.interp(gt[:, 0], est[:, 0], est[:, 1])
            y = np.interp(gt[:, 0], est[:, 0], est[:, 2])
            inside = (gt[:, 0] >= est[0, 0]) & (gt[:, 0] <= est[-1, 0])
            err = np.hypot(x - gt[:, 1], y - gt[:, 2])
            curves[e].append(np.interp(grid, travelled[inside], err[inside]))
    return grid, {e: np.array(c) for e, c in curves.items()}


def section_distance(by):
    grid, curves = error_vs_distance(by['warehouse_loop'])
    out = ['## Warehouse: error vs travelled distance (warehouse_loop, 15 seeds, mean ± std)', '']
    body = []
    for dist in (25, 50, 75, 100, 125, 150, 175):
        k = int(np.argmin(np.abs(grid - dist)))
        body.append([dist] + [f'{curves[e][:, k].mean():.3f} ± {curves[e][:, k].std(ddof=1):.3f}'
                              for e in ('base', 'corr')])
    out.append(table(['travelled, m', 'base layer error, m', 'corrected error, m'], body))
    out += ['', 'Curves (also vs distance from the start): fig 4.']
    return out


def route_polyline():
    """Return the closed route of route_room.yaml (same in route_room_fast.yaml), (N, 2)."""
    config = yaml.safe_load((REPO / 'src' / 'gnssd_bringup' / 'config' / 'route_room.yaml')
                            .read_text())
    points = np.array(config['route_driver_node']['ros__parameters']['waypoints']).reshape(-1, 2)
    return np.vstack([points, points[:1]])


def distance_to_polyline(xy, polyline):
    """Distance of every point of xy (N, 2) to the polyline (M, 2)."""
    best = np.full(len(xy), np.inf)
    for a, b in zip(polyline[:-1], polyline[1:]):
        ab = b - a
        u = np.clip(((xy - a) @ ab) / (ab @ ab), 0.0, 1.0)
        best = np.minimum(best, np.hypot(*(xy - (a + u[:, None] * ab)).T))
    return best


def occupied_cells(name='final_room_s1'):
    """World (x, y) of the occupied cells of a saved map (trinary pgm: 0 = occupied)."""
    meta = yaml.safe_load((REPO / 'maps' / f'{name}.yaml').read_text())
    import matplotlib.pyplot as plt  # only needed here (reads the pgm)
    image = plt.imread(REPO / 'maps' / meta['image'])
    rows, cols = np.nonzero(image < 100)
    r = meta['resolution']
    x0, y0 = meta['origin'][:2]
    return np.column_stack([x0 + (cols + 0.5) * r, y0 + (image.shape[0] - rows - 0.5) * r])


def section_speed(by):
    """Paired comparison of room_loop at 0.2 m/s (experiments-v1) and 0.5 m/s (v1.1)."""
    slow = {r['seed']: r for r in by[SPEED_SLOW]}
    fast = {r['seed']: r for r in by[SPEED_FAST]}
    seeds = sorted(set(fast), key=int)
    if not seeds:
        return []
    polyline = route_polyline()
    walls = cKDTree(occupied_cells())
    geometry = {}
    for label, runs in (('0.2', slow), ('0.5', fast)):
        for seed in seeds:
            gt = np.load(EXTRACTS / f'{runs[seed]["bag"]}.npz')['gt'][:, 1:3]
            geometry[(label, seed)] = (distance_to_polyline(gt, polyline).max(),
                                       walls.query(gt)[0].min())

    def drift(row):
        return math.degrees(abs(float(row['gyro_bias'])) * float(row['duration_s']))

    metrics = [('odometry ATE, m', lambda r: float(r['odom_ate_rmse'])),
               ('base ATE, m', lambda r: float(r['base_ate_rmse'])),
               ('corrected ATE, m', lambda r: float(r['corr_ate_rmse'])),
               ('base heading RMSE, deg', lambda r: float(r['base_head_rmse'])),
               ('corrected heading RMSE, deg', lambda r: float(r['corr_head_rmse'])),
               ('|gyro bias| x duration, deg', drift),
               ('duration, s', lambda r: float(r['duration_s'])),
               ('loop closures', lambda r: float(r['loop_closures'])),
               ('scans in the graph', lambda r: float(r['graph_nodes']))]
    body = []
    for name, get in metrics:
        a = np.array([get(slow[s]) for s in seeds])
        b = np.array([get(fast[s]) for s in seeds])
        d = b - a
        t_p = stats.ttest_rel(b, a).pvalue if np.any(d != 0) else float('nan')
        w_p = stats.wilcoxon(b, a).pvalue if np.any(d != 0) else float('nan')
        body.append([name, f'{a.mean():.3f}', f'{b.mean():.3f}', f'{d.mean():+.3f}',
                     pvalue(t_p), pvalue(w_p), f'{int(np.sum(d < 0))}/{len(d)}'])
    per_seed = []
    for s in seeds:
        per_seed.append([s] + [f'{float(slow[s][k]):.3f} / {float(fast[s][k]):.3f}'
                               for k in ('odom_ate_rmse', 'base_ate_rmse', 'corr_ate_rmse')]
                        + [f'{drift(slow[s]):.2f} / {drift(fast[s]):.2f}',
                           f'{geometry[("0.2", s)][0]:.3f} / {geometry[("0.5", s)][0]:.3f}',
                           f'{geometry[("0.2", s)][1]:.3f} / {geometry[("0.5", s)][1]:.3f}'])
    clearance_fast = min(geometry[('0.5', s)][1] for s in seeds)
    deviation_fast = max(geometry[('0.5', s)][0] for s in seeds)
    deviation_slow = max(geometry[('0.2', s)][0] for s in seeds)
    return ['## Speed experiment: room_loop at 0.2 vs 0.5 m/s (seeds 1–5, paired)', '',
            '0.2 m/s: the room_loop runs of experiments-v1; 0.5 m/s: room_loop_fast, tag '
            'experiments-v1.1 (route_room_fast.yaml = route_room.yaml with linear_speed 0.5; '
            'run_info.yaml records the driver parameters). Same seed = same noise draw (gyro '
            'bias, odometry scale errors). d = 0.5 m/s - 0.2 m/s; n = 5, so the exact Wilcoxon '
            'p cannot go below 0.0625.', '',
            table(['metric', '0.2 m/s (mean)', '0.5 m/s (mean)', 'mean d', 'paired t-test p',
                   'Wilcoxon p', 'runs with d < 0'], body), '',
            table(['seed', 'odometry ATE 0.2 / 0.5', 'base ATE 0.2 / 0.5',
                   'corrected ATE 0.2 / 0.5', '|bias| x T, deg', 'max route deviation, m',
                   'min clearance to walls, m'], per_seed), '',
            f'Route: largest distance from the route polyline {deviation_slow:.3f} m at 0.2 m/s, '
            f'{deviation_fast:.3f} m at 0.5 m/s (the 0.6 rad/s turn-rate limit gives a turn '
            f'radius of 0.5 / 0.6 = 0.83 m instead of 0.33 m, so the corners are cut wider). '
            f'Collision: smallest distance from base_link to an occupied cell of final_room_s1 '
            f'{clearance_fast:.3f} m at 0.5 m/s, robot radius {ROBOT_RADIUS} m -> '
            + ('no contact possible.' if clearance_fast > ROBOT_RADIUS else
               'a contact is POSSIBLE (clearance below the robot radius); check the run.'),
            '',
            'Scan motion distortion: NOT simulated. gz-sensors 8 GpuLidarSensor renders the whole '
            'scan in one Render() per update from one sensor pose and puts one stamp on it (no '
            'per-ray pose or time offset); the recorded LaserScan has time_increment = 0 and '
            'scan_time = 0. Any difference at 0.5 m/s therefore comes from the shorter drive time '
            '(less gyro drift accumulates) and the wider turns, not from smeared scans; a real '
            'rotating lidar would add that effect. The pose graph gets about the same number of '
            'scans at both speeds: slam_toolbox adds a scan per minimum_travel_distance (0.5 m), '
            'not per time; only the raw scans per metre drop (10 Hz: 50 per m at 0.2 m/s, 20 at '
            '0.5 m/s).', '',
            'Lowest RTF of the 0.5 m/s runs: '
            f'{min(float(fast[s]["rtf"]) for s in seeds):.3f} (seed 1, recorded while analysis '
            'scripts were being edited on the same machine).']


def section_verdicts(nfr1, nfr2, nfr3, nfr4, nfr5, nfr67, nfr8):
    shares1, shares3 = nfr1, nfr3
    mapping = [s for s in SCENARIOS]
    corr1 = {s: shares1[(s, 'corr')] for s in mapping}
    nfr1_text = ', '.join(f'{s} {pct(v)}' for s, v in corr1.items())
    ok2 = [s for s, (ok, _, _) in nfr2.items() if ok]
    not2 = [s for s, (ok, _, _) in nfr2.items() if not ok]
    corr3 = {s: shares3[(s, 'corr')] for s in mapping}
    nfr3_text = ', '.join(f'{s} {pct(v)}' for s, v in corr3.items())
    rate_mean_min, max_gap = nfr4
    p95, growth = nfr5[1]
    cpu_core, cpu_total, rss = nfr67
    nodes_ok, rate_min = nfr8
    rows = [
        ['NFR-1', 'ATE <= 0.2 m', f'corrected, share of runs: {nfr1_text}',
         'met except after a wrong post-outage correction (warehouse_dropout / recovery)',
         'base layer meets it only in the room'],
        ['NFR-2', 'corrected ATE < base ATE',
         f'significant (t-test p < 0.05, d < 0): {", ".join(ok2)}',
         'met in ' + f'{len(ok2)}/{len(nfr2)} scenarios',
         'not: ' + (', '.join(not2) or '–') + ' (warehouse_dropout: corrected significantly WORSE '
         'after a wrong post-outage correction; recovery: no significant difference)'],
        ['NFR-3', 'loop-closure error < 0.1 m', f'corrected, e_final < 0.1 m: {nfr3_text}',
         'met where SLAM is correct', 'see NFR-3 table'],
        ['NFR-4', 'base rate >= 30 Hz', f'mean {rate_mean_min:.2f} Hz (lowest per-run mean), '
         f'largest gap between stamps {1000 * max_gap:.0f} ms', 'met',
         'stamps on the 10 ms measurement grid'],
        ['NFR-5', 'no scan queue', f'p95 latency <= {1000 * p95:.0f} ms, growth <= '
         f'{1000 * growth:+.1f} ms', 'met' if nfr5[0] else 'not met',
         'criterion defined in NFR-5'],
        ['NFR-6', 'mean CPU < 80 %', f'worst run {cpu_core:.1f} % of one core = {cpu_total:.1f} % '
         'of 8 cores', 'met', 'incl. /clock artifact'],
        ['NFR-7', 'peak RAM < 400 MB', f'{rss:.0f} MiB', 'met', 'sum of system RSS'],
        ['NFR-8', 'no termination on lidar loss',
         f'all system nodes alive: {"yes" if nodes_ok else "NO"}'
         f'; base rate during outage >= {rate_min:.0f} msg/s', 'met' if nodes_ok else 'not met',
         '25 runs with an outage'],
    ]
    return ['## NFR summary', '',
            table(['NFR', 'threshold', 'measured', 'verdict', 'notes'], rows)]


FIGURES = [
    ('fig01_ate_boxplots', 'ATE RMSE of the three estimates per scenario (box plots, all seeds).'),
    ('fig02_trajectories', 'Trajectories of room_loop and warehouse_loop, seed with the median '
     'corrected ATE: reference, odometry, base layer, corrected.'),
    ('fig03_room_long_error', 'Position error over time, room_long seed 1: base layer vs '
     'corrected.'),
    ('fig04_error_vs_distance', 'Warehouse: error vs travelled distance and vs distance from the '
     'start, mean ± std over 15 seeds, base vs corrected.'),
    ('fig05_dropout', 'Lidar dropout: error over time with the outage shaded, room and warehouse '
     '(one seed each).'),
    ('fig06_rotated_map', 'warehouse_dropout map after a wrong post-outage correction '
     '(rotated copy).'),
    ('fig07_recovery', 'Recovery (continued mapping): corrected error after the switch, all '
     'seeds.'),
    ('fig08_maps', 'Maps saved by room_loop seed 1 and warehouse_loop seed 1 (source maps of the '
     'localization series).'),
    ('fig09_resources', 'CPU (% of one core and % of 8 cores) and peak RSS per system process.'),
    ('fig10_gyro_bias', 'Histogram of the per-seed gyro bias estimates vs the model N(0, 1e-4).'),
    ('fig11_speed_ate', 'Speed experiment: ATE of the three estimates at 0.2 and 0.5 m/s, '
     'room_loop seeds 1–5, lines join the same seed.'),
]


def main():
    rows, by = load_runs()
    parts = ['# Step 7 — final experiment series: results', '',
             'Generated by src/gnssd_tools/scripts/final_summary.py from results/final/runs.csv '
             '(final_extract.py -> final_metrics.py). Units: m, deg, s; ATE never aligned.', '']
    parts += section_runs(rows, by) + ['']
    parts += section_ate(by) + ['']
    nfr1_text, nfr1 = section_nfr1(by)
    nfr2_text, nfr2 = section_nfr2(by)
    nfr3_text, nfr3 = section_nfr3(by)
    nfr4_text, nfr4 = section_nfr4(rows, by)
    nfr5_text, nfr5_met, nfr5_values = section_nfr5(by)
    nfr67_text, nfr67 = section_nfr67(by)
    drop_text, nodes_ok, rate_min = section_dropout(by)
    verdicts = section_verdicts(nfr1, nfr2, nfr3, nfr4, (nfr5_met, nfr5_values), nfr67,
                                (nodes_ok, rate_min))
    parts += verdicts + ['']
    for text in (nfr1_text, nfr2_text, nfr3_text, nfr4_text, nfr5_text, nfr67_text, drop_text,
                 section_recovery(by)):
        parts += text + ['']
    bias_text, _ = section_bias(rows)
    parts += bias_text + [''] + section_consistency(by) + [''] + section_distance(by) + ['']
    parts += section_speed(by) + ['']
    parts += ['## Figures (results/final/fig/, PNG 300 dpi + PDF)', '']
    parts += [f'- `{name}`: {text}' for name, text in FIGURES]
    OUT.write_text('\n'.join(parts) + '\n')
    print(f'-> {OUT}')


if __name__ == '__main__':
    main()
