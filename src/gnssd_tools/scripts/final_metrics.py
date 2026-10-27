#!/usr/bin/env python3
"""
Step 7, stage 2: one row of metrics per run of the final series -> results/final/runs.csv.

Input: the per-run extracts of final_extract.py (results/final_work/runs/) and
the side files of every bag (run_info.yaml, loop_closures.yaml, resources.csv,
nodes.csv). Output columns (m, deg, s, %, MiB):

  identity     bag, scenario, world, seed, laps, duration_s, path_m, rtf
  model        gyro_bias, odom_scale_lin, odom_scale_ang (drawn per run)
  per estimate (odom = /odom, base = /odometry/filtered, corr = TF map -> base_link):
               <e>_ate_rmse, <e>_ate_mean, <e>_ate_max      translation, NOT aligned
               <e>_head_rmse, <e>_head_max                  heading, deg
               <e>_loop_final, <e>_loop_max, <e>_loop_mean  NFR-3 (loop_closure_error.py)
  slam         loop_closures, alarms (/correction_valid -> false)
  NFR-4        base_rate_mean, base_rate_min        /odometry/filtered, 1 s windows while moving
  NFR-5        scan_lat_median, scan_lat_p95, scan_lat_max, scan_lat_growth, scan_coverage
  NFR-6/7      cpu_core_mean, cpu_total_mean, cpu_core_noclock, cpu_total_noclock, rss_peak
  dropout      drop_start, drop_end, <base|corr>_<before|during|after>_rmse,
               lost_latency (scan LOST - outage start), alarm_latency (INVALID - lidar back),
               drop_nodes_ok (all system nodes in every census row during the outage),
               drop_base_rate_min
  recovery     switch_t, init_pos_err, init_head_err, after_rmse, after_max, conv_time

Matching an estimate to the ground truth: nearest ground-truth stamp within
MAX_DT (10 ms, as evo's default association); no alignment (CLAUDE.md).

NFR-5 (no scan queue), the measurable criterion: slam_toolbox publishes a
map -> odom for every scan it processes, stamped scan stamp + transform_timeout
(0.2 s, slam_toolbox.yaml). The processing latency of a scan is the receive
time of that map -> odom minus the scan stamp. No queue = the latency stays
below one scan period (p95 < 0.1 s) and does not grow (median of the second
half of the run - median of the first half = scan_lat_growth <= 0.01 s), and
almost every scan that arrives while slam_toolbox runs gets its map -> odom
(scan_coverage; start-up, a restart at a mode switch and outages excluded).

NFR-6 /clock artifact: ekf_filter_node and slam_toolbox (C++) handle the
1 kHz /clock of the simulation (use_sim_time); measured cost 5.7 % of one
core per C++ node (Step 5b). sensor_monitor_node and map_manager_node run
without use_sim_time since Steps 5b/6, so their share is 0. cpu_*_noclock
subtracts 5.7 % per C++ system process present in a sample.

Run with the Python of evo's venv:
  ~/.venvs/evo/bin/python src/gnssd_tools/scripts/final_metrics.py
"""

import csv
import json
import math
from pathlib import Path
import sys

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from loop_closure_error import loop_errors  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
RUNS = REPO / 'results' / 'final_work' / 'runs'
OUT = REPO / 'results' / 'final' / 'runs.csv'

MAX_DT = 0.01                 # s, estimate <-> ground truth association
MOVING_SPEED = 0.05           # m/s, "the robot is moving"
TRANSFORM_TIMEOUT = 0.2       # s, slam_toolbox stamps map -> odom with scan stamp + this
CLOCK_COST_CPP = 5.7          # % of one core per C++ node with use_sim_time (Step 5b)
CORES = 8
CONVERGED = 0.1               # m, recovery: error below this ...
HOLD = 5.0                    # s, ... for this long
SYSTEM_PROCESSES = ('ekf_filter_node', 'async_slam_toolbox_node',
                    'localization_slam_toolbox_node', 'sensor_monitor_node', 'map_manager_node')
CPP_PROCESSES = ('ekf_filter_node', 'async_slam_toolbox_node', 'localization_slam_toolbox_node')
SYSTEM_NODES = ('/ekf_filter_node', '/slam_toolbox', '/sensor_monitor_node', '/map_manager_node')
ESTIMATES = ('odom', 'base', 'corr')


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def errors(est, gt):
    """Time, position error and heading error (rad) of est (t, x, y, yaw) vs the ground truth."""
    i = np.clip(np.searchsorted(gt[:, 0], est[:, 0]), 1, len(gt) - 1)
    i = i - ((est[:, 0] - gt[i - 1, 0]) < (gt[i, 0] - est[:, 0]))
    ok = np.abs(gt[i, 0] - est[:, 0]) <= MAX_DT
    e, g = est[ok], gt[i[ok]]
    return e[:, 0], np.hypot(e[:, 1] - g[:, 1], e[:, 2] - g[:, 2]), wrap(e[:, 3] - g[:, 3])


def rmse(values):
    return float(np.sqrt(np.mean(np.square(values)))) if len(values) else None


def window_rmse(t, err, start, end):
    inside = (t >= start) & (t < end)
    return rmse(err[inside])


def moving_mask(gt):
    speed = np.hypot(np.gradient(gt[:, 1], gt[:, 0]), np.gradient(gt[:, 2], gt[:, 0]))
    return speed > MOVING_SPEED


def base_rate(base_t, gt, start=None, end=None):
    """Messages per 1 s window (mean, min) over windows in which the robot moves all the time."""
    moving = moving_mask(gt)
    start = math.ceil(gt[0, 0]) if start is None else start
    end = math.floor(gt[-1, 0]) if end is None else end
    counts = []
    for k in np.arange(start, end - 1, 1.0):
        in_gt = (gt[:, 0] >= k) & (gt[:, 0] < k + 1)
        if in_gt.any() and moving[in_gt].all():
            counts.append(np.count_nonzero((base_t >= k) & (base_t < k + 1)))
    return (float(np.mean(counts)), float(np.min(counts))) if counts else (None, None)


def scan_latency(scan, m2o):
    """NFR-5: latency of every processed scan; median, p95, max, growth, coverage."""
    if not len(m2o):
        return {}
    first_receive = {}
    for stamp_, received in m2o:
        key = round(stamp_ - TRANSFORM_TIMEOUT, 6)
        first_receive.setdefault(key, received)
    stamps = scan[:, 0]
    processed = np.array([round(s, 6) in first_receive for s in stamps])
    latency = np.array([first_receive[round(s, 6)] - s for s in stamps[processed]])
    times = stamps[processed]
    half = len(latency) // 2
    # Coverage counts only scans that arrived while slam_toolbox was running: between
    # the first and the last processed scan, outside gaps > 1 s between processed scans
    # (start-up, the restart at a mode switch, a lidar outage). A missing map -> odom
    # there is a scan slam_toolbox skipped.
    running = (stamps >= times[0]) & (stamps <= times[-1])
    for gap_start, gap_end in zip(times[:-1], times[1:]):
        if gap_end - gap_start > 1.0:
            running &= ~((stamps > gap_start) & (stamps < gap_end))
    return {'scan_lat_median': float(np.median(latency)),
            'scan_lat_p95': float(np.percentile(latency, 95)),
            'scan_lat_max': float(latency.max()),
            'scan_lat_growth': float(np.median(latency[half:]) - np.median(latency[:half])),
            'scan_coverage': float(np.mean(processed[running])),
            '_lat_t': times, '_lat': latency}


def resources(bag):
    """System CPU (mean of per-sample sums, raw and without the /clock artifact), peak RSS."""
    samples = {}
    with open(bag / 'resources.csv') as f:
        for row in csv.DictReader(f):
            if row['process'] in SYSTEM_PROCESSES:
                samples.setdefault(row['wall_time'], []).append(row)
    times = sorted(samples)[1:]          # the first psutil sample is always 0 % CPU
    core, core_noclock, rss = [], [], []
    for t in times:
        rows = samples[t]
        total = sum(float(r['cpu_core_pct']) for r in rows)
        cpp = sum(1 for r in rows if r['process'] in CPP_PROCESSES)
        core.append(total)
        core_noclock.append(total - CLOCK_COST_CPP * cpp)
        rss.append(sum(float(r['rss_mb']) for r in rows))
    return {'cpu_core_mean': float(np.mean(core)), 'cpu_total_mean': float(np.mean(core)) / CORES,
            'cpu_core_noclock': float(np.mean(core_noclock)),
            'cpu_total_noclock': float(np.mean(core_noclock)) / CORES,
            'rss_peak': float(np.max(rss))}


def census_ok(bag, start, end):
    """Tell whether every system node is in every node-census row taken during [start, end]."""
    with open(bag / 'nodes.csv') as f:
        rows = [r for r in csv.DictReader(f) if start <= float(r['sim_time']) <= end]
    return bool(rows) and all(all(n in r['nodes'].split(';') for n in SYSTEM_NODES)
                              for r in rows)


def first_event(events, kind, after, **match):
    for e in events:
        if e['kind'] == kind and e['t'] >= after and all(e.get(k) == v for k, v in match.items()):
            return e
    return None


def recovery_metrics(events, gt, corr_t, corr_err):
    load = next((e for e in events if e['kind'] == 'event' and e['text'].startswith('load map=')),
                None)
    if load is None:
        return {}
    t_switch = load['t']
    numbers = load['text'].split('pose=(')[1].rstrip(' deg)').split(', ')
    x, y, yaw = float(numbers[0]), float(numbers[1]), math.radians(float(numbers[2]))
    k = np.argmin(np.abs(gt[:, 0] - t_switch))
    after = corr_t >= t_switch
    ta, ea = corr_t[after], corr_err[after]
    conv = None
    below = ea < CONVERGED
    for i in range(len(ta)):
        j = np.searchsorted(ta, ta[i] + HOLD)
        if j < len(ta) and below[i:j + 1].all():
            conv = float(ta[i] - t_switch)
            break
    return {'switch_t': t_switch,
            'init_pos_err': float(math.hypot(gt[k, 1] - x, gt[k, 2] - y)),
            'init_head_err': float(math.degrees(wrap(yaw - gt[k, 3]))),
            'after_rmse': rmse(ea), 'after_max': float(ea.max()), 'conv_time': conv}


def metrics(npz_path):
    name = npz_path.stem
    bag = REPO / 'bags' / name
    d = np.load(npz_path)
    info = json.loads(npz_path.with_suffix('.json').read_text())
    run = yaml.safe_load((bag / 'run_info.yaml').read_text())
    loops = yaml.safe_load((bag / 'loop_closures.yaml').read_text())
    meta = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    gt, events = d['gt'], info['events']

    row = {'bag': name, 'scenario': run['scenario'], 'world': run['world'], 'seed': run['seed'],
           'laps': run['laps'], 'duration_s': meta['duration']['nanoseconds'] * 1e-9,
           'path_m': float(np.sum(np.hypot(np.diff(gt[:, 1]), np.diff(gt[:, 2])))),
           'rtf': info['rtf'], 'gyro_bias': info['gyro_bias'],
           'odom_scale_lin': run['odom_noise']['applied_linear_scale_error'],
           'odom_scale_ang': run['odom_noise']['applied_angular_scale_error'],
           'loop_closures': loops['loop_closures'],
           'alarms': sum(1 for e in events if e['kind'] == 'valid' and not e['value'])}

    series = {}
    for est in ESTIMATES:
        t, err, head = errors(d[est], gt)
        series[est] = (t, err)
        row.update({f'{est}_ate_rmse': rmse(err), f'{est}_ate_mean': float(err.mean()),
                    f'{est}_ate_max': float(err.max()),
                    f'{est}_head_rmse': math.degrees(rmse(head)),
                    f'{est}_head_max': float(np.degrees(np.abs(head).max()))})
        loop = loop_errors(gt[:, :3], d[est][:, :3])
        row.update({f'{est}_loop_final': loop['e_final'], f'{est}_loop_max': loop['e_max'],
                    f'{est}_loop_mean': loop['e_mean']})

    row['base_rate_mean'], row['base_rate_min'] = base_rate(d['base'][:, 0], gt)
    row.update({k: v for k, v in scan_latency(d['scan'], d['m2o']).items()
                if not k.startswith('_')})
    row.update(resources(bag))

    windows = run.get('scan_dropout_windows') or []
    if windows:
        start, end = windows[0]
        row.update({'drop_start': start, 'drop_end': end})
        for est in ('base', 'corr'):
            t, err = series[est]
            row[f'{est}_before_rmse'] = window_rmse(t, err, -1e9, start)
            row[f'{est}_during_rmse'] = window_rmse(t, err, start, end)
            row[f'{est}_after_rmse'] = window_rmse(t, err, end, 1e9)
        lost = first_event(events, 'diag', start - 1.0, name='sensor_monitor: scan', level=2)
        alarm = first_event(events, 'diag', end - 1.0,
                            name='sensor_monitor: correction consistency', level=2)
        row['lost_latency'] = None if lost is None else lost['t'] - start
        row['alarm_latency'] = None if alarm is None else alarm['t'] - end
        row['drop_nodes_ok'] = census_ok(bag, start, end)
        row['drop_base_rate_min'] = base_rate(d['base'][:, 0], gt, math.ceil(start),
                                              math.floor(end))[1]
    row.update(recovery_metrics(events, gt, *series['corr']))
    return row


def main():
    OUT.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    for npz in sorted(RUNS.glob('*.npz')):
        rows.append(metrics(npz))
        r = rows[-1]
        print(f'{r["bag"]:45s} base {r["base_ate_rmse"]:.3f}  corr {r["corr_ate_rmse"]:.3f} m',
              flush=True)
    columns = []
    for r in rows:
        columns += [k for k in r if k not in columns]
    with open(OUT, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        for r in rows:
            writer.writerow({k: (f'{v:.6g}' if isinstance(v, float) else v)
                             for k, v in r.items()})
    print(f'{len(rows)} runs -> {OUT}')


if __name__ == '__main__':
    main()
