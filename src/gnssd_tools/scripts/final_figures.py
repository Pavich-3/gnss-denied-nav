#!/usr/bin/env python3
"""
Step 7, stage 4: the figures of the thesis -> results/final/fig/ (PNG 300 dpi + PDF).

One style for all figures: labels and legends in Ukrainian, SI units, no titles
inside the image (the captions are in the thesis), the same colour for each
estimate everywhere:
    еталон (ground truth) black, одометрія grey, базовий шар blue,
    скоригована оцінка orange.

Inputs: results/final/runs.csv, the extracts in results/final_work/runs/,
maps/final_room_s1.*, maps/final_wh_s1.* and, for figure 6, the last /map of
one warehouse_dropout bag. The list of figures is in results/final/summary.md.

Run with the Python of evo's venv:
  ~/.venvs/evo/bin/python src/gnssd_tools/scripts/final_figures.py
"""

from collections import defaultdict
import csv
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402  (must come after matplotlib.use)
from matplotlib.ticker import FuncFormatter  # noqa: E402
import numpy as np  # noqa: E402
from scipy import stats  # noqa: E402
import yaml  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from map_to_png import read_bag  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
RUNS_CSV = REPO / 'results' / 'final' / 'runs.csv'
EXTRACTS = REPO / 'results' / 'final_work' / 'runs'
FIG = REPO / 'results' / 'final' / 'fig'

COLOR = {'gt': '#1A1A1A', 'odom': '#9AA0A6', 'base': '#2F6DB5', 'corr': '#E07A1F'}
LABEL = {'gt': 'еталон', 'odom': 'одометрія', 'base': 'базовий шар',
         'corr': 'скоригована оцінка'}
SCENARIO_LABEL = {
    'room_loop': 'кімната,\nпетля', 'room_long': 'кімната,\n10 кіл',
    'room_loc': 'кімната,\nлокалізація', 'room_dropout': 'кімната,\nвтрата лідара',
    'warehouse_loop': 'склад,\nпетля', 'warehouse_loc': 'склад,\nлокалізація',
    'warehouse_dropout': 'склад,\nвтрата лідара', 'warehouse_recovery': 'склад,\nвідновлення'}
OUTAGE_LABEL = 'втрата лідара'

plt.rcParams.update({
    'font.family': 'DejaVu Sans', 'font.size': 10, 'axes.labelsize': 10,
    'legend.fontsize': 9, 'xtick.labelsize': 9, 'ytick.labelsize': 9,
    'axes.grid': True, 'grid.alpha': 0.3, 'axes.spines.top': False,
    'axes.spines.right': False, 'savefig.bbox': 'tight', 'pdf.fonttype': 42})


def decimal_comma(value, _position):
    """Tick label with the Ukrainian decimal comma (0,12 instead of 0.12)."""
    return f'{value:g}'.replace('.', ',').replace('-', '\u2212')


def save(figure, name, keep_x=False):
    """
    Write <name>.png (300 dpi) and <name>.pdf.

    Linear axes get decimal commas; log axes keep their 10^n labels. keep_x: the
    x tick labels are names set by hand (scenarios, processes) - leave them.
    """
    for axes in figure.axes:
        pairs = [(axes.yaxis, axes.get_yscale())]
        if not keep_x:
            pairs.append((axes.xaxis, axes.get_xscale()))
        for axis, scale in pairs:
            if scale == 'linear':
                axis.set_major_formatter(FuncFormatter(decimal_comma))
    FIG.mkdir(parents=True, exist_ok=True)
    figure.savefig(FIG / f'{name}.png', dpi=300)
    figure.savefig(FIG / f'{name}.pdf')
    plt.close(figure)
    print(f'{name}: png + pdf')


def load_runs():
    with open(RUNS_CSV) as f:
        rows = list(csv.DictReader(f))
    by = defaultdict(list)
    for r in rows:
        by[r['scenario']].append(r)
    for s in by:
        by[s].sort(key=lambda r: int(r['seed']))
    return rows, by


def extract(row):
    return np.load(EXTRACTS / f'{row["bag"]}.npz')


def position_error(est, gt):
    """Time and position error of est (t, x, y, ...) vs the nearest ground-truth sample."""
    i = np.clip(np.searchsorted(gt[:, 0], est[:, 0]), 1, len(gt) - 1)
    i = i - ((est[:, 0] - gt[i - 1, 0]) < (gt[i, 0] - est[:, 0]))
    ok = np.abs(gt[i, 0] - est[:, 0]) <= 0.01
    return est[ok, 0], np.hypot(est[ok, 1] - gt[i[ok], 1], est[ok, 2] - gt[i[ok], 2])


def median_run(rows, key='corr_ate_rmse'):
    ordered = sorted(rows, key=lambda r: float(r[key]))
    return ordered[len(ordered) // 2]


def shade_outage(axes, start, end, label=True):
    axes.axvspan(start, end, color='#D9534F', alpha=0.12, lw=0,
                 label=OUTAGE_LABEL if label else None)


# ---------------------------------------------------------------------------------------------
def fig01(by):
    scenarios = list(SCENARIO_LABEL)
    figure, axes = plt.subplots(figsize=(10, 4.2))
    width = 0.26
    for k, est in enumerate(('odom', 'base', 'corr')):
        data = [[float(r[f'{est}_ate_rmse']) for r in by[s]] for s in scenarios]
        positions = np.arange(len(scenarios)) + (k - 1) * width
        parts = axes.boxplot(data, positions=positions, widths=width * 0.85, patch_artist=True,
                             showfliers=True, flierprops={'markersize': 3})
        for box in parts['boxes']:
            box.set(facecolor=COLOR[est], alpha=0.75, edgecolor='#333333', linewidth=0.6)
        for element in ('whiskers', 'caps', 'medians'):
            for line in parts[element]:
                line.set(color='#333333', linewidth=0.8)
        axes.plot([], [], 's', color=COLOR[est], alpha=0.75, label=LABEL[est])
    axes.axhline(0.2, color='#D9534F', linestyle='--', linewidth=1, label='поріг NFR-1, 0,2 м')
    axes.set_yscale('log')
    axes.set_xticks(np.arange(len(scenarios)))
    axes.set_xticklabels([SCENARIO_LABEL[s] for s in scenarios])
    axes.set_ylabel('ATE (RMSE), м')
    axes.legend(ncol=4, loc='upper left')
    save(figure, 'fig01_ate_boxplots', keep_x=True)


def fig02(by):
    figure, panels = plt.subplots(1, 2, figsize=(11, 4.6),
                                  gridspec_kw={'width_ratios': [1, 1.6]})
    for axes, scenario in zip(panels, ('room_loop', 'warehouse_loop')):
        d = extract(median_run(by[scenario]))
        for est in ('odom', 'base', 'corr', 'gt'):
            a = d[est]
            axes.plot(a[:, 1], a[:, 2], color=COLOR[est], label=LABEL[est],
                      linewidth=1.6 if est == 'gt' else 1.0,
                      linestyle='--' if est == 'gt' else '-')
        axes.plot(0, 0, 'o', color=COLOR['gt'], markersize=4)
        axes.set_aspect('equal')
        axes.set_xlabel('x, м')
        axes.set_ylabel('y, м')
    panels[0].legend(loc='center')
    save(figure, 'fig02_trajectories')


def fig03(by):
    row = next(r for r in by['room_long'] if r['seed'] == '1')
    d = extract(row)
    figure, axes = plt.subplots(figsize=(9, 3.4))
    for est in ('base', 'corr'):
        t, err = position_error(d[est], d['gt'])
        axes.plot(t, err, color=COLOR[est], linewidth=0.9, label=LABEL[est])
    axes.axhline(0.2, color='#D9534F', linestyle='--', linewidth=1, label='поріг NFR-1, 0,2 м')
    axes.set_xlabel('час, с')
    axes.set_ylabel('похибка положення, м')
    axes.set_ylim(bottom=0)
    axes.legend(loc='upper left')
    save(figure, 'fig03_room_long_error')


def error_curves(rows):
    """Per run: travelled distance, distance from the start and base / corr error (on gt times)."""
    curves = []
    for row in rows:
        d = extract(row)
        gt = d['gt']
        steps = np.hypot(np.diff(gt[:, 1]), np.diff(gt[:, 2]))
        travelled = np.concatenate([[0], np.cumsum(steps)])
        from_start = np.hypot(gt[:, 1], gt[:, 2])
        errors = {}
        for est in ('base', 'corr'):
            a = d[est]
            inside = (gt[:, 0] >= a[0, 0]) & (gt[:, 0] <= a[-1, 0])
            x = np.interp(gt[:, 0], a[:, 0], a[:, 1])
            y = np.interp(gt[:, 0], a[:, 0], a[:, 2])
            errors[est] = np.where(inside, np.hypot(x - gt[:, 1], y - gt[:, 2]), np.nan)
        curves.append((travelled, from_start, errors))
    return curves


def fig04(by):
    curves = error_curves(by['warehouse_loop'])
    figure, (left, right) = plt.subplots(1, 2, figsize=(11, 3.8))
    grid = np.arange(0, 178, 1.0)
    bins = np.arange(0, 31, 1.5)
    centres = (bins[:-1] + bins[1:]) / 2
    for est in ('base', 'corr'):
        along = np.array([np.interp(grid, trav[~np.isnan(e[est])], e[est][~np.isnan(e[est])])
                          for trav, _, e in curves])
        mean, std = along.mean(axis=0), along.std(axis=0, ddof=1)
        left.plot(grid, mean, color=COLOR[est], label=LABEL[est])
        left.fill_between(grid, mean - std, mean + std, color=COLOR[est], alpha=0.2, lw=0)
        per_run = []
        for _, start_dist, e in curves:
            ok = ~np.isnan(e[est])
            index = np.digitize(start_dist[ok], bins) - 1
            per_run.append([np.mean(e[est][ok][index == b]) if np.any(index == b) else np.nan
                            for b in range(len(centres))])
        per_run = np.array(per_run)
        mean, std = np.nanmean(per_run, axis=0), np.nanstd(per_run, axis=0, ddof=1)
        right.plot(centres, mean, color=COLOR[est], marker='o', markersize=3, label=LABEL[est])
        right.fill_between(centres, mean - std, mean + std, color=COLOR[est], alpha=0.2, lw=0)
    left.set_xlabel('пройдена відстань, м')
    right.set_xlabel('відстань від старту, м')
    for axes in (left, right):
        axes.set_ylabel('похибка положення, м')
        axes.set_ylim(bottom=0)
    left.legend(loc='upper left')
    save(figure, 'fig04_error_vs_distance')


def fig05(by):
    room = next(r for r in by['room_dropout'] if r['seed'] == '1')
    warehouse = next(r for r in by['warehouse_dropout'] if r['seed'] == '1')
    figure, panels = plt.subplots(2, 1, figsize=(9, 6))
    for axes, row in zip(panels, (room, warehouse)):
        d = extract(row)
        for est in ('base', 'corr'):
            t, err = position_error(d[est], d['gt'])
            axes.plot(t, err, color=COLOR[est], linewidth=0.9, label=LABEL[est])
        shade_outage(axes, float(row['drop_start']), float(row['drop_end']))
        axes.set_ylabel('похибка положення, м')
    panels[0].set_ylim(bottom=0)
    panels[1].set_yscale('log')
    panels[1].set_ylim(0.005, 30)
    panels[1].set_xlabel('час, с')
    panels[0].legend(loc='upper left')
    save(figure, 'fig05_dropout')


def fig06(by):
    row = next(r for r in by['warehouse_dropout'] if r['seed'] == '1')
    grid, xs, ys = read_bag(REPO / 'bags' / row['bag'], True)
    info = grid.info
    cells = np.array(grid.data, dtype=np.int16).reshape(info.height, info.width)
    image = np.full(cells.shape, 0.80)
    known = cells >= 0
    image[known] = 1.0 - cells[known] / 100.0
    x0, y0 = info.origin.position.x, info.origin.position.y
    extent = [x0, x0 + info.width * info.resolution, y0, y0 + info.height * info.resolution]
    figure, axes = plt.subplots(figsize=(9, 6.5))
    axes.imshow(image, cmap='gray', vmin=0.0, vmax=1.0, origin='lower', extent=extent)
    axes.plot(xs, ys, color=COLOR['gt'], linewidth=0.8, linestyle='--', label=LABEL['gt'])
    axes.set_xlabel('x, м')
    axes.set_ylabel('y, м')
    axes.grid(False)
    axes.legend(loc='upper right')
    save(figure, 'fig06_rotated_map')


def fig07(by):
    figure, axes = plt.subplots(figsize=(9, 3.6))
    switched = [r for r in by['warehouse_recovery'] if r['switch_t']]
    # All lines are the corrected estimate: shades of its colour, told apart by line style.
    shades = plt.get_cmap('Oranges')(np.linspace(0.45, 0.95, len(switched)))
    styles = ['-', '--', '-.', ':', '-']
    for k, row in enumerate(switched):
        d = extract(row)
        t, err = position_error(d['corr'], d['gt'])
        switch = float(row['switch_t'])
        after = t >= switch
        axes.plot(t[after] - switch, err[after], linewidth=1.1, color=shades[k],
                  linestyle=styles[k % len(styles)], label=f'сід {row["seed"]}')
    axes.axhline(0.1, color=COLOR['gt'], linestyle=(0, (1, 2)), linewidth=1, label='0,1 м')
    axes.set_xlabel('час після перемикання, с')
    axes.set_ylabel('похибка скоригованої оцінки, м')
    axes.set_ylim(bottom=0)
    axes.legend(ncol=5, loc='upper right')
    save(figure, 'fig07_recovery')


def read_pgm_map(name):
    meta = yaml.safe_load((REPO / 'maps' / f'{name}.yaml').read_text())
    image = plt.imread(REPO / 'maps' / meta['image']) if (REPO / 'maps' / meta['image']).exists() \
        else plt.imread(REPO / 'maps' / f'{name}.pgm')
    height, width = image.shape[:2]
    x0, y0 = meta['origin'][:2]
    r = meta['resolution']
    return image, [x0, x0 + width * r, y0, y0 + height * r]


def fig08():
    figure, panels = plt.subplots(1, 2, figsize=(11, 4.4),
                                  gridspec_kw={'width_ratios': [1, 2.4]})
    for axes, name in zip(panels, ('final_room_s1', 'final_wh_s1')):
        image, extent = read_pgm_map(name)
        axes.imshow(image, cmap='gray', extent=extent)
        axes.plot(0, 0, 'o', color=COLOR['corr'], markersize=4, label='старт')
        axes.set_xlabel('x, м')
        axes.set_ylabel('y, м')
        axes.grid(False)
    panels[0].legend(loc='upper right')
    save(figure, 'fig08_maps')


def fig09(rows):
    processes = ['ekf_filter_node', 'async_slam_toolbox_node', 'localization_slam_toolbox_node',
                 'sensor_monitor_node', 'map_manager_node']
    cpu, rss = defaultdict(list), defaultdict(list)
    for row in rows:
        per = defaultdict(list)
        with open(REPO / 'bags' / row['bag'] / 'resources.csv') as f:
            samples = list(csv.DictReader(f))
        first = samples[0]['wall_time'] if samples else None
        for s in samples:
            if s['process'] in processes and s['wall_time'] != first:
                per[s['process']].append((float(s['cpu_core_pct']), float(s['rss_mb'])))
        for p, values in per.items():
            cpu[p].append(np.mean([v[0] for v in values]))
            rss[p].append(np.max([v[1] for v in values]))
    present = [p for p in processes if cpu[p]]
    names = [p.replace('_node', '').replace('_slam_toolbox', '\nslam_toolbox') for p in present]
    core = np.array([np.mean(cpu[p]) for p in present])
    figure, (a1, a2, a3) = plt.subplots(1, 3, figsize=(12, 3.8))
    for axes, values, label in ((a1, core, 'ЦП, % одного ядра'),
                                (a2, core / 8, 'ЦП, % восьми ядер'),
                                (a3, np.array([np.mean(rss[p]) for p in present]),
                                 "пам'ять (RSS), МіБ")):
        axes.bar(range(len(present)), values, color=COLOR['base'], alpha=0.8)
        axes.set_xticks(range(len(present)))
        axes.set_xticklabels(names, rotation=35, ha='right', fontsize=8)
        axes.set_ylabel(label)
        for k, v in enumerate(values):
            axes.text(k, v, f'{v:.1f}'.replace('.', ','), ha='center', va='bottom', fontsize=8)
    save(figure, 'fig09_resources', keep_x=True)


def fig10(rows):
    per_seed = defaultdict(list)
    for r in rows:
        per_seed[int(r['seed'])].append(float(r['gyro_bias']))
    bias = np.array([np.mean(v) for v in per_seed.values()]) * 1e5
    figure, axes = plt.subplots(figsize=(7, 3.6))
    bins = np.arange(-25, 26, 5)
    axes.hist(bias, bins=bins, color=COLOR['base'], alpha=0.75, edgecolor='white',
              label=f'оцінки за сідами (n = {len(bias)})')
    x = np.linspace(-30, 30, 300)
    axes.plot(x, stats.norm.pdf(x, 0, 10) * len(bias) * 5, color=COLOR['corr'],
              label='модель N(0; 10⁻⁴ рад/с)')
    axes.set_xlabel('зміщення гіроскопа, 10⁻⁵ рад/с')
    axes.set_ylabel('кількість сідів')
    axes.legend(loc='upper left')
    save(figure, 'fig10_gyro_bias')


def main():
    rows, by = load_runs()
    fig01(by)
    fig02(by)
    fig03(by)
    fig04(by)
    fig05(by)
    fig06(by)
    fig07(by)
    fig08()
    fig09(rows)
    fig10(rows)


if __name__ == '__main__':
    main()
