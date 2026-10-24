#!/usr/bin/env python3
"""
Summarise <bag>/resources.csv: mean and max CPU and RSS per process and for the system.

"system" is the sum over all logged processes at the same sample time
(e.g. ekf_filter_node + async_slam_toolbox_node).
The first sample of each process is skipped: psutil's CPU % needs one
interval before it is valid.

Usage:
  python3 src/gnssd_tools/scripts/summarize_resources.py bags/<bag>/resources.csv
"""

import argparse
import csv
from collections import defaultdict


def mean(values):
    return sum(values) / len(values)


def main():
    parser = argparse.ArgumentParser(description='Summarise resources.csv')
    parser.add_argument('csv_file')
    args = parser.parse_args()

    per_process = defaultdict(lambda: {'core': [], 'total': [], 'rss': []})
    per_sample = defaultdict(lambda: {'core': 0.0, 'total': 0.0, 'rss': 0.0})
    seen = set()
    with open(args.csv_file) as f:
        for row in csv.DictReader(f):
            name = row['process']
            if name not in seen:   # skip the warm-up sample
                seen.add(name)
                continue
            core, total, rss = (float(row['cpu_core_pct']), float(row['cpu_total_pct']),
                                float(row['rss_mb']))
            per_process[name]['core'].append(core)
            per_process[name]['total'].append(total)
            per_process[name]['rss'].append(rss)
            sample = per_sample[row['wall_time']]
            sample['core'] += core
            sample['total'] += total
            sample['rss'] += rss

    system = {'core': [s['core'] for s in per_sample.values()],
              'total': [s['total'] for s in per_sample.values()],
              'rss': [s['rss'] for s in per_sample.values()]}

    print(f'{"process":<26}{"samples":>8}{"CPU 1-core mean/max %":>24}'
          f'{"CPU all-cores mean/max %":>27}{"RSS mean/max MiB":>20}')
    for name, values in list(per_process.items()) + [('system (sum)', system)]:
        print(f'{name:<26}{len(values["core"]):>8}'
              f'{mean(values["core"]):>14.1f} / {max(values["core"]):<7.1f}'
              f'{mean(values["total"]):>17.2f} / {max(values["total"]):<7.2f}'
              f'{mean(values["rss"]):>12.1f} / {max(values["rss"]):<7.1f}')


if __name__ == '__main__':
    main()
