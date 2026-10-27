#!/usr/bin/env python3
"""
Step 7, stage 1: read every bag of the final series ONCE and keep what the evaluation needs.

Reading a 15-minute bag takes about a minute (mostly /tf and the IMU), and
the metrics, the statistics and the figures all need the same data. So this
script reads each bag once and writes a compact copy:

  results/final_work/runs/<bag>.npz    arrays (all times in seconds of SIMULATION time)
      gt        t, x, y, yaw                 /ground_truth (world frame = map frame at the start)
      odom      t, x, y, yaw                 /odom (wheel odometry with noise)
      base      t, x, y, yaw, cxx, cxy, cyy, cyawyaw   /odometry/filtered + its pose covariance
      corr      t, x, y, yaw                 TF map -> base_link (corrected estimate)
      base_recv receive time (wall clock) of every /odometry/filtered message
      scan      t_stamp, t_received_sim      /scan header stamp, receive time as sim time
      m2o       t_stamp, t_received_sim      every NEW map -> odom stamp and when it arrived
  results/final_work/runs/<bag>.json   scalars and events
      gyro_bias (rad/s), rtf, events of /diagnostics, /correction_valid, /map_manager/events

The corrected estimate is map -> odom composed with odom -> base_link, evaluated
at the stamps of odom -> base_link, with the rules of tf_to_tum.py: the latest
sample per stamp is kept, inside a gap > 1 s map -> odom is HELD (slam_toolbox
re-sends a frozen value while the lidar is off), otherwise it is interpolated.
Here it is done in 2D (x, y, yaw), vectorised; the robot is planar.

Event times: /diagnostics, /correction_valid and /map_manager/events carry no
simulation stamp, so their RECEIVE time (bag time, wall clock) is converted to
simulation time with the receive-time -> stamp pairs of /odometry/filtered
(30 Hz, interpolated).

The gyro bias is estimated as in estimate_gyro_bias.py:
  bias = (integral of w_gyro dt - true heading change) / T.

Run with the Python of evo's venv (rosbags, numpy, scipy; no ROS needed):
  ~/.venvs/evo/bin/python src/gnssd_tools/scripts/final_extract.py \
      [--jobs 6] [--tag experiments-v1]
Bags already extracted are skipped (delete the .npz to redo one).
"""

import argparse
import csv
import json
import math
from multiprocessing import Pool
from pathlib import Path
import subprocess

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores

REPO = Path(__file__).resolve().parents[3]
OUT = REPO / 'results' / 'final_work' / 'runs'
MAX_INTERPOLATION_GAP = 1.0  # s, as in tf_to_tum.py

TOPICS = ('/ground_truth', '/odom', '/odometry/filtered', '/tf', '/imu_raw', '/scan',
          '/diagnostics', '/correction_valid', '/map_manager/events')


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pose_row(t, pose):
    return (t, pose.position.x, pose.position.y, yaw_of(pose.orientation))


def latest_per_stamp(samples):
    """Sort (t, ...) samples by t, keeping the LAST sample of every repeated stamp."""
    by_stamp = {}
    for sample in samples:
        by_stamp[sample[0]] = sample
    return np.array(sorted(by_stamp.values()))


def corrected_trajectory(map_odom, odom_base):
    """
    Compose map -> odom with odom -> base_link at the odom -> base_link stamps.

    Both inputs are arrays of rows (t, x, y, yaw). map -> odom is interpolated
    linearly (yaw along the shorter way) except inside gaps longer than
    MAX_INTERPOLATION_GAP, where its last value is held; stamps outside its
    recorded span are dropped (no extrapolation), as in tf_to_tum.py.
    """
    t = odom_base[:, 0]
    mt = map_odom[:, 0]
    keep = (t >= mt[0]) & (t <= mt[-1])
    t, ob = t[keep], odom_base[keep]
    i = np.clip(np.searchsorted(mt, t, side='right') - 1, 0, len(mt) - 1)
    j = np.minimum(i + 1, len(mt) - 1)
    gap = mt[j] - mt[i]
    w = np.where((gap > 0) & (gap <= MAX_INTERPOLATION_GAP),
                 (t - mt[i]) / np.where(gap > 0, gap, 1.0), 0.0)
    mx = map_odom[i, 1] + w * (map_odom[j, 1] - map_odom[i, 1])
    my = map_odom[i, 2] + w * (map_odom[j, 2] - map_odom[i, 2])
    dyaw = np.arctan2(np.sin(map_odom[j, 3] - map_odom[i, 3]),
                      np.cos(map_odom[j, 3] - map_odom[i, 3]))
    myaw = map_odom[i, 3] + w * dyaw
    x = mx + np.cos(myaw) * ob[:, 1] - np.sin(myaw) * ob[:, 2]
    y = my + np.sin(myaw) * ob[:, 1] + np.cos(myaw) * ob[:, 2]
    yaw = np.arctan2(np.sin(myaw + ob[:, 3]), np.cos(myaw + ob[:, 3]))
    return np.column_stack([t, x, y, yaw])


def gyro_bias(gyro, gt):
    """(integral of gyro z - true heading change) / T over the common span."""
    true_t, true_yaw = gt[:, 0], np.unwrap(gt[:, 3])
    start, end = max(gyro[0, 0], true_t[0]), min(gyro[-1, 0], true_t[-1])
    inside = (gyro[:, 0] >= start) & (gyro[:, 0] <= end)
    t, w = gyro[inside, 0], gyro[inside, 1]
    measured = np.trapezoid(w, t)
    true_change = np.interp(t[-1], true_t, true_yaw) - np.interp(t[0], true_t, true_yaw)
    return float((measured - true_change) / (t[-1] - t[0]))


def extract(bag_name):
    """Read one bag, write <bag>.npz and <bag>.json; return a short status line."""
    bag = REPO / 'bags' / bag_name
    out_npz, out_json = OUT / f'{bag_name}.npz', OUT / f'{bag_name}.json'
    if out_npz.exists() and out_json.exists():
        return f'{bag_name}: already extracted'

    gt, odom, base, base_recv, gyro, scan = [], [], [], [], [], []
    map_odom, odom_base = [], []
    m2o_new = []                 # (stamp, receive time) of every new map -> odom stamp
    seen_m2o = set()
    raw_events = []              # (receive time, kind, data)
    with AnyReader([bag], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        connections = [c for c in reader.connections if c.topic in TOPICS]
        for connection, recv_ns, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, connection.msgtype)
            recv = recv_ns * 1e-9
            topic = connection.topic
            if topic == '/ground_truth':
                gt.append(pose_row(stamp(msg.header), msg.pose.pose))
            elif topic == '/odom':
                odom.append(pose_row(stamp(msg.header), msg.pose.pose))
            elif topic == '/odometry/filtered':
                c = msg.pose.covariance
                base.append(pose_row(stamp(msg.header), msg.pose.pose) + (c[0], c[1], c[7], c[35]))
                base_recv.append(recv)
            elif topic == '/imu_raw':
                gyro.append((stamp(msg.header), msg.angular_velocity.z))
            elif topic == '/scan':
                scan.append((stamp(msg.header), recv))
            elif topic == '/tf':
                for tr in msg.transforms:
                    edge = (tr.header.frame_id, tr.child_frame_id)
                    if edge not in (('map', 'odom'), ('odom', 'base_link')):
                        continue
                    s = stamp(tr.header)
                    row = (s, tr.transform.translation.x, tr.transform.translation.y,
                           yaw_of(tr.transform.rotation))
                    if edge == ('map', 'odom'):
                        map_odom.append(row)
                        if s not in seen_m2o:
                            seen_m2o.add(s)
                            m2o_new.append((s, recv))
                    else:
                        odom_base.append(row)
            elif topic == '/diagnostics':
                for status in msg.status:
                    if status.name.startswith('sensor_monitor'):
                        raw_events.append((recv, 'diag', [status.name, int(status.level),
                                                          status.message]))
            elif topic == '/correction_valid':
                raw_events.append((recv, 'valid', bool(msg.data)))
            else:
                raw_events.append((recv, 'event', msg.data))

    base = np.array(sorted(base))
    base_recv = np.sort(np.array(base_recv))
    order = np.argsort(base[:, 0])
    recv_to_sim = (np.array(base_recv), base[order, 0])   # both increase together

    def to_sim(recv):
        return float(np.interp(recv, recv_to_sim[0], recv_to_sim[1]))

    gt = latest_per_stamp(gt)
    corr = (corrected_trajectory(latest_per_stamp(map_odom), latest_per_stamp(odom_base))
            if map_odom else np.zeros((0, 4)))
    scan = np.array(scan)
    m2o = np.array(m2o_new)

    events = []
    last_diag = {}
    for recv, kind, data in sorted(raw_events, key=lambda e: e[0]):
        if kind == 'diag':
            name, level, message = data
            if last_diag.get(name) == level:
                continue                     # report state CHANGES only
            last_diag[name] = level
            events.append({'t': to_sim(recv), 'kind': 'diag', 'name': name,
                           'level': level, 'message': message})
        elif kind == 'valid':
            events.append({'t': to_sim(recv), 'kind': 'valid', 'value': data})
        else:
            events.append({'t': to_sim(recv), 'kind': 'event', 'text': data})

    np.savez_compressed(
        out_npz, gt=gt, odom=latest_per_stamp(odom), base=base, base_recv=base_recv, corr=corr,
        scan=np.column_stack([scan[:, 0], [to_sim(r) for r in scan[:, 1]]]),
        m2o=(np.column_stack([m2o[:, 0], [to_sim(r) for r in m2o[:, 1]]])
             if len(m2o) else np.zeros((0, 2))))
    rtf = (base[-1, 0] - base[0, 0]) / (base_recv[-1] - base_recv[0])
    info = {'bag': bag_name, 'gyro_bias': gyro_bias(np.array(sorted(gyro)), gt),
            'rtf': float(rtf), 'events': events}
    out_json.write_text(json.dumps(info, indent=1))
    return f'{bag_name}: {len(gt)} gt, {len(base)} base, {len(corr)} corrected, ' \
           f'{len(events)} events, rtf {rtf:.3f}'


def final_bags(tag):
    """Bags of bags/index.csv recorded at the commit of the given git tag."""
    commit = subprocess.run(['git', '-C', str(REPO), 'rev-parse', f'{tag}^{{commit}}'],
                            capture_output=True, text=True, check=True).stdout.strip()
    with open(REPO / 'bags' / 'index.csv') as f:
        return [row['bag'] for row in csv.DictReader(f) if row['commit'] == commit]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('--tag', default='experiments-v1')
    parser.add_argument('--jobs', type=int, default=6)
    parser.add_argument('--only', nargs='*', help='extract only these bags')
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    bags = args.only or final_bags(args.tag)
    print(f'{len(bags)} bags')
    with Pool(args.jobs) as pool:
        for line in pool.imap_unordered(extract, bags):
            print(line, flush=True)


if __name__ == '__main__':
    main()
