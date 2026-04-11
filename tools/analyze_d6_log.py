#!/usr/bin/env python3
"""Analyze SixDofJoint solver logs to find instability patterns.

Usage:
    cargo run -p avian3d --example six_dof_ragdoll_3d 2>&1 > /tmp/ragdoll_log.txt
    python3 tools/analyze_d6_log.py /tmp/ragdoll_log.txt
"""

import sys
import re
from collections import defaultdict

def parse_log(path):
    steps = []
    lin_combs = []
    ang_locks = []

    with open(path) as f:
        for line in f:
            line = line.strip()

            # D6_STEP: per-joint-per-substep summary
            m = re.search(r'D6_STEP b1d=([\d.]+) b2d=([\d.]+) sep=\(([-\d.]+),([-\d.]+),([-\d.]+)\) \|sep\|=([\d.]+)', line)
            if m:
                ts = extract_timestamp(line)
                steps.append({
                    'ts': ts,
                    'b1d': float(m.group(1)),
                    'b2d': float(m.group(2)),
                    'sep': (float(m.group(3)), float(m.group(4)), float(m.group(5))),
                    'sep_mag': float(m.group(6)),
                })
                continue

            # LIN_COMB: combined linear solver
            m = re.search(r'LIN_COMB \|sep\|=([\d.]+) dl=([-\d.e+]+) w=\(([\d.]+),([\d.]+)\) \|imp\|=([\d.]+)', line)
            if m:
                ts = extract_timestamp(line)
                lin_combs.append({
                    'ts': ts,
                    'sep_mag': float(m.group(1)),
                    'dl': float(m.group(2)),
                    'w1': float(m.group(3)),
                    'w2': float(m.group(4)),
                    'imp_mag': float(m.group(5)),
                })
                continue

    return steps, lin_combs, ang_locks


def extract_timestamp(line):
    m = re.search(r'(\d{4}-\d{2}-\d{2}T[\d:.]+)Z', line)
    return m.group(1) if m else ''


def analyze(path):
    steps, lin_combs, ang_locks = parse_log(path)

    print(f"=== D6 Joint Log Analysis ===")
    print(f"File: {path}")
    print(f"D6_STEP entries: {len(steps)}")
    print(f"LIN_COMB entries: {len(lin_combs)}")
    print()

    if steps:
        sep_mags = [s['sep_mag'] for s in steps]
        b1ds = [s['b1d'] for s in steps]
        b2ds = [s['b2d'] for s in steps]
        print(f"--- D6_STEP (post-solve residual separation) ---")
        print(f"  |sep| max={max(sep_mags):.4f}  mean={sum(sep_mags)/len(sep_mags):.4f}")
        print(f"  b1_delta max={max(b1ds):.4f}  mean={sum(b1ds)/len(b1ds):.4f}")
        print(f"  b2_delta max={max(b2ds):.4f}  mean={sum(b2ds)/len(b2ds):.4f}")
        print()

        # Find when things blow up
        print(f"  Largest post-solve separations:")
        sorted_steps = sorted(steps, key=lambda s: s['sep_mag'], reverse=True)
        for s in sorted_steps[:5]:
            print(f"    ts={s['ts']}  |sep|={s['sep_mag']:.4f}  sep={s['sep']}")
        print()

        # Timeline: group by timestamp second and show max sep
        by_second = defaultdict(list)
        for s in steps:
            sec = s['ts'][:19] if s['ts'] else 'unknown'
            by_second[sec].append(s['sep_mag'])
        print(f"  Timeline (max |sep| per second):")
        for sec in sorted(by_second.keys()):
            vals = by_second[sec]
            print(f"    {sec}  max={max(vals):.4f}  count={len(vals)}")
        print()

    if lin_combs:
        sep_mags = [l['sep_mag'] for l in lin_combs]
        print(f"--- LIN_COMB (combined linear solver) ---")
        print(f"  |sep| max={max(sep_mags):.4f}  mean={sum(sep_mags)/len(sep_mags):.4f}")
        print()

        # Check if dl is consistently small despite large sep
        big_sep = [l for l in lin_combs if l['sep_mag'] > 0.5]
        if big_sep:
            print(f"  Entries with |sep| > 0.5: {len(big_sep)}")
            dls = [abs(l['dl']) for l in big_sep]
            ws = [l['w1'] + l['w2'] for l in big_sep]
            print(f"    |dl| range: {min(dls):.6f} - {max(dls):.6f}")
            print(f"    w_sum range: {min(ws):.1f} - {max(ws):.1f}")
            # Check correction ratio: dl * w should ≈ sep
            ratios = [abs(l['dl']) * (l['w1'] + l['w2']) / l['sep_mag'] for l in big_sep if l['sep_mag'] > 0]
            if ratios:
                print(f"    correction ratio (|dl*w|/|sep|): {min(ratios):.3f} - {max(ratios):.3f}  (should be ~1.0)")
            print()

    if not steps and not lin_combs:
        print("No D6 log entries found. Make sure to move the pelvis to trigger warnings.")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <logfile>")
        sys.exit(1)
    analyze(sys.argv[1])
