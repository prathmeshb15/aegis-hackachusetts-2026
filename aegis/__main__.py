"""Command-line benchmark: ``python -m aegis --start 1000 --count 20 [--types crossing,noisy] [--json]``."""
from __future__ import annotations

import argparse
import json
import time

from .engine import benchmark
from .scenarios import SCENARIO_TYPES


def _fmt(row):
    b, a, p = row['baseline'], row['aegis'], row['paired']
    n = row['pairs']
    t = a['mean_time_completed']
    return (f"| {row['scenario']} | {row['category']} | {b['collisions']}/{n} | {a['collisions']}/{n} | "
            f"{b['completed']}/{n} | {a['completed']}/{n} | {a['timeouts']} | {a['safe_stops_no_path']} | "
            f"{p['aegis_safe_where_baseline_collided']} | {p['both_collided']} | {p['aegis_only_collided']} | "
            f"{'-' if t is None else t} |")


def main():
    ap = argparse.ArgumentParser(description='Run the paired AEGIS vs baseline benchmark.')
    ap.add_argument('--start', type=int, default=1000)
    ap.add_argument('--count', type=int, default=20)
    ap.add_argument('--types', default=','.join(SCENARIO_TYPES))
    ap.add_argument('--json', action='store_true')
    args = ap.parse_args()
    t0 = time.perf_counter()
    res = benchmark(range(args.start, args.start + args.count), tuple(args.types.split(',')))
    dt = time.perf_counter() - t0
    if args.json:
        print(json.dumps(res, indent=1))
        return
    print(res['method'])
    print('| Scenario | Category | Base coll. | AEGIS coll. | Base done | AEGIS done | AEGIS timeout | '
          'AEGIS safe stop (no path) | AEGIS saved | Both coll. | AEGIS-only coll. | AEGIS mean time (s) |')
    print('|' + '---|' * 12)
    for row in res['results']:
        print(_fmt(row))
    tot = res['totals']
    print(f"Totals over {tot['pairs']} pairs: baseline collisions {tot['baseline_collisions']}, "
          f"AEGIS collisions {tot['aegis_collisions']}, baseline completed {tot['baseline_completed']}, "
          f"AEGIS completed {tot['aegis_completed']}, AEGIS-only collisions {tot['aegis_only_collided']}. "
          f"Runtime {dt:.1f} s.")


if __name__ == '__main__':
    main()
