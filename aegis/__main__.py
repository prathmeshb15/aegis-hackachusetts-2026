"""Command-line benchmark.

Original two-way (Basic vs AEGIS) experiment, unchanged:
    python -m aegis --start 1000 --count 30 [--types crossing,noisy] [--json]
Three-way experiment (Basic vs Reactive vs AEGIS):
    python -m aegis --three-way --start 1000 --count 30 [--json]
"""
from __future__ import annotations

import argparse
import json
import time

from .engine import benchmark, benchmark3
from .scenarios import SCENARIO_TYPES


def _fmt(row):
    b, a, p = row['baseline'], row['aegis'], row['paired']
    n = row['pairs']
    t = a['mean_time_completed']
    return (f"| {row['scenario']} | {row['category']} | {b['collisions']}/{n} | {a['collisions']}/{n} | "
            f"{b['completed']}/{n} | {a['completed']}/{n} | {a['timeouts']} | {a['safe_stops_no_path']} | "
            f"{p['aegis_safe_where_baseline_collided']} | {p['both_collided']} | {p['aegis_only_collided']} | "
            f"{'-' if t is None else t} |")


def _d(x):
    return '-' if x is None else x


def _print3(res):
    print(res['method'])
    cs = res['controllers']
    print('| Scenario | Cat. | ' + ' | '.join(f'{c} coll.' for c in cs) + ' | ' +
          ' | '.join(f'{c} done' for c in cs) + ' | ' + ' | '.join(f'{c} time' for c in cs) + ' | ' +
          ' | '.join(f'{c} stops' for c in cs) + ' | R-only coll. | A-only coll. |')
    print('|' + '---|' * (2 + 4 * len(cs) + 2))
    for r in res['results']:
        n = r['episodes']
        rva = r.get('reactive_vs_aegis', {})
        print(f"| {r['scenario']} | {r['category']} | " +
              ' | '.join(f"{r[c]['collisions']}/{n}" for c in cs) + ' | ' +
              ' | '.join(f"{r[c]['completed']}/{n}" for c in cs) + ' | ' +
              ' | '.join(f"{_d(r[c]['mean_time_completed'])}" for c in cs) + ' | ' +
              ' | '.join(f"{_d(r[c]['mean_stop_events'])}" for c in cs) + ' | ' +
              f"{rva.get('reactive_only_collided', '-')} | {rva.get('aegis_only_collided', '-')} |")
    print('\nTotals:')
    print(json.dumps(res['totals'], indent=1))


def main():
    ap = argparse.ArgumentParser(description='Run the paired AEGIS benchmarks.')
    ap.add_argument('--start', type=int, default=1000)
    ap.add_argument('--count', type=int, default=20)
    ap.add_argument('--types', default=','.join(SCENARIO_TYPES))
    ap.add_argument('--three-way', action='store_true', help='Basic vs Reactive vs AEGIS')
    ap.add_argument('--json', action='store_true')
    args = ap.parse_args()
    t0 = time.perf_counter()
    seeds, types = range(args.start, args.start + args.count), tuple(args.types.split(','))
    if args.three_way:
        res = benchmark3(seeds, types)
        if args.json:
            print(json.dumps(res, indent=1))
        else:
            _print3(res)
            print(f'Runtime {time.perf_counter() - t0:.1f} s.')
        return
    res = benchmark(seeds, types)
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
