#!/usr/bin/env python3
"""이전 결과(results/pre_v2) 와 현재 결과(results) 비교: 프로파일 × 출현거리별 충돌률과 평균 최소 이격.
사용: python3 scripts/compare_runs.py [이름 ...]   (기본: E2_doorway E2b_doorway_stop E1_bus_exit E3_car_reverse E4_rear_approach)
"""
import csv, collections, os, sys

RES = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'results')


def agg(path, rear):
    g = collections.defaultdict(list)
    for r in csv.DictReader(open(path)):
        if r.get('emerged', '1') != '1' and not rear:
            continue
        key = (r['profile'], float(r['robot_l']) if rear else None, float(r['v_ped']) if rear else None, float(r['d_app']))
        g[key].append((int(r['collision']), float(r['min_clear']) if r['min_clear'] else None))
    out = {}
    for k, v in g.items():
        mc = [x[1] for x in v if x[1] is not None]
        out[k] = (100 * sum(x[0] for x in v) / len(v), sum(mc) / len(mc) if mc else float('nan'), len(v))
    return out


def main():
    args = sys.argv[1:]; old_dir = 'pre_v2'
    if args and args[0].startswith('--old='):
        old_dir = args.pop(0)[6:]
    names = args or ['E2_doorway', 'E2b_doorway_stop', 'E1_bus_exit', 'E3_car_reverse', 'E4_rear_approach']
    for n in names:
        old_p, new_p = os.path.join(RES, old_dir, n + '.csv'), os.path.join(RES, n + '.csv')
        if not (os.path.exists(old_p) and os.path.exists(new_p)):
            print(f'== {n}: 파일 없음'); continue
        rear = n.startswith('E4')
        old, new = agg(old_p, rear), agg(new_p, rear)
        print(f'== {n}   (충돌률 % / 평균 최소 이격 m : {old_dir} → 현재)')
        changed = 0
        for k in sorted(set(old) | set(new), key=lambda k: (k[0], k[1] or 0, k[2] or 0, k[3])):
            o, w = old.get(k), new.get(k)
            if not o or not w:
                continue
            flag = ' *' if abs(o[0] - w[0]) >= 1 or abs(o[1] - w[1]) > 0.05 else ''
            changed += bool(flag)
            lab = f"{k[0]:15s}" + (f" l={k[1]:g} v={k[2]:g}" if rear else '') + f" d={k[3]:g}"
            print(f"  {lab:34s} {o[0]:5.0f} → {w[0]:5.0f}   {o[1]:+.2f} → {w[1]:+.2f}{flag}")
        print(f'  변화(* 충돌률 차 또는 이격 차 > 0.05 m): {changed}개 셀')


if __name__ == '__main__':
    main()
