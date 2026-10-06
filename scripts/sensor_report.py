#!/usr/bin/env python3
"""E6 센서 배치 시험 결과 정리 → results/sensor/sensor_report.png, results/sensor/summary.csv"""
import csv, collections, json, os
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.family'] = 'NanumSquare'; plt.rcParams['axes.unicode_minus'] = False
PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); RES = os.path.join(PKG, 'results', 'sensor')
INK, INK2, MUTED, GRID, AXIS = '#0b0b0b', '#52514e', '#898781', '#e1e0d9', '#c3c2b7'
C_L, C_R, C_SYS = '#2a78d6', '#eb6834', '#1baf7a'


def rows(name):
    p = os.path.join(RES, name)
    return list(csv.DictReader(open(p))) if os.path.exists(p) else []


def style(ax, title, ylabel):
    ax.set_title(title, fontsize=11, color=INK, loc='left', fontweight='bold')
    ax.set_ylabel(ylabel, fontsize=9, color=INK2); ax.grid(color=GRID, lw=0.6, axis='y'); ax.set_axisbelow(True)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(AXIS)
    ax.tick_params(colors=MUTED, labelsize=8.5)


def main():
    cands = json.load(open(os.path.join(RES, 'candidates.json')))
    geo = {r['cand']: r for r in csv.DictReader(open(os.path.join(RES, 'geometry.csv')))}
    cams = [k for k, v in cands.items() if v['kind'] == 'cam']; lids = [k for k, v in cands.items() if v['kind'] == 'lidar']
    summ = []
    for k in cams:
        door = rows(f'{k}_door.csv'); curb = rows(f'{k}_curb.csv'); rev = rows(f'{k}_rev.csv')
        def coll(rs, side=None):
            rs = [r for r in rs if (side is None or r.get('variant') == side) and r.get('emerged', '1') == '1']
            return 100 * sum(int(r['collision']) for r in rs) / len(rs) if rs else float('nan')
        def clear(rs, side, d):
            m = [float(r['min_clear']) for r in rs if r.get('variant') == side and float(r['d_app']) == d and r['min_clear']]
            return sum(m) / len(m) if m else float('nan')
        cres = collections.defaultdict(list)
        for r in curb:
            cres[r['variant']].append(r)
        ok3 = sum(r['result'] == 'crossed' for r in cres['h3']); n3 = len(cres['h3'])
        stop58 = sum(r['result'] == 'stopped' for r in cres['h5'] + cres['h8']); n58 = len(cres['h5'] + cres['h8'])
        gaps = [float(r['stop_gap']) for r in cres['h5'] + cres['h8'] if r['stop_gap']]
        summ.append(dict(cand=k, desc=cands[k]['desc'], ground_near=geo[k]['ground_near_m'], corridor_full=geo[k]['corridor_full_m'],
                         door_left=round(coll(door, 'left')), door_right=round(coll(door, 'right')),
                         clear_left_3m=round(clear(door, 'left', 3.0), 2), clear_right_3m=round(clear(door, 'right', 3.0), 2),
                         curb3_cross=f'{ok3}/{n3}', curb58_stop=f'{stop58}/{n58}', curb_gap=round(sum(gaps) / len(gaps), 2) if gaps else '',
                         rev=round(coll(rev))))
    lsum = []
    for k in lids:
        rr = rows(f'{k}_rear.csv')
        g = {(float(r['v_ped']), float(r['d_app'])): r for r in rr}
        lsum.append(dict(cand=k, desc=cands[k]['desc'], **{f'v{int(v)}_G{int(d)}': (round(float(g[(v, d)]['min_clear']), 2) if (v, d) in g and g[(v, d)]['min_clear'] else '')
                                                           for v in (5.0, 7.0) for d in (6.0, 10.0)},
                         det=round(sum(float(r['d_det']) for r in rr if r['d_det']) / max(1, sum(1 for r in rr if r['d_det'])), 2)))
    with open(os.path.join(RES, 'summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(summ[0].keys())); w.writeheader(); w.writerows(summ)
    with open(os.path.join(RES, 'summary_lidar.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(lsum[0].keys())); w.writeheader(); w.writerows(lsum)

    fig = plt.figure(figsize=(16.5, 10.5), dpi=140)
    gs = fig.add_gridspec(2, 3, height_ratios=[1, 1], hspace=0.55, wspace=0.28, left=0.05, right=0.98, top=0.86, bottom=0.17)
    fig.text(0.05, 0.945, 'E6 센서 배치 시험 — 전방 깊이 카메라 6안 · 후방 라이다 5안', fontsize=17, fontweight='bold', color=INK)
    fig.text(0.05, 0.915, '카메라: 출입구 보행자(왼쪽/오른쪽 출현, 출현 1~3 m) · 턱 3/5/8 cm · 주차 줄 사이 후진 차량(출현 1~3 m), 시스템 프로파일, 점당 2회.  '
             '라이다: 이면도로 뒤차 5·7 m/s, 간격 6·10 m.', fontsize=9.5, color=INK2)
    labels = [f"{s['cand']}\n{s['desc'].split(':')[-1].strip()}" for s in summ]
    x = range(len(summ)); w = 0.38
    ax = fig.add_subplot(gs[0, 0])
    ax.bar([i - w / 2 for i in x], [s['door_left'] for s in summ], w * 0.95, color=C_L, label='왼쪽에서 출현')
    ax.bar([i + w / 2 for i in x], [s['door_right'] for s in summ], w * 0.95, color=C_R, label='오른쪽에서 출현')
    style(ax, '출입구 보행자 — 충돌률 (출현 1~3 m 평균)', '충돌률 [%]'); ax.set_xticks(list(x)); ax.set_xticklabels([s['cand'] for s in summ]); ax.set_ylim(0, 100); ax.legend(fontsize=8, frameon=False, loc='upper center', ncol=2)
    ax = fig.add_subplot(gs[0, 1])
    ax.bar([i - w / 2 for i in x], [s['clear_left_3m'] for s in summ], w * 0.95, color=C_L, label='왼쪽')
    ax.bar([i + w / 2 for i in x], [s['clear_right_3m'] for s in summ], w * 0.95, color=C_R, label='오른쪽')
    ax.axhline(0, color=INK2, lw=0.8)
    style(ax, '출입구 보행자 출현 3 m — 최소 이격', '최소 이격 [m]'); ax.set_xticks(list(x)); ax.set_xticklabels([s['cand'] for s in summ]); ax.set_ylim(0, 0.6); ax.legend(fontsize=8, frameon=False, loc='upper center', ncol=2)
    ax = fig.add_subplot(gs[0, 2])
    ax.bar(list(x), [s['rev'] for s in summ], 0.6, color=C_SYS)
    style(ax, '주차 줄 사이 후진 차량 — 충돌률 (출현 1~3 m)', '충돌률 [%]'); ax.set_xticks(list(x)); ax.set_xticklabels([s['cand'] for s in summ])
    ax = fig.add_subplot(gs[1, 0])
    ax.bar([i - w / 2 for i in x], [float(s['ground_near']) for s in summ], w * 0.95, color='#4a3aa7', label='바닥이 처음 보이는 거리')
    ax.bar([i + w / 2 for i in x], [float(s['corridor_full']) for s in summ], w * 0.95, color='#e87ba4', label='진로 폭 전체가 보이는 거리')
    style(ax, '카메라 사각지대 (앞범퍼 기준, 기하 계산)', '거리 [m]'); ax.set_xticks(list(x)); ax.set_xticklabels([s['cand'] for s in summ]); ax.legend(fontsize=8, frameon=False)
    ax = fig.add_subplot(gs[1, 1]); ax.axis('off')
    ax.set_title('턱 인식 (3 cm 통과 · 5/8 cm 턱 앞 정지)', fontsize=11, color=INK, loc='left', fontweight='bold')
    for i, s in enumerate(summ):
        y = 0.9 - i * 0.15
        ax.text(0.0, y, s['cand'], fontsize=10, fontweight='bold', transform=ax.transAxes)
        ax.text(0.12, y, f"3 cm 통과 {s['curb3_cross']}   5·8 cm 정지 {s['curb58_stop']}   정지 시 턱까지 {s['curb_gap'] or '-'} m", fontsize=9.5, color=INK2, transform=ax.transAxes)
    ax = fig.add_subplot(gs[1, 2])
    lx = range(len(lsum))
    for j, (key, lab, c) in enumerate((('v5_G10', '뒤차 5 m/s, 간격 10 m', '#2a78d6'), ('v7_G10', '뒤차 7 m/s, 간격 10 m', '#eb6834'))):
        ax.bar([i + (j - 0.5) * 0.36 for i in lx], [float(l[key]) if l[key] != '' else 0 for l in lsum], 0.34, color=c, label=lab)
    ax.axhline(0, color=INK2, lw=0.8)
    style(ax, '후방 라이다 — 추월 시 측면 최소 이격', '최소 이격 [m]'); ax.set_xticks(list(lx)); ax.set_xticklabels([l['cand'] for l in lsum]); ax.set_ylim(0, 0.65); ax.legend(fontsize=7.5, frameon=False, loc='upper center', ncol=2)
    ax.text(0.0, -0.2, '뒤차 7 m/s · 간격 6 m 는 모든 후보에서 이격 0 (접촉 직전)', transform=ax.transAxes, fontsize=8, color=MUTED)
    legend = '   '.join(f"{s['cand']} {s['desc']}" for s in summ) + '\n' + '   '.join(f"{l['cand']} {l['desc']}" for l in lsum)
    fig.text(0.05, 0.06, legend, fontsize=8.5, color=INK2, linespacing=1.6)
    out = os.path.join(RES, 'sensor_report.png'); fig.savefig(out, facecolor='white'); print('saved', out)
    for s in summ:
        print(s)
    for l in lsum:
        print(l)


if __name__ == '__main__':
    main()
