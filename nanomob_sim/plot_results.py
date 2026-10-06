#!/usr/bin/env python3
"""results.csv → 정지거리·최소이격·충돌률 비교 그림 (시스템 vs 사람)."""
import csv, sys, os, collections, math
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'NanumSquare'; plt.rcParams['axes.unicode_minus'] = False


def main_bus(src, rows):
    """E1(버스 진출): 프로파일 × 버스 속도 별로 출현거리에 따른 결과(충돌률·최소 이격·정지거리)."""
    colors = {'system': '#1f77b4', 'human': '#d62728'}
    names = {'system': '시스템 (지연 0.2 s)', 'human': '사람 조작 (지연 1.0 s)'}
    vs = sorted({float(r['v_ped']) for r in rows}); dapps = sorted({float(r['d_app']) for r in rows})
    fig, axes = plt.subplots(2, len(vs), figsize=(5.4 * len(vs), 8), dpi=120, squeeze=False)
    for j, v in enumerate(vs):
        for p in ('system', 'human'):
            cr, mc = [], []
            for d in dapps:
                rs = [r for r in rows if r['profile'] == p and float(r['v_ped']) == v and float(r['d_app']) == d]
                cr.append(100 * sum(int(r['collision']) for r in rs) / max(len(rs), 1))
                m = [float(r['min_clear']) for r in rs if r['min_clear']]
                mc.append(sum(m) / len(m) if m else float('nan'))
            axes[0][j].plot(dapps, cr, 'o-', color=colors[p], label=names[p]); axes[1][j].plot(dapps, mc, 'o-', color=colors[p], label=names[p])
        axes[0][j].set_title(f'상대 차량 속도 {v:g} m/s ({v * 3.6:.0f} km/h) — 충돌률 [%]'); axes[1][j].set_title('차량과 최소 이격 [m] (0 이하 = 충돌)')
        axes[0][j].set_ylim(-5, 105); axes[1][j].axhline(0, color='k', lw=0.8)
        for ax in (axes[0][j], axes[1][j]):
            ax.set_xlabel('차량 출현 시 기기-차량 근접면 거리 [m]'); ax.grid(alpha=0.3); ax.legend()
    n = len([r for r in rows if r['profile'] == 'system' and float(r['v_ped']) == vs[0] and float(r['d_app']) == dapps[0]])
    fig.suptitle(f'{"E3 주차장 후진 차량(B1) — 주차장 옆 보행로" if rows[0]["event"] == "car_reverse" else "E1 차고지 출구 버스 진출(B1) — 가상 시가지"}, 기기 {rows[0]["v_robot"]} m/s, 감속 {rows[0]["decel"]} m/s², 점당 {n}회')
    plt.tight_layout(); out = os.path.splitext(src)[0] + '.png'; plt.savefig(out); print('saved', out)


def main_rear(src, rows):
    """E4(뒤차 접근): 뒤차가 나란히 지나갈 때의 측면 최소 이격 — 프로파일별 비교 (열 = 뒤차 속도, 행 = 기기 길가 위치)."""
    colors = {'none': '#7f7f7f', 'system': '#1f77b4', 'human': '#d62728'}
    names = {'none': '무반응(기준: 그대로 직진)', 'system': '시스템 (지연 0.2 s)', 'human': '사람 조작 (지연 1.0 s)'}
    vs = sorted({float(r['v_ped']) for r in rows}); lats = sorted({float(r['robot_l']) for r in rows}); gaps = sorted({float(r['d_app']) for r in rows})
    fig, axes = plt.subplots(len(lats), len(vs), figsize=(5.2 * len(vs), 4.2 * len(lats)), dpi=120, squeeze=False)
    for i, lat in enumerate(lats):
        for j, v in enumerate(vs):
            ax = axes[i][j]
            for p in ('none', 'system', 'human'):
                xs, ys, cs = [], [], []
                for g in gaps:
                    rs = [r for r in rows if r['profile'] == p and float(r['v_ped']) == v and float(r['d_app']) == g and float(r['robot_l']) == lat]
                    m = [float(r['min_clear']) for r in rs if r['min_clear']]
                    if m:
                        xs.append(g); ys.append(sum(m) / len(m)); cs.append(any(int(r['collision']) for r in rs))
                ax.plot(xs, ys, '-', color=colors[p], label=names[p], alpha=0.7)
                ax.scatter(xs, ys, c=[('#d62728' if c else colors[p]) for c in cs], marker='o', s=40, zorder=5)
                ax.scatter([x for x, c in zip(xs, cs) if c], [y for y, c in zip(ys, cs) if c], marker='x', color='k', s=70, zorder=6)
            ax.axhline(0, color='k', lw=0.8); ax.axhline(0.5, color='#999', lw=0.8, ls=':')
            ax.set_title(f'뒤차 {v:g} m/s ({v * 3.6:.0f} km/h), 기기 길가 위치 l={lat:g} m'); ax.set_xlabel('트리거 시 뒤차 앞면~기기 뒤 범퍼 간격 [m]'); ax.set_ylabel('차량과 최소 이격 [m] (× = 충돌)')
            ax.grid(alpha=0.3)
            if i == 0 and j == 0:
                ax.legend(fontsize=8)
    fig.suptitle('E4 보도 없는 이면도로 뒤차 접근(B4) — 후방 라이다 감지 → 길 가장자리로 붙고 감속·정지 (점선 = 이격 0.5 m)')
    plt.tight_layout(); out = os.path.splitext(src)[0] + '.png'; plt.savefig(out); print('saved', out)


def main_slope(src, rows):
    """E5a(C3 내리막): 경사별로 출현거리에 따른 최소 이격(× = 충돌) — 경사 보정 유무·사람 조작 비교."""
    colors = {'system_noC3': '#ff7f0e', 'system': '#1f77b4', 'human': '#d62728'}
    names = {'system_noC3': '시스템, 경사 보정 없음', 'system': '시스템 + C3 경사 보정', 'human': '사람 조작 (지연 1.0 s)'}
    vars_ = sorted({r['variant'] for r in rows}, key=lambda v: int(v[1:])); dapps = sorted({float(r['d_app']) for r in rows})
    fig, axes = plt.subplots(1, len(vars_), figsize=(4.8 * len(vars_), 4.6), dpi=120, squeeze=False)
    for j, v in enumerate(vars_):
        ax = axes[0][j]
        for p in ('system_noC3', 'system', 'human'):
            rs = {float(r['d_app']): r for r in rows if r['variant'] == v and r['profile'] == p}
            xs = [d for d in dapps if d in rs and rs[d]['min_clear']]; ys = [float(rs[d]['min_clear']) for d in xs]
            ax.plot(xs, ys, 'o-', color=colors[p], label=names[p])
            cx = [d for d in xs if rs[d]['collision'] == '1']
            ax.scatter(cx, [float(rs[d]['min_clear']) for d in cx], marker='x', color='k', s=70, zorder=6)
        g = int(v[1:]); a = max(0.1, 2.0 - 9.81 * math.sin(math.atan(g / 100)))
        ax.set_title(f'경사 {g} % — 가용 감속도 {min(a, 2.0):.2f} m/s²'); ax.axhline(0, color='k', lw=0.8)
        ax.set_xlabel('보행자 출현 거리 [m]'); ax.set_ylabel('보행자와 최소 이격 [m] (× = 충돌)'); ax.grid(alpha=0.3)
        if j == 0:
            ax.legend(fontsize=8)
    fig.suptitle('E5a 내리막(C3) — 경사로 위 보행자 진입·정지, 기기 1.67 m/s, 제동 한계 2.0 m/s² 가정 (내리막 가용 감속도 = 2.0 - g·sinθ)')
    plt.tight_layout(); out = os.path.splitext(src)[0] + '.png'; plt.savefig(out); print('saved', out)


def main_curb(src, rows):
    """E5b(C1 턱): 높이별 결과 막대(통과/걸림/턱 앞 정지)와 접촉 속도·충격."""
    profs = ['none', 'system_noC1', 'system']
    names = {'none': '무반응(기준)', 'system_noC1': '시스템, 턱 인식 없음', 'system': '시스템 + C1 턱 인식'}
    rc = {'crossed': '#2ca02c', 'stopped': '#1f77b4', 'stuck': '#d62728', 'tipped': '#000000', 'timeout': '#999999'}
    rl = {'crossed': '통과', 'stopped': '턱 앞 정지(우회 필요)', 'stuck': '턱에 걸림', 'tipped': '전복', 'timeout': '시간 초과'}
    hs = sorted({float(r['curb_h']) for r in rows})
    fig, axes = plt.subplots(1, 2, figsize=(15, 4.8), dpi=120)
    ax = axes[0]; w = 0.26
    for i, p in enumerate(profs):
        for j, h in enumerate(hs):
            rs = [r for r in rows if r['profile'] == p and float(r['curb_h']) == h]
            if not rs:
                continue
            res = collections.Counter(r['result'] for r in rs).most_common(1)[0][0]
            ax.bar(j + (i - 1) * w, 1, w * 0.92, color=rc[res], edgecolor='white')
            ax.text(j + (i - 1) * w, 0.5, f"{names[p]}\n{rl[res]}", ha='center', va='center', rotation=90, fontsize=7.5, color='white')
    ax.set_xticks(range(len(hs))); ax.set_xticklabels([f'턱 {h * 100:.0f} cm' for h in hs]); ax.set_yticks([])
    ax.set_title('결과 (막대 왼쪽부터 무반응 · 턱 인식 없음 · C1 턱 인식)')
    ax = axes[1]
    for i, p in enumerate(profs):
        for j, h in enumerate(hs):
            rs = [r for r in rows if r['profile'] == p and float(r['curb_h']) == h and r['v_contact']]
            if rs:
                az = [max(float(r['az_peak']), float(r['ax_peak'])) for r in rs]
                ax.bar(j + (i - 1) * w, sum(az) / len(az), w * 0.92, color=['#7f7f7f', '#ff7f0e', '#1f77b4'][i], label=names[p] if j == 0 else None)
                ax.text(j + (i - 1) * w, sum(az) / len(az) + 1, f"{float(rs[0]['v_contact']):.1f} m/s", ha='center', fontsize=7)
    ax.set_xticks(range(len(hs))); ax.set_xticklabels([f'턱 {h * 100:.0f} cm' for h in hs])
    ax.set_ylabel('접촉 시 최대 충격 [m/s²] (50 ms 평균 IMU, 상하·전후 중 큰 값)'); ax.set_title('턱 접촉 충격과 접촉 속도 (턱 앞에서 멈춘 경우는 막대 없음)'); ax.legend(fontsize=8); ax.grid(alpha=0.3, axis='y')
    fig.suptitle('E5b 턱·단차(C1) — 구동륜 r 0.18 m · 앞 캐스터 r 0.09 m: 3 cm 는 1.0 m/s 이상에서만 넘고 5 cm 이상은 못 넘음')
    plt.tight_layout(); out = os.path.splitext(src)[0] + '.png'; plt.savefig(out); print('saved', out)


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else 'results/results.csv'
    rows = list(csv.DictReader(open(src)))
    if rows and rows[0]['event'] == 'curb':
        return main_curb(src, rows)
    if rows and 'variant' in rows[0] and rows[0]['event'] == 'doorway':
        return main_slope(src, rows)
    if rows and rows[0]['event'] == 'rear_approach':
        return main_rear(src, [r for r in rows if r['emerged'] in ('0', '1')])
    if rows and rows[0]['event'] in ('bus_exit', 'car_reverse'):
        return main_bus(src, [r for r in rows if r['emerged'] == '1'])
    rows = [r for r in rows if r['emerged'] == '1']
    by = collections.defaultdict(list)
    for r in rows:
        by[(r['profile'], float(r['d_app']))].append(r)
    profiles = sorted({k[0] for k in by}); dapps = sorted({k[1] for k in by})
    fig, axes = plt.subplots(1, 4, figsize=(19, 4.8), dpi=120)
    colors = {'system': '#1f77b4', 'human': '#d62728', 'system_preslow': '#2ca02c'}
    for p in profiles:
        sd, mc, cr, vi = [], [], [], []
        for d in dapps:
            rs = by.get((p, d), [])
            s = [float(r['stop_dist']) for r in rs if r['stop_dist']]
            m = [float(r['min_clear']) for r in rs if r['min_clear']]
            v = [float(r['v_impact']) if r['v_impact'] else 0.0 for r in rs]   # 비충돌 = 충돌속도 0
            sd.append(sum(s) / len(s) if s else float('nan')); mc.append(sum(m) / len(m) if m else float('nan'))
            cr.append(100 * sum(int(r['collision']) for r in rs) / len(rs) if rs else float('nan'))
            vi.append(sum(v) / len(v) if v else float('nan'))
        c = colors.get(p)
        lab = {'system': f'시스템 (지연 0.2 s, {rows[0]["v_robot"] if rows else ""} m/s)', 'human': '사람 조작 (지연 1.0 s)', 'system_preslow': '시스템 + 출입구 접근 감속 (0.8 m/s)'}.get(p, p)
        axes[0].plot(dapps, sd, 'o-', color=c, label=lab); axes[1].plot(dapps, mc, 'o-', color=c, label=lab)
        axes[2].plot(dapps, cr, 'o-', color=c, label=lab); axes[3].plot(dapps, vi, 'o-', color=c, label=lab)
    axes[0].set_title('출현 후 정지거리 [m] (정지한 시행만)'); axes[1].set_title('보행자와 최소 이격 [m] (0 이하 = 충돌)')
    axes[2].set_title('충돌률 [%]'); axes[3].set_title('평균 충돌 속도 [m/s] (비충돌 = 0)')
    axes[1].axhline(0, color='k', lw=0.8)
    for ax in axes:
        ax.set_xlabel('보행자 출현 거리 [m]'); ax.grid(alpha=0.3); ax.legend()
    n_rep = len(next(iter(by.values()))) if by else 0
    title = 'E2b 출입구 출현 후 진로 위 정지(A1)' if 'E2b' in src else 'E2 출입구 출현·횡단(A1)'
    fig.suptitle(f'{title} — 가상 시가지, 건물 출입구 / 감속 {rows[0]["decel"] if rows else "?"} m/s², 보행자 {sorted({r["v_ped"] for r in rows})} m/s, 점당 {n_rep}회')
    plt.tight_layout()
    out = os.path.splitext(src)[0] + '.png'
    plt.savefig(out); print('saved', out)


if __name__ == '__main__':
    main()
