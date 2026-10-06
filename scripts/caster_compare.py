#!/usr/bin/env python3
"""E5c 캐스터 크기별 턱 넘기 비교 → results/E5c_caster.png
입력: results/E5c_caster_r{0.09,0.125,0.16}.csv (턱 높이 × 접근 속도, 점당 2회, 시스템은 턱 앞에서 멈추지 않게 h_pass 0.25)
"""
import csv, os
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.family'] = 'NanumSquare'; plt.rcParams['axes.unicode_minus'] = False
RES = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'results')
INK, INK2, MUTED, AXIS = '#0b0b0b', '#52514e', '#898781', '#c3c2b7'
GOOD, SERIOUS, CRIT = '#0ca30c', '#ec835a', '#d03b3b'
RADII = ['0.09', '0.125', '0.16']


def load(r):
    return list(csv.DictReader(open(os.path.join(RES, f'E5c_caster_r{r}.csv'))))


def main():
    data = {r: load(r) for r in RADII}
    hs = sorted({float(x['curb_h']) for x in data[RADII[0]]})
    vs = sorted({int(x['profile'][1:]) / 100 for x in data[RADII[0]]})
    fig, axes = plt.subplots(1, len(RADII), figsize=(17, 5.6), dpi=140)
    summary = []
    for ax, r in zip(axes, RADII):
        rows = data[r]; best = {}
        for i, v in enumerate(vs):
            for j, h in enumerate(hs):
                rs = [x for x in rows if float(x['curb_h']) == h and abs(int(x['profile'][1:]) / 100 - v) < 1e-6]
                n_ok = sum(x['result'] == 'crossed' for x in rs)
                c = GOOD if n_ok == len(rs) else (SERIOUS if n_ok else CRIT)
                t = '통과' if n_ok == len(rs) else (f'{n_ok}/{len(rs)} 통과' if n_ok else ('전복' if any(x['result'] == 'tipped' for x in rs) else '걸림'))
                jolt = [max(float(x['az_peak']), float(x['ax_peak'])) for x in rs if x['result'] == 'crossed']
                ax.add_patch(plt.Rectangle((j + 0.04, i + 0.06), 0.92, 0.88, color=c, alpha=0.16, lw=0))
                ax.add_patch(plt.Rectangle((j + 0.04, i + 0.06), 0.06, 0.88, color=c, lw=0))
                ax.text(j + 0.55, i + 0.42, t, ha='center', va='center', fontsize=10, fontweight='bold', color=INK)
                if jolt:
                    ax.text(j + 0.55, i + 0.72, f'충격 {sum(jolt) / len(jolt):.0f} m/s²', ha='center', va='center', fontsize=7.5, color=INK2)
                if n_ok == len(rs):
                    best.setdefault(h, v)          # 그 높이를 확실히 넘는 최소 접근 속도
        ax.set_xlim(0, len(hs)); ax.set_ylim(0, len(vs)); ax.invert_yaxis()
        ax.set_xticks([j + 0.5 for j in range(len(hs))]); ax.set_xticklabels([f'턱 {h * 100:.0f} cm' for h in hs], fontsize=9, color=INK2)
        ax.set_yticks([i + 0.5 for i in range(len(vs))]); ax.set_yticklabels([f'{v:g} m/s' for v in vs], fontsize=9, color=INK2)
        for s in ax.spines.values():
            s.set_visible(False)
        ax.tick_params(length=0)
        hmax = max(best) if best else None
        ax.set_title(f'캐스터 지름 {float(r) * 200:.0f} cm (r {r} m)', fontsize=12, fontweight='bold', color=INK, loc='left')
        line = f'넘는 최대 높이 {hmax * 100:.0f} cm (최소 {best[hmax]:g} m/s)' if hmax else '시험한 턱 모두 못 넘음'
        ax.text(0, -0.08, line, transform=ax.transAxes, fontsize=9.5, color=INK2, va='top')
        summary.append((r, best))
    axes[0].set_ylabel('접근 속도', fontsize=9.5, color=INK2)
    fig.suptitle('E5c 캐스터 크기별 턱 넘기 — 구동륜 r 0.18 m · 차체 지상고 0.10 m 고정, 점당 2회 (충격 = 50 ms 평균 IMU 최대값, 통과한 시행만)',
                 fontsize=12.5, color=INK, x=0.01, ha='left')
    plt.tight_layout(rect=(0, 0.02, 1, 0.94))
    out = os.path.join(RES, 'E5c_caster.png'); fig.savefig(out, facecolor='white'); print('saved', out)
    for r, best in summary:
        print(r, {f'{h * 100:.0f}cm': v for h, v in sorted(best.items())})


if __name__ == '__main__':
    main()
