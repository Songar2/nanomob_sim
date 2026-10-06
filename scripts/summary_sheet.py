#!/usr/bin/env python3
"""E1~E5 결과 한 장 요약 (A4 가로) → results/E1_E5_summary.png / .pdf
기획안 9장(시뮬레이션 검증)용. 결과 CSV 에서 수치를 다시 집계하므로 배치를 다시 돌리면 이 스크립트만 재실행하면 된다.
"""
import csv, collections, os
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

plt.rcParams['font.family'] = 'NanumSquare'; plt.rcParams['axes.unicode_minus'] = False
RES = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'results')

# 색: 시리즈는 역할 고정(시스템=1번 파랑, 사람=2번 주황, 시스템 변형=3번 청록), 기준선은 무채색
C_SYS, C_HUM, C_VAR, C_BASE = '#2a78d6', '#eb6834', '#1baf7a', '#898781'
INK, INK2, MUTED, GRID, AXIS, SURF = '#0b0b0b', '#52514e', '#898781', '#e1e0d9', '#c3c2b7', '#fcfcfb'
GOOD, SERIOUS, CRIT = '#0ca30c', '#ec835a', '#d03b3b'


def rows(name):
    return list(csv.DictReader(open(os.path.join(RES, name))))


def coll_rate(rs, prof, key='d_app', filt=lambda r: True):
    g = collections.defaultdict(list)
    for r in rs:
        if r['profile'] == prof and filt(r) and r.get('emerged', '1') == '1':
            g[float(r[key])].append(int(r['collision']))
    return sorted((k, 100 * sum(v) / len(v)) for k, v in g.items())


def safe_from(series):
    """충돌률이 그 거리부터 끝까지 0 인 최소 출현거리."""
    out = None
    for d, c in reversed(series):
        if c > 0:
            break
        out = d
    return out


def style(ax, title, ylabel, xlabel):
    ax.set_facecolor(SURF)
    ax.set_title(title, fontsize=10.5, color=INK, loc='left', pad=6, fontweight='bold')
    ax.set_ylabel(ylabel, fontsize=8.5, color=INK2); ax.set_xlabel(xlabel, fontsize=8.5, color=INK2)
    ax.grid(color=GRID, lw=0.6); ax.set_axisbelow(True)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(AXIS)
    ax.tick_params(colors=MUTED, labelsize=8)


def line(ax, pts, color, label, dashed=False, lab_dy=0):
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    ax.plot(xs, ys, '--' if dashed else '-', color=color, lw=2, marker='o', ms=5, mec=SURF, mew=1.2, label=label, zorder=3)


def rate_panel(ax, rs, profs, title, xlabel):
    for p, c, lab in profs:
        line(ax, coll_rate(rs, p), c, lab)
    style(ax, title, '충돌률 [%]', xlabel); ax.set_ylim(-5, 108); ax.set_yticks([0, 25, 50, 75, 100])
    ax.legend(fontsize=7.5, frameon=False, loc='upper right')


def main():
    e1, e2, e3, e4 = rows('E1_bus_exit.csv'), rows('E2_doorway.csv'), rows('E3_car_reverse.csv'), rows('E4_rear_approach.csv')
    e5a, e5b, e5s = rows('E5_slope.csv'), rows('E5_curb.csv'), rows('E5_curb_speed.csv')
    n_total = sum(len(x) for x in (e1, e2, e3, e4, e5a, e5b, e5s))

    s = {k: safe_from(coll_rate(r, p)) for k, (r, p) in {
        'e2s': (e2, 'system'), 'e2h': (e2, 'human'), 'e2p': (e2, 'system_preslow'),
        'e1s': (e1, 'system'), 'e1h': (e1, 'human'), 'e3s': (e3, 'system'), 'e3h': (e3, 'human')}.items()}
    fmt = lambda v: f'{v:g} m' if v is not None else '6 m 에서도 충돌 발생'

    lat0 = min(float(r['robot_l']) for r in e4)   # 진로 쪽으로 더 나온(이격이 작은) 기기 위치

    def e4gap(prof, v, lat=None):
        lat = lat0 if lat is None else lat
        m = [float(r['min_clear']) for r in e4 if r['profile'] == prof and float(r['v_ped']) == v and float(r['robot_l']) == lat and r['min_clear']]
        return sum(m) / len(m)

    def e5clear(prof, var, d=3.5):
        r = [x for x in e5a if x['profile'] == prof and x['variant'] == var and float(x['d_app']) == d][0]
        return float(r['min_clear']), r['collision'] == '1'

    fig = plt.figure(figsize=(16.54, 11.69), dpi=150, facecolor='white')
    gs = GridSpec(3, 3, figure=fig, height_ratios=[0.8, 1, 1], hspace=0.40, wspace=0.24, left=0.05, right=0.975, top=0.885, bottom=0.085)
    fig.text(0.05, 0.955, '시니어 nano-mobility 1단계 시나리오 시뮬레이션 결과 (E1~E5)', fontsize=19, fontweight='bold', color=INK)
    fig.text(0.05, 0.928, f'가상 시가지(nanomob_town) · Gazebo Harmonic · 기기 6 km/h(1.67 m/s), 감속 1.5 m/s², 반응지연 시스템 0.2 s / 사람 1.0 s · '
             f'전방 Depth(6 m) + 후방 라이다(12 m) + IMU · 총 {n_total}회', fontsize=10, color=INK2)
    fig.text(0.05, 0.908, '출입구·담장·주차 줄 위치, 도로 폭, 경사·턱 값은 현장 측량 전 가정값입니다. 상대 물체는 충돌 지오메트리 없이 기하학적으로 접촉을 판정합니다.', fontsize=8.5, color=MUTED)

    # ---- 요약 표 ----
    ax = fig.add_subplot(gs[0, :]); ax.axis('off')
    cols = [('시나리오', 0.0), ('기획안', 0.085), ('상황 (장소)', 0.135), ('시스템 (자율 급정거·대응)', 0.37), ('사람 조작 / 기준', 0.6), ('시사점', 0.78)]
    y = 1.0
    for name, x in cols:
        ax.text(x, y, name, fontsize=9.5, fontweight='bold', color=INK2, transform=ax.transAxes, va='top')
    ax.plot([0, 1], [0.925, 0.925], color=AXIS, lw=1, transform=ax.transAxes, clip_on=False)
    g12c, g12x = e5clear('system_noC3', 'g12'); g12s, _ = e5clear('system', 'g12'); g8c, _ = e5clear('system_noC3', 'g8'); g8s, _ = e5clear('system', 'g8')
    table = [
        ('E2', 'A1', '건물 출입구에서 보행자 튀어나옴\n(구역 A)', f'출현 {fmt(s["e2s"])} 이상이면 충돌 0 %\n출입구 접근 시 0.8 m/s 사전감속 → {fmt(s["e2p"])}',
         f'사람: {fmt(s["e2h"])} 이상에서 충돌 0 %', '출현 2~3 m 는 반응만으로 부족 →\n출입구 앞 사전감속이 핵심'),
        ('E1', 'B1', '버스 차고지 출구에서 버스 진출\n(구역 B, 담장 틈)', f'출현 {fmt(s["e1s"])} 이상이면 충돌 0 %\n(정지 후 버스 통과 뒤 재출발)',
         f'사람: {fmt(s["e1h"])}', '옆에서 오는 차량은 진로 진입 직전에야\n보임 → 측면 접근 예측 필요'),
        ('E3', 'B1', '주차면에서 차량 후진 출현\n(구역 C, 주차장 옆 보행로)', f'출현 {fmt(s["e3s"])} 이상이면 충돌 0 %\n(3 m 이상은 대부분 정지 후 재개)',
         f'사람: {fmt(s["e3h"])}', '주차 줄이 가리는 1~2 m 출현은\n시스템도 충돌'),
        ('E4', 'B4', '보도 없는 이면도로 뒤차 접근\n(구역 D, 폭 6 m)', f'후방 라이다 → 길가로 붙고 정지\n측면 이격(뒤차 5 m/s) {e4gap("none", 5.0):.2f} → {e4gap("system", 5.0):.2f} m',
         f'기준(무반응) {e4gap("none", 5.0):.2f} m · 사람 {e4gap("human", 5.0):.2f} m\n뒤차 7 m/s: 시스템 {e4gap("system", 7.0):.2f} m', '25 km/h 뒤차는 라이다 12 m 로는\n붙을 시간 부족 → 감지거리 확대'),
        ('E5a', 'C3', '내리막 경사로 위 보행자 진입·정지\n(시험 레인, 경사 0~12 %)', f'IMU 경사 보정: 12 % 에서 최고속도 1.29 m/s 로\n자동 하향 → 출현 3.5 m 이격 {g12s:.2f} m',
         f'보정 없음: 8 % 이격 {g8c:.2f} m, 12 % {"충돌" if g12x else f"{g12c:.2f} m"}\n사람: 전 경사 충돌', '제동 한계 2.0 m/s² 가정 시 12 % 는\n가용 감속도 0.83 → 속도 하향 필수'),
        ('E5b', 'C1', '보도 턱·문턱 3/5/8/12 cm\n(시험 레인, 캐스터 지름 18/25/32 cm 비교)', 'Depth 로 턱 높이 측정: 3 cm 는 1.0 m/s 통과,\n5 cm 이상은 턱 앞 0.2~0.3 m 정지(우회 필요)',
         '인식 없음: 5·8 cm 에 걸림\n(3 cm 는 전속력 관성으로만 넘음)', '캐스터 지름 18→32 cm: 5 cm 를 1.0 m/s 로 통과\n(25 cm 는 1.67 m/s 필요), 8 cm↑ 는 32 cm 로도 불가'),
    ]
    rh = 0.148
    for i, row in enumerate(table):
        yy = 0.90 - i * rh
        if i % 2 == 0:
            ax.add_patch(plt.Rectangle((-0.005, yy - rh + 0.012), 1.01, rh, transform=ax.transAxes, color='#f4f3ef', zorder=0, clip_on=False))
        for (name, x), txt in zip(cols, row):
            ax.text(x + 0.002, yy, txt, fontsize=8.6 if x > 0.1 else 10, color=INK if x < 0.13 else INK2,
                    fontweight='bold' if x < 0.13 else 'normal', transform=ax.transAxes, va='top', linespacing=1.35)

    # ---- 그래프 6개 ----
    rate_panel(fig.add_subplot(gs[1, 0]), e2, [('system', C_SYS, '시스템'), ('system_preslow', C_VAR, '시스템 + 출입구 사전감속'), ('human', C_HUM, '사람 조작')],
               'E2 · A1 출입구 보행자 출현', '보행자 출현 거리 [m]')
    rate_panel(fig.add_subplot(gs[1, 1]), e1, [('system', C_SYS, '시스템'), ('human', C_HUM, '사람 조작')], 'E1 · B1 차고지 버스 진출', '버스 출현 시 거리 [m] (버스 1.5~5 m/s 평균)')
    rate_panel(fig.add_subplot(gs[1, 2]), e3, [('system', C_SYS, '시스템'), ('human', C_HUM, '사람 조작')], 'E3 · B1 주차장 후진 차량', '차량 출현 시 거리 [m] (후진 1~3 m/s 평균)')

    ax = fig.add_subplot(gs[2, 0])
    for p, c, lab, dsh in (('system', C_SYS, '후방 감지 시스템', False), ('human', C_HUM, '사람 조작', False), ('none', C_BASE, '무반응(기준)', True)):
        line(ax, [(v, e4gap(p, v)) for v in (3.0, 5.0, 7.0)], c, lab, dashed=dsh)
    ax.axhline(0.5, color=MUTED, lw=0.8, ls=':'); ax.text(3.0, 0.52, '이격 0.5 m', fontsize=7, color=MUTED, va='bottom')
    style(ax, 'E4 · B4 이면도로 뒤차 접근', '추월 시 측면 최소 이격 [m]', '뒤차 속도 [m/s] (기기가 진로 쪽에 있는 경우, 간격 6~25 m 평균)')
    ax.set_xticks([3, 5, 7]); ax.set_xticklabels(['3 (11 km/h)', '5 (18 km/h)', '7 (25 km/h)']); ax.set_ylim(-0.02, 0.95); ax.legend(fontsize=7.5, frameon=False, loc='upper right')

    ax = fig.add_subplot(gs[2, 1])
    for p, c, lab in (('system', C_SYS, '시스템 + C3 경사 보정'), ('system_noC3', C_VAR, '시스템, 경사 보정 없음'), ('human', C_HUM, '사람 조작')):
        pts = [(int(v[1:]), *e5clear(p, v)) for v in ('g0', 'g5', 'g8', 'g12')]
        line(ax, [(x, max(c_, 0.0) if not k else 0.0) for x, c_, k in pts], c, lab)
        hit = [(x, 0.0) for x, c_, k in pts if k]
        ax.scatter([h[0] for h in hit], [h[1] for h in hit], marker='x', s=55, color=CRIT, zorder=5, lw=1.8)
    ax.text(0.2, 0.02, '× = 충돌', fontsize=7.5, color=CRIT)
    style(ax, 'E5a · C3 내리막 (출현 3.5 m)', '보행자와 최소 이격 [m]', '경사 [%] (가용 감속도 2.0 → 0.83 m/s²)')
    ax.set_xticks([0, 5, 8, 12]); ax.set_ylim(-0.03, 0.45); ax.legend(fontsize=7.5, frameon=False, loc='upper right')

    ax = fig.add_subplot(gs[2, 2]); style(ax, 'E5b · C1 턱·단차', '', '턱 높이')
    profs = [('system', 'C1 턱 인식'), ('system_noC1', '인식 없음'), ('none', '무반응')]
    hs = sorted({float(r['curb_h']) for r in e5b})
    lab = {'crossed': ('통과', GOOD), 'stopped': ('턱 앞 정지', '#2a78d6'), 'stuck': ('걸림', CRIT), 'tipped': ('전복', CRIT)}
    for i, (p, pl) in enumerate(profs):
        for j, h in enumerate(hs):
            rs = [r for r in e5b if r['profile'] == p and float(r['curb_h']) == h]
            res = collections.Counter(r['result'] for r in rs).most_common(1)[0][0]
            vc = [float(r['v_contact']) for r in rs if r['v_contact']]
            t, c = lab[res]
            ax.add_patch(plt.Rectangle((j + 0.04, i + 0.06), 0.92, 0.88, color=c, alpha=0.16, lw=0))
            ax.add_patch(plt.Rectangle((j + 0.04, i + 0.06), 0.06, 0.88, color=c, lw=0))
            ax.text(j + 0.55, i + 0.58, t, ha='center', va='center', fontsize=9, color=INK, fontweight='bold')
            ax.text(j + 0.55, i + 0.3, f'접촉 {vc[0]:.1f} m/s' if vc else '접촉 없음', ha='center', va='center', fontsize=7, color=INK2)
    ax.set_xlim(0, len(hs)); ax.set_ylim(0, len(profs)); ax.invert_yaxis(); ax.grid(False)
    ax.set_xticks([j + 0.5 for j in range(len(hs))]); ax.set_xticklabels([f'{h * 100:.0f} cm' for h in hs])
    ax.set_yticks([i + 0.5 for i in range(len(profs))]); ax.set_yticklabels([p[1] for p in profs], fontsize=8.5, color=INK2)
    ax.spines['left'].set_visible(False); ax.tick_params(length=0)

    fig.text(0.05, 0.018, 'E1~E3·E5a 충돌률은 출현거리별 전 조건 평균(점당 2회, E5a 1회). 시스템 = 전방 Depth 정지거리 예산 제어(v·τ + v²/2a = d - 0.5 m). '
             '결과·영상: ~/Documents/nanomob/결과/E*, 코드: ~/ros2_ws/src/nanomob_sim', fontsize=7.5, color=MUTED)
    for ext in ('png', 'pdf'):
        fig.savefig(os.path.join(RES, f'E1_E5_summary.{ext}'), facecolor='white')
    print('saved', os.path.join(RES, 'E1_E5_summary.png'))


if __name__ == '__main__':
    main()
