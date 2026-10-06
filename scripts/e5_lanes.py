#!/usr/bin/env python3
"""E5 시험 레인(C3 내리막 · C1 턱) → models/e5_lanes/model.sdf (정적, 충돌 있음).
위치: 가상 시가지 구역 E(x 112~131, y -22~44).
- 내리막 레인: 상단 평지(y -22~-10, 높이 H) → 경사로(y -10~2, 12 m) → 평지
- 턱 레인: 같은 x, y 32~44 에 높이 h 의 단(보도 턱/문턱)
"""
import math, os, json
HERE = os.path.dirname(os.path.abspath(__file__)); PKG = os.path.dirname(HERE)
LANE_W = 3.0
SLOPES = [(114.0, 0.00), (119.0, 0.05), (124.0, 0.08), (129.0, 0.12)]   # (레인 중심 x, 경사)
CURBS = [(114.0, 0.03), (119.0, 0.05), (124.0, 0.08), (129.0, 0.12)]    # (레인 중심 x, 턱 높이 m)
PLAT = (-22.0, -10.0); RAMP = (-10.0, 2.0); CURB = (32.0, 44.0); T = 0.5


def mat(r, g, b):
    return f"<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>"


def box(name, pose, size, color, collide=True):
    g = f"<geometry><box><size>{size[0]:.3f} {size[1]:.3f} {size[2]:.3f}</size></box></geometry>"
    c = f"<collision name='{name}_c'><pose>{pose}</pose>{g}</collision>" if collide else ''
    return f"      {c}<visual name='{name}_v'><pose>{pose}</pose>{g}{mat(*color)}</visual>\n"


def main():
    out = []
    for x, _ in SLOPES:   # 레인 바닥 표시(시각 전용)
        out.append(box(f'lane{int(x)}', f"{x} 11 0.004 0 0 0", (LANE_W, 66, 0.008), (0.78, 0.76, 0.72), collide=False))
    for x, g in SLOPES:
        if g <= 0:
            continue
        L = RAMP[1] - RAMP[0]; H = g * L; th = math.atan(g)
        out.append(box(f'plat{int(x)}', f"{x} {(PLAT[0] + PLAT[1]) / 2} {H / 2} 0 0 0", (LANE_W, PLAT[1] - PLAT[0], H), (0.70, 0.70, 0.72)))
        ym, zm = (RAMP[0] + RAMP[1]) / 2, H / 2
        cy, cz = ym - T / 2 * math.sin(th), zm - T / 2 * math.cos(th)
        out.append(box(f'ramp{int(x)}', f"{x} {cy:.4f} {cz:.4f} {-th:.5f} 0 0", (LANE_W, L / math.cos(th), T), (0.62, 0.64, 0.68)))
    for x, h in CURBS:
        out.append(box(f'curb{int(x)}', f"{x} {(CURB[0] + CURB[1]) / 2} {h / 2} 0 0 0", (LANE_W, CURB[1] - CURB[0], h), (0.66, 0.62, 0.58)))
    sdf = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <!-- scripts/e5_lanes.py 로 생성. 내리막 {[g for _, g in SLOPES]}, 턱 {[h for _, h in CURBS]} m -->
  <model name="e5_lanes"><static>true</static>
    <link name="link">
{''.join(out)}    </link>
  </model>
</sdf>
"""
    os.makedirs(os.path.join(PKG, 'models', 'e5_lanes'), exist_ok=True)
    open(os.path.join(PKG, 'models', 'e5_lanes', 'model.sdf'), 'w').write(sdf)
    open(os.path.join(PKG, 'models', 'e5_lanes', 'model.config'), 'w').write('<?xml version="1.0"?>\n<model><name>e5_lanes</name><version>1.0</version><sdf version="1.9">model.sdf</sdf><description>E5 경사·턱 시험 레인</description></model>\n')
    json.dump(dict(lane_w=LANE_W, slopes=SLOPES, curbs=CURBS, plat=PLAT, ramp=RAMP, curb=CURB), open(os.path.join(PKG, 'maps', 'e5_lanes.json'), 'w'), indent=1)
    print('ok', len(out), 'boxes')


if __name__ == '__main__':
    main()
