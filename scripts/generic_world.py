#!/usr/bin/env python3
"""가상 시가지 월드 생성 → worlds/nanomob_town.sdf (월드 이름 nanomob_town)
특정 장소를 본뜨지 않은 일반 환경. 시나리오별 구역을 평지에 배치하고, 치수는 일반적인 값을 쓴다.
  A 건물 출입구 보도 (E2/A1):  x 0 의 건물 동쪽 면, 출입구(폭 1.6 m, 안쪽 2 m 들어간 현관) y=25, 보도 3.5 m, 차도 7 m
  B 버스 차고지 출구 (E1/B1):  남북 도로(중심 x=60, 폭 7 m), 서쪽 길가 → 차고지(담장·출구는 러너가 배치)
  C 주차장 보행로 (E3/B1):     동서 보행로(y=-60, 폭 3 m) 북쪽 주차장(주차면 2.5×5 m)
  D 이면도로 (E4/B4):          보도 없는 남북 도로(중심 x=200, 폭 6 m), 양옆 건물
  E 경사·턱 시험 레인 (E5):    models/e5_lanes (x 112~131, y -22~44) — 이 스크립트 밖에서 따로 띄움
바닥 판(차도·보도·주차장)은 시각 전용이고 윗면을 1 cm 이하로 둔다(깊이 카메라 턱 인식이 판 가장자리를 턱으로 오인하지 않게).
"""
import os

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD = 'nanomob_town'
ASPHALT, SIDEWALK, LOT, LINE = (0.24, 0.24, 0.26), (0.72, 0.70, 0.66), (0.33, 0.34, 0.36), (0.93, 0.93, 0.90)
WALLS = [(0.80, 0.76, 0.68), (0.70, 0.62, 0.55), (0.78, 0.78, 0.80), (0.66, 0.70, 0.74), (0.82, 0.72, 0.60)]
out = []; n = [0]


def mat(rgb):
    r, g, b = rgb
    return f"<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>"


def name(p):
    n[0] += 1; return f"{p}{n[0]}"


def plate(x0, y0, x1, y1, rgb, z=0.003, yaw=0.0):
    """바닥 판(시각 전용, 두께 2 mm). (x0,y0)-(x1,y1) 축정렬 사각형."""
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    out.append(f"    <visual name='{name('p')}'><pose>{cx:.2f} {cy:.2f} {z:.4f} 0 0 {yaw}</pose><geometry><box><size>{abs(x1 - x0):.2f} {abs(y1 - y0):.2f} 0.002</size></box></geometry>{mat(rgb)}</visual>")


def solid(x0, y0, x1, y1, h, rgb, z0=0.0):
    """충돌 있는 상자(건물·벽)."""
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2; sz = f"{abs(x1 - x0):.2f} {abs(y1 - y0):.2f} {h:.2f}"
    k = name('b'); pose = f"<pose>{cx:.2f} {cy:.2f} {z0 + h / 2:.2f} 0 0 0</pose>"
    out.append(f"    <collision name='c_{k}'>{pose}<geometry><box><size>{sz}</size></box></geometry></collision>")
    out.append(f"    <visual name='{k}'>{pose}<geometry><box><size>{sz}</size></box></geometry>{mat(rgb)}</visual>")


def deco(x, y, z, sx, sy, sz, rgb):
    """시각 전용 장식(창문·문 등)."""
    out.append(f"    <visual name='{name('d')}'><pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose><geometry><box><size>{sx:.2f} {sy:.2f} {sz:.2f}</size></box></geometry>{mat(rgb)}</visual>")


def windows_x(xface, y0, y1, floors, side, h_floor=3.5):
    """x=xface 면(side=+1 동쪽을 봄, -1 서쪽)에 층별 창."""
    y = y0 + 2.0
    while y < y1 - 1.5:
        for f in range(floors):
            deco(xface + side * 0.03, y, f * h_floor + 2.0, 0.04, 1.6, 1.3, (0.20, 0.26, 0.32))
        y += 3.2


def windows_y(yface, x0, x1, floors, side, h_floor=3.5):
    x = x0 + 2.0
    while x < x1 - 1.5:
        for f in range(floors):
            deco(x, yface + side * 0.03, f * h_floor + 2.0, 1.6, 0.04, 1.3, (0.20, 0.26, 0.32))
        x += 3.2


def lamp(x, y):
    k = name('l'); pole = "<geometry><cylinder><radius>0.08</radius><length>4.5</length></cylinder></geometry>"
    out.append(f"    <collision name='c_{k}'><pose>{x} {y} 2.25 0 0 0</pose>{pole}</collision>")
    out.append(f"    <visual name='{k}'><pose>{x} {y} 2.25 0 0 0</pose>{pole}{mat((0.35, 0.36, 0.38))}</visual>")
    deco(x, y, 4.55, 0.4, 0.4, 0.15, (0.95, 0.93, 0.80))


def tree(x, y):
    k = name('t'); trunk = "<geometry><cylinder><radius>0.15</radius><length>3.0</length></cylinder></geometry>"
    out.append(f"    <collision name='c_{k}'><pose>{x} {y} 1.5 0 0 0</pose>{trunk}</collision>")
    out.append(f"    <visual name='{k}'><pose>{x} {y} 1.5 0 0 0</pose>{trunk}{mat((0.35, 0.25, 0.17))}</visual>")
    out.append(f"    <visual name='{k}c'><pose>{x} {y} 4.2 0 0 0</pose><geometry><sphere><radius>2.2</radius></sphere></geometry>{mat((0.22, 0.44, 0.22))}</visual>")


def dashes_y(x, y0, y1, rgb=LINE, z=0.0055):
    y = y0
    while y < y1:
        plate(x - 0.07, y, x + 0.07, min(y + 3.0, y1), rgb, z); y += 6.0


def zone_a():
    """E2 건물 출입구: 건물 동쪽 면 x=0 (y -5~45), 출입구 y 24.2~25.8 (현관 안쪽 x=-2)."""
    h = 10.5
    solid(-16, -5, 0, 24.2, h, WALLS[0]); solid(-16, 25.8, 0, 45, h, WALLS[0])
    solid(-16, 24.2, -2, 25.8, h, WALLS[0])                       # 현관 뒤 벽(안쪽 2 m)
    deco(-1.0, 25.0, 3.15, 2.0, 1.6, 0.3, WALLS[0])                # 현관 위 캐노피(높이 3 m, 시각 전용)
    deco(-1.99, 25.0, 1.1, 0.04, 1.2, 2.1, (0.30, 0.22, 0.16))     # 안쪽 문
    windows_x(0, -5, 24.2, 3, +1); windows_x(0, 25.8, 45, 3, +1)
    plate(0, -10, 3.5, 55, SIDEWALK, 0.004)                         # 보도 3.5 m
    plate(3.5, -10, 10.5, 55, ASPHALT, 0.003)                       # 차도 7 m
    dashes_y(7.0, -10, 55)
    for y in (-8, 10, 40, 52):
        lamp(3.2, y)


def zone_b():
    """E1 버스 차고지: 도로 중심 x=60(폭 7 m), 서쪽 길가(로봇 x≈56.5), 담장선 x≈54.7(러너가 담장·버스 배치), 차고지 x 25~54.7."""
    plate(56.5, -15, 63.5, 55, ASPHALT, 0.003); dashes_y(60.0, -15, 55)
    plate(55.3, -15, 56.5, 55, SIDEWALK, 0.004)                     # 길가 포장 1.2 m (로봇은 이 경계를 따라 감)
    plate(25, 0, 54.6, 42, LOT, 0.0035)                             # 차고지 바닥
    for y in range(3, 40, 4):                                       # 버스 주차면 선(폭 4 m, 길이 12 m)
        plate(28, y - 0.06, 40, y + 0.06, LINE, 0.0055)
    solid(10, 4, 24, 36, 7.0, WALLS[3])                             # 차고·정비동
    windows_x(24, 4, 36, 2, +1)
    plate(63.5, -15, 66.5, 55, SIDEWALK, 0.004)                     # 동쪽 보도
    solid(68, -10, 84, 18, 9.0, WALLS[1]); solid(68, 24, 84, 50, 12.0, WALLS[2])
    windows_x(68, -10, 18, 2, -1); windows_x(68, 24, 50, 3, -1)


def zone_c():
    """E3 주차장: 보행로 y -61.5~-58.5 (중심 y=-60, 동쪽으로 걷는 방향), 북쪽 주차장 y -57.5~-38, x 96~144."""
    plate(94, -61.5, 146, -58.5, SIDEWALK, 0.004)
    plate(94, -68.5, 146, -61.5, ASPHALT, 0.003)                    # 남쪽 차도
    plate(96, -57.5, 144, -38, LOT, 0.0035)
    for x in range(96, 145, 3):                                     # 주차면: 폭 2.5 m(선 간격 2.5~3 m), 깊이 5 m — 남북 두 줄
        plate(x - 0.06, -57.5, x + 0.06, -52.5, LINE, 0.0055)
        plate(x - 0.06, -43.0, x + 0.06, -38.0, LINE, 0.0055)
    solid(96, -36, 144, -27, 14.0, WALLS[2])                        # 주차장 북쪽 건물(시험 레인 y≥-22 와 5 m 띄움)
    windows_y(-36, 96, 144, 4, -1)
    for x in (95, 120, 145):
        lamp(x, -58.9)


def zone_d():
    """E4 이면도로: 중심 x=200, 폭 6 m(x 197~203), 보도 없음. 양옆 건물은 도로 가장자리에서 2 m 이상 떨어뜨림."""
    plate(197, -15, 203, 115, ASPHALT, 0.003)
    plate(196.9, -15, 197.0, 115, LINE, 0.0055); plate(203.0, -15, 203.1, 115, LINE, 0.0055)   # 길가 흰 선
    for (y0, y1, hh, c) in ((-10, 18, 9.0, 1), (24, 52, 12.0, 4), (58, 90, 9.0, 0), (96, 112, 15.0, 2)):
        solid(205, y0, 219, y1, hh, WALLS[c]); windows_x(205, y0, y1, int(hh // 3.5), -1)
        solid(178, y0 + 3, 193, y1 - 3, hh - 1.5, WALLS[(c + 2) % 5]); windows_x(193, y0 + 3, y1 - 3, int((hh - 1.5) // 3.5), +1)


def zone_misc():
    """구역 사이 녹지·나무(경로·시험 레인과 떨어진 곳에만)."""
    for x, y in ((20, -20), (30, -30), (85, -20), (90, 60), (150, 60), (165, -10), (170, 20), (40, 70), (-25, 60), (-25, -20)):
        tree(x, y)


def main():
    zone_a(); zone_b(); zone_c(); zone_d(); zone_misc()
    sdf = f"""<?xml version="1.0"?>
<sdf version="1.9">
<world name="{WORLD}">
  <!-- scripts/generic_world.py 로 생성한 가상 시가지(특정 장소 아님). 구역 A~D 는 스크립트 설명 참고 -->
  <physics name="4ms" type="ignored"><max_step_size>0.004</max_step_size><real_time_factor>0</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
  <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
  <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.75 0.85 0.95 1</background><shadows>true</shadows></scene>
  <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 60 0 0 0</pose>
    <diffuse>0.9 0.9 0.85 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.4 0.3 -0.85</direction></light>
  <model name="ground"><static>true</static><link name="link">
    <collision name="c"><geometry><plane><normal>0 0 1</normal><size>600 600</size></plane></geometry>
      <surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction></surface></collision>
    <visual name="v"><geometry><plane><normal>0 0 1</normal><size>600 600</size></plane></geometry>{mat((0.47, 0.55, 0.40))}</visual>
  </link></model>
  <model name="town"><static>true</static><link name="link">
{chr(10).join(out)}
  </link></model>
</world>
</sdf>
"""
    os.makedirs(os.path.join(PKG, 'worlds'), exist_ok=True)
    open(os.path.join(PKG, 'worlds', f'{WORLD}.sdf'), 'w').write(sdf)
    print('saved', len(out), 'elements')


if __name__ == '__main__':
    main()
