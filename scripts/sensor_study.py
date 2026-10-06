#!/usr/bin/env python3
"""E6 센서 배치 시험: 후보별 YAML 생성 + 기하 분석.
- 카메라 후보: 좌우 위치(왼쪽 모서리/중앙), 높이(0.30~1.60 m), 아래 기울기
- 라이다 후보: 높이(0.25~1.50 m), 좌우(중앙/왼쪽 모서리)
출력: config/scenarios/sensor/*.yaml, results/sensor/geometry.csv, results/sensor/candidates.json
실행 순서는 scripts/run_sensor_study.sh 가 후보마다 sim.launch.py 를 그 인자로 다시 띄워서 돌린다.
"""
import copy, csv, json, math, os
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(PKG, 'config', 'scenarios'); OUT = os.path.join(CFG, 'sensor'); RES = os.path.join(PKG, 'results', 'sensor')
BASE_Z = 0.25                      # base_link 원점 지상고
HFOV = math.radians(87.0)          # 깊이 카메라 수평 화각
VFOV = 2 * math.atan(math.tan(HFOV / 2) * 120 / 160)   # 160×120 → 수직 약 71°
RANGE = 6.0; FRONT = 0.55; CORR = 0.60

# 이름: (cam_x, cam_y, 지상고 h, 아래 기울기[deg], 설명)
CAMS = {
    'C0': (0.56, 0.40, 0.45, 0, '현재: 왼쪽 앞 모서리 0.45 m'),
    'C1': (0.56, 0.00, 0.45, 0, '앞 중앙 0.45 m'),
    'C2': (0.56, 0.00, 0.45, 15, '앞 중앙 0.45 m, 15° 숙임'),
    'C3': (0.56, 0.00, 0.30, 0, '앞 중앙 범퍼 0.30 m'),
    'C4': (0.56, 0.00, 0.90, 20, '앞 중앙 0.90 m(팔걸이 높이), 20° 숙임'),
    'C5': (0.56, 0.00, 1.60, 30, '앞 중앙 기둥 1.60 m, 30° 숙임'),
}
# 이름: (lidar_x, lidar_y, 지상고 h, 설명)
LIDARS = {
    'L0': (-0.56, 0.00, 0.45, '현재: 뒤 중앙 0.45 m'),
    'L1': (-0.56, 0.00, 0.25, '뒤 중앙 0.25 m(낮게)'),
    'L2': (-0.56, 0.00, 0.90, '뒤 중앙 0.90 m'),
    'L3': (-0.40, 0.00, 1.50, '등받이 위 기둥 1.50 m'),
    'L4': (-0.56, 0.40, 0.45, '뒤 왼쪽 모서리 0.45 m(차선 쪽)'),
}


def geometry(cx, cy, h, pitch_deg):
    """카메라 기하: 앞범퍼 기준 [바닥이 처음 보이는 거리, 진로 폭 전체가 보이기 시작하는 거리, 바닥이 보이는 최대 거리]."""
    p = math.radians(pitch_deg); half_v = VFOV / 2
    low = p + half_v                                        # 아래쪽 화각 끝(수평 아래 +)
    d_ground_near = cx + h / math.tan(low) - FRONT
    lat = max(abs(CORR - cy), abs(-CORR - cy))              # 카메라에서 진로 가장자리까지 가장 먼 횡거리
    d_corr_full = cx + lat / math.tan(HFOV / 2) - FRONT
    high = p - half_v                                       # 위쪽 화각 끝이 수평 아래면 그 각도까지 바닥이 보임
    far_by_range = math.sqrt(max(RANGE ** 2 - h ** 2, 0.0))
    far = min(far_by_range, h / math.tan(high)) if high > 0 else far_by_range
    return round(d_ground_near, 2), round(max(d_corr_full, 0.0), 2), round(cx + far - FRONT, 2)


def cam_params(cx, cy, h, pitch):
    return dict(cam_xyz=[cx, cy, h], cam_pitch=math.radians(pitch))


def launch_args_cam(cx, cy, h, pitch):
    return f"cam_x:={cx} cam_y:={cy} cam_z:={round(h - BASE_Z, 3)} cam_pitch:={round(math.radians(pitch), 4)}"


def main():
    os.makedirs(OUT, exist_ok=True); os.makedirs(RES, exist_ok=True)
    e2 = yaml.safe_load(open(os.path.join(CFG, 'E2_doorway.yaml')))
    curb = yaml.safe_load(open(os.path.join(CFG, 'E5_curb.yaml')))
    e3 = yaml.safe_load(open(os.path.join(CFG, 'E3_car_reverse.yaml')))
    e4 = yaml.safe_load(open(os.path.join(CFG, 'E4_rear_approach.yaml')))
    sys_prof = {'system': dict(e2['profiles']['system'], label='시스템(자율 급정거)')}
    cands = {}
    with open(os.path.join(RES, 'geometry.csv'), 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['cand', 'desc', 'x', 'y', 'h', 'pitch_deg', 'ground_near_m', 'corridor_full_m', 'ground_far_m'])
        for k, (cx, cy, h, pd, desc) in CAMS.items():
            w.writerow([k, desc, cx, cy, h, pd, *geometry(cx, cy, h, pd)])
    for k, (cx, cy, h, pd, desc) in CAMS.items():
        cp = cam_params(cx, cy, h, pd); files = []
        # S1 출입구 보행자: 왼쪽(기존 E2) / 오른쪽(같은 벽을 오른쪽에 두고 남쪽으로 주행)
        c = copy.deepcopy(e2); c.update(profiles=sys_prof, repeats=2, output=f'results/sensor/{k}_door.csv', controller_params=cp)
        c['event'].update(appearance_distances=[1.0, 1.5, 2.0, 3.0], ped_speeds=[1.2], variants=[
            dict(name='left'),
            dict(name='right', mirror=True, facade_p0=e2['event']['facade_p1'], facade_p1=e2['event']['facade_p0'],
                 door_s=round(math.dist(e2['event']['facade_p0'], e2['event']['facade_p1']) - e2['event']['door_s'], 2),
                 start_s=round(math.dist(e2['event']['facade_p0'], e2['event']['facade_p1']) - e2['event']['door_s'] - (e2['event']['door_s'] - e2['event']['start_s']), 2))])
        files.append(f'{k}_door.yaml'); yaml.safe_dump(c, open(os.path.join(OUT, files[-1]), 'w'), allow_unicode=True, sort_keys=False)
        # S2 턱: 3 cm(통과해야), 5·8 cm(턱 앞 정지해야)
        c = copy.deepcopy(curb); c.update(profiles={'system': curb['profiles']['system']}, repeats=2, output=f'results/sensor/{k}_curb.csv', controller_params=cp)
        c['event']['variants'] = [v for v in curb['event']['variants'] if v['name'] in ('h3', 'h5', 'h8')]
        files.append(f'{k}_curb.yaml'); yaml.safe_dump(c, open(os.path.join(OUT, files[-1]), 'w'), allow_unicode=True, sort_keys=False)
        # S3 주차 줄 사이 후진 차량(가림)
        c = copy.deepcopy(e3); c.update(profiles={'system': e3['profiles']['system']}, repeats=2, output=f'results/sensor/{k}_rev.csv', controller_params=cp)
        c['event'].update(appearance_distances=[1.0, 2.0, 3.0], ped_speeds=[2.0])
        files.append(f'{k}_rev.yaml'); yaml.safe_dump(c, open(os.path.join(OUT, files[-1]), 'w'), allow_unicode=True, sort_keys=False)
        cands[k] = dict(kind='cam', desc=desc, launch=launch_args_cam(cx, cy, h, pd), files=files)
    for k, (lx, ly, h, desc) in LIDARS.items():
        c = copy.deepcopy(e4); c.update(profiles={'system': e4['profiles']['system']}, repeats=1, output=f'results/sensor/{k}_rear.csv',
                                         controller_params=dict(rear_lidar_x=lx, rear_lidar_y=ly))
        c['event'].update(appearance_distances=[6.0, 10.0], ped_speeds=[5.0, 7.0], robot_l_list=[1.0])
        f = f'{k}_rear.yaml'; yaml.safe_dump(c, open(os.path.join(OUT, f), 'w'), allow_unicode=True, sort_keys=False)
        cands[k] = dict(kind='lidar', desc=desc, launch=f"lidar_x:={lx} lidar_y:={ly} lidar_z:={round(h - BASE_Z, 3)}", files=[f])
    json.dump(cands, open(os.path.join(RES, 'candidates.json'), 'w'), ensure_ascii=False, indent=1)
    for r in csv.reader(open(os.path.join(RES, 'geometry.csv'))):
        print(r)


if __name__ == '__main__':
    main()
