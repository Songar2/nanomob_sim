# nanomob_sim — 시니어 nano-mobility 돌발상황 시뮬레이션

특정 장소를 본뜨지 않은 **가상 시가지**(`worlds/nanomob_town.sdf`, `scripts/generic_world.py`)에서 기획안 1단계 시나리오
(A1·B1·B4, C1·C3)를 돌발상황 이벤트로 재현해 **충돌 여부 · 정지거리 · 최소 이격 · 충돌 속도**를 자동 측정한다.

| 구역 | 시나리오 | 위치(월드 좌표, m) | 일반 치수 |
|---|---|---|---|
| A 건물 출입구 보도 | E2/A1 (+ 오른쪽 출현) | 건물 동쪽 면 x=0, 출입구 y=25 | 보도 3.5 m, 차도 7 m, 출입구 폭 1.6 m·안쪽 2 m |
| B 버스 차고지 출구 | E1/B1 | 도로 중심 x=60, 서쪽 차고지 | 도로 7 m, 길가 1.2 m, 담장 2 m·출구 7 m |
| C 주차장 옆 보행로 | E3/B1 | 보행로 y=-60, 북쪽 주차장 | 보행로 3 m, 주차면 2.5×5 m |
| D 이면도로 | E4/B4 | 중심 x=200 | 폭 6 m, 보도 없음 |
| E 경사·턱 시험 레인 | E5/C1·C3, E5c | x 112~131, y -22~44 | 경사 0/5/8/12 %, 턱 3/5/8/12 cm |


## 구성
```
worlds/nanomob_town.sdf      가상 시가지 월드 (scripts/generic_world.py 로 생성)
urdf/nanomob.urdf.xacro      기기 모델: 6바퀴(중앙 구동 2 + 캐스터 4), 폭 0.9 m, 전방 Depth 카메라(차체 고정), 후방 라이다
models/pedestrian/           보행자(원기둥+구, 속도 제어, 중력·충돌 없음 → 벽 뒤에서 걸어 나옴)
nanomob_sim/safety_controller.py  4단계 안전 제어기 (FREE/WARN/SLOW/STOP), 반응지연·감속도 파라미터
nanomob_sim/scenario_runner.py    YAML 시나리오 배치 실행 + CSV 지표 기록
nanomob_sim/plot_results.py       결과 그래프
config/scenarios/E2_doorway.yaml  E2(A1) 출입구 출현 시나리오
config/scenarios/E1_bus_exit.yaml E1(B1) 버스 차고지 출구 진출 배치 (60회) / E1_video*.yaml 영상 녹화용 대표 시행
models/bus, fence, car, parked_row, spectator  버스(11×2.5 m) · 차고지 담장 · 승용차(4.5×1.8 m) · 주차 차량 줄 · 관찰 카메라
config/scenarios/E3_car_reverse.yaml E3(B1) 주차장 후진 차량 배치 (60회) / E3_video*.yaml 영상용
nanomob_sim/recorder.py           관찰 카메라 → mp4 (ffmpeg, imageio-ffmpeg 필요), 한글 자막
```

## 실행
```bash
cd ~/ros2_ws && colcon build --packages-select nanomob_sim --symlink-install && source install/setup.bash
ros2 launch nanomob_sim sim.launch.py headless:=true      # GUI로 보려면 headless:=false
cd src/nanomob_sim
ros2 run nanomob_sim scenario_runner config/scenarios/E2_doorway.yaml
ros2 run nanomob_sim plot_results results/E2_doorway.csv  # → results/E2_doorway.png
ros2 run nanomob_sim scenario_runner config/scenarios/E1_bus_exit.yaml   # E1 60회 (약 6분)
ros2 run nanomob_sim plot_results results/E1_bus_exit.csv
ros2 run nanomob_sim scenario_runner config/scenarios/E1_video.yaml      # 영상: results/videos/*.mp4 (녹화 중엔 RTF 0.8 로 감속, 끝나면 5.0 복원)
python3 scripts/generic_world.py                          # 월드 재생성
```

## 안전 제어기 로직
- Depth 포인트클라우드 → base_link 변환 → 진로 폭(차체 반폭 0.45 + 0.15) · 높이 0.1~2.0 m 안의 최소 전방거리 `d`
- 반응 지연 `delay` 만큼 과거의 `d`를 사용 (시스템 0.2 s / 사람 1.0 s)
- 허용 속도는 정지거리 예산식 `v·delay + v²/(2·decel) = d − stop_margin` 의 해 (기획안 5-3 공식과 동일), 감속률은 `decel`로 제한
  - `budget_delay:=false`로 끄면 제동거리만 보는 순진한 제어기가 되어, 지연 동안 진행한 거리만큼 늦게 멈춘다 (첫 배치에서 확인한 실패 모드)
- 단계: FREE → WARN(TTC<3 s) → SLOW(허용속도<요청속도) → STOP(허용속도 0)

## 지표 (CSV)
`collision`, `min_clear`(footprint-보행자 원 이격, ≤0 충돌), `stop_dist`(출현 후 정지까지 주행거리),
`t_stop`, `peak_decel`(0.2 s 창), `v_emerge`(출현 순간 속도), `v_impact`(충돌 순간 속도), `v_min`

## 가정·한계
- 가상 시가지라 치수는 일반값(보도·도로 폭, 주차면, 담장 높이). 실제 장소 검증은 하지 않음.
- 상대 물체(보행자·버스·차량)는 충돌 지오메트리가 없어 접촉은 기하학적으로 판정.
- 깊이 카메라는 햇빛·비 열화를 재현하지 않음(가우시안 잡음 1 cm). 캐스터는 마찰 0 구체로 단순화.

## 다음 단계
1. 현장 측량: 100주년기념관 동측 출입구 실제 위치, 보도 폭·턱, 미래관 주차장 출구, 버스 차고지 출구
2. 캐스터 크기·구동 방식 비교(E5 턱), 실제 경사 위치(DEM)에 E5 레인 대체
3. DEM 지형, V-World 건물 높이 반영 → Blender → USD → ws1 Isaac Sim 이식
