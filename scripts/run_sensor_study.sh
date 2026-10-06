#!/bin/bash
# E6 센서 배치 시험 실행기: 후보마다 sim 을 그 장착 인자로 다시 띄우고 YAML 들을 돌린다.
# 사용: scripts/run_sensor_study.sh [후보 ...]   (기본: candidates.json 전체) — 먼저 scripts/sensor_study.py 실행
# 진행 기록: results/sensor/progress.txt
source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/nanomob_sim
P=results/sensor/progress.txt; mkdir -p results/sensor
killsim() { pkill -9 -x gz; pkill -9 -x ruby; pkill -9 -f robot_state_publisher; pkill -9 -f lib/nanomob_sim/safety_controller; pkill -9 -f lib/nanomob_sim/slope_plant; pkill -9 -f ros_gz_bridge; pkill -9 -f "ros2 launch nanomob_sim"; sleep 3; }
CANDS=${@:-$(python3 -c "import json;print(' '.join(json.load(open('results/sensor/candidates.json'))))")}
for k in $CANDS; do
  args=$(python3 -c "import json;print(json.load(open('results/sensor/candidates.json'))['$k']['launch'])")
  files=$(python3 -c "import json;print(' '.join(json.load(open('results/sensor/candidates.json'))['$k']['files']))")
  killsim
  setsid ros2 launch nanomob_sim sim.launch.py headless:=true $args > results/sensor/sim_$k.log 2>&1 &
  sleep 35
  for f in $files; do
    out=$(python3 -c "import yaml;print(yaml.safe_load(open('config/scenarios/sensor/$f'))['output'])"); rm -f $out
    ros2 run nanomob_sim scenario_runner config/scenarios/sensor/$f > results/sensor/${f%.yaml}.log 2>&1
  done
  echo "done $k" >> $P
done
killsim
setsid ros2 launch nanomob_sim sim.launch.py headless:=true > /dev/null 2>&1 &
echo all >> $P
