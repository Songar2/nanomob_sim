#!/usr/bin/env python3
"""4단계 안전 제어기: 전방 Depth 포인트클라우드 → 진로 내 최소거리 → 경고/감속/정지 → cmd_vel.
반응 지연(delay)과 감속도(decel)를 파라미터로 바꿔 '시스템(0.2 s)' vs '사람(1.0 s)'을 같은 코드로 비교한다.
"""
import json, math, collections
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import PointCloud2, LaserScan, Imu
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

LEVELS = ['FREE', 'WARN', 'SLOW', 'STOP']


class SafetyController(Node):
    def __init__(self):
        super().__init__('safety_controller')
        p = self.declare_parameters('', [
            ('delay', 0.2),            # 반응 지연 [s]
            ('decel', 1.5),            # 허용 감속도 [m/s^2]
            ('accel', 0.8),            # 가속 한계 [m/s^2]
            ('stop_margin', 0.5),      # 정지 시 확보할 전방 이격 [m]
            ('max_speed', 1.67),       # 6 km/h
            ('corridor_halfwidth', 0.60),  # 차체 반폭 0.45 + 여유 0.15
            ('front_x', 0.55),         # base_link 원점 → 앞 범퍼
            ('cam_xyz', [0.56, 0.40, 0.45]),  # base_link 기준 카메라 위치 (z는 지상고)
            ('cam_pitch', 0.0),        # 카메라 아래 기울기 [rad] (URDF cam_pitch 와 같게)
            ('z_min', 0.10), ('z_max', 2.0),  # 지상 기준 장애물 높이 창
            ('warn_ttc', 3.0),
            ('cloud_frame', 'auto'),   # gz(x전방,y좌,z상) | optical(z전방,x우,y하) | auto
            ('enabled', True),         # False면 사용자 명령 그대로 통과 (비교용)
            ('budget_delay', True),    # 정지거리 예산에 반응지연 주행거리(v·τ)를 포함 (기획안 5-3 공식)
            # --- 후방 라이다 뒤차 접근 (기획안 B4): 감지 → 길 가장자리로 붙기(pull) + 감속 → 임박하면 정지 → 통과 후 재출발 ---
            ('rear_lidar_x', -0.56),   # base_link 기준 라이다 위치(뒤 범퍼), 라이다는 yaw π 로 장착
            ('rear_lidar_y', 0.0),
            ('rear_halfwidth', 3.0),   # 후방 감시 폭(좌우 ±3 m, 이면도로 차선 포함)
            ('rear_min_speed', 1.0),   # 지면 기준 접근 속도가 이 이상인 물체만 위협 (정지 물체·자기 운동 제외)
            ('rear_ttc', 6.0),         # 충돌까지 6 s 이내면 위협
            ('rear_d_trig', 3.0),      # 또는 3 m 이내
            ('rear_v_pull', 1.0),      # 가장자리로 붙는 동안 속도 상한 (옆으로 붙으려면 전진이 필요)
            ('rear_stop_ttc', 2.0),    # 가장자리에 붙은 뒤 TTC 2 s 이내면 정지(붙기 전에는 멈추지 않고 계속 붙는다)
            ('rear_clear_hold', 1.0),  # 뒤차가 시야에서 사라진 뒤 1 s 후 재출발
            ('rear_max_active', 10.0), # 위협 상태가 이 시간을 넘기면 강제 해제(멈춰 선 뒤차에 영구 정지 방지)
            ('reset_rear', False),     # True 로 설정하면 후방·턱·전방 이력 초기화(시행 시작용 — 이전 시행 상태가 넘어오지 않게)
            # --- C3 경사(기획안: IMU 로 경사 감지 → 최고속도 자동 하향) ---
            ('gravity_align', True),   # IMU pitch 로 포인트 높이를 중력 기준으로 보정(경사로 위에서 바닥을 장애물로 오인 방지)
            ('slope_comp', True),      # 내리막에서 가용 감속도(brake_cap − g·sinθ)로 계획하고, 평지와 같은 정지거리가 되게 최고속도를 낮춤
            ('brake_cap', 2.0),        # 평지 최대 제동 감속도 [m/s²] (구동계 한계, slope_plant 와 같은 가정)
            # --- C1 턱·단차(기획안: Depth 로 높이 측정 → 넘을 수 있으면 감속 통과, 아니면 정지/우회) ---
            ('curb_comp', True),
            ('h_pass', 0.035),         # 넘을 수 있는 턱 높이 상한 [m] — E5 속도 스윕: 캐스터 r=0.09 로는 5 cm 불가, 3 cm 는 1.0 m/s 이상에서만 통과
            ('curb_min', 0.025),       # 이보다 낮으면 무시(노면 요철·센서 잡음)
            ('curb_look', 2.5),        # 앞범퍼 앞 탐색 길이 [m]
            ('curb_speed', 1.0),       # 턱 통과 속도 [m/s] — 0.8 m/s 이하에서는 캐스터가 턱을 못 올라감(E5 스윕)
            ('curb_slow_dist', 1.5),   # 턱까지 이 거리 안이면 통과 속도로 제한
        ])
        self.user = Twist(); self.v_cmd = 0.0; self.odom_v = 0.0
        self.hist = collections.deque(maxlen=400)   # (t, dmin)
        self.dmin_now = float('inf'); self.frame_detected = None
        self.last_t = None
        self.rear_hist = collections.deque(maxlen=400)   # (t, d_r) 후방 최근접 거리
        self.rear_active = False; self.rear_stop = False; self.rear_seen_t = 0.0; self.rear_since = 0.0; self.rear_ttc = float('inf'); self.rear_d = float('inf')
        self.create_subscription(LaserScan, '/scan_rear', self.on_scan, 5)
        self.pitch = 0.0; self.pitch_f = 0.0
        self.create_subscription(Imu, '/imu', self.on_imu, 20)
        self.curb_state = 'NONE'; self.curb_h = 0.0; self.curb_d = float('inf'); self.curb_hold_until = -1.0
        self.sub_pc = self.create_subscription(PointCloud2, '/depth/points', self.on_cloud, 5)
        self.sub_user = self.create_subscription(Twist, '/nanomob/user_cmd', self.on_user, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_state = self.create_publisher(String, '/nanomob/safety', 10)
        self.timer = self.create_timer(0.02, self.step)   # 50 Hz
        self.add_on_set_parameters_callback(self.on_params)
        self.get_logger().info('safety_controller up')

    def P(self, name):
        return self.get_parameter(name).value

    def on_user(self, msg):
        self.user = msg

    def on_odom(self, msg):
        self.odom_v = msg.twist.twist.linear.x

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cloud(self, msg):
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.size == 0:
            self.dmin_now = float('inf'); self.hist.append((self.now(), self.dmin_now)); return
        pts = pts[np.isfinite(pts).all(axis=1)]
        cf = self.P('cloud_frame')
        if cf == 'auto':
            # gz 관례면 x가 양수 깊이, optical이면 z가 양수 깊이
            fx = np.mean(pts[:, 0] > 0.2); fz = np.mean(pts[:, 2] > 0.2)
            cf = 'gz' if fx >= fz else 'optical'
            if self.frame_detected != cf:
                self.frame_detected = cf; self.get_logger().info(f'cloud frame detected: {cf}')
        if cf == 'optical':
            fwd, left, up = pts[:, 2], -pts[:, 0], -pts[:, 1]
        else:
            fwd, left, up = pts[:, 0], pts[:, 1], pts[:, 2]
        cx, cy, cz = self.P('cam_xyz')
        cp = self.P('cam_pitch')
        if abs(cp) > 1e-6:   # 카메라가 아래로 cp 만큼 숙여져 있으면 차체 기준으로 되돌림
            fwd, up = fwd * math.cos(cp) + up * math.sin(cp), -fwd * math.sin(cp) + up * math.cos(cp)
        xb = fwd + cx; yb = left + cy; zg = up + cz          # base_link 기준 (zg는 지상고)
        if self.P('gravity_align') and abs(self.pitch_f) > 0.005:   # 차체가 숙여진 만큼 되돌려 중력 기준 높이로
            p = self.pitch_f; zr = zg - 0.25
            xb, zg = xb * math.cos(p) + zr * math.sin(p), -xb * math.sin(p) + zr * math.cos(p) + 0.25
        if self.P('curb_comp'):
            c = self.detect_curb(xb, yb, zg); t = self.now()
            if c is not None and c[0] > self.P('h_pass'):
                self.curb_state = 'BLOCK'; self.curb_h, self.curb_d = c      # 넘을 수 없는 턱 → 장애물로 취급(정지, 우회 필요)
            elif c is not None:
                self.curb_state = 'SLOW'; self.curb_h, self.curb_d = c
                if c[1] < self.P('curb_slow_dist'):                         # 턱을 다 넘을 때까지(차체 길이+여유) 통과 속도 유지
                    self.curb_hold_until = t + (c[1] + 1.4) / self.P('curb_speed')
            else:
                self.curb_state = 'SLOW' if t < self.curb_hold_until else 'NONE'
        hw = self.P('corridor_halfwidth')
        m = (np.abs(yb) < hw) & (zg > self.P('z_min')) & (zg < self.P('z_max')) & (xb > self.P('front_x'))
        d = (xb[m] - self.P('front_x'))
        self.dmin_now = float(d.min()) if d.size else float('inf')
        if self.P('curb_comp') and self.curb_state == 'BLOCK':
            self.dmin_now = min(self.dmin_now, self.curb_d)
        self.hist.append((self.now(), self.dmin_now))

    def on_imu(self, m):
        q = m.orientation
        self.pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))   # +: 앞이 숙여짐(내리막)
        self.pitch_f += 0.1 * (self.pitch - self.pitch_f)                          # 100 Hz, 시정수 약 0.1 s

    def detect_curb(self, xb, yb, zg):
        """진로 안 바닥 포인트를 앞뒤 0.1 m 칸으로 나눠 칸별 높이 중앙값을 보고, 처음 curb_min 을 넘는 칸 = 턱 위치,
        그 뒤 3칸 중앙값 = 턱 높이. 반환 (높이, 앞범퍼~턱 거리) 또는 None."""
        fx = self.P('front_x'); m = (np.abs(yb) < self.P('corridor_halfwidth')) & (xb > fx) & (xb < fx + self.P('curb_look')) & (zg < 0.30)
        if m.sum() < 30:
            return None
        bins = np.floor((xb[m] - fx) / 0.1).astype(int); z = zg[m]
        med = {}
        for b in np.unique(bins):
            zz = z[bins == b]
            if zz.size >= 4:
                med[int(b)] = float(np.median(zz))
        keys = sorted(med)
        for i, b in enumerate(keys):
            if med[b] > self.P('curb_min'):
                top = [med[k] for k in keys[i:i + 4]]
                return float(np.median(top)), b * 0.1
        return None

    def on_params(self, params):
        if any(p.name == 'reset_rear' and p.value for p in params):
            self.rear_hist.clear(); self.rear_active = False; self.rear_stop = False; self.rear_d = float('inf'); self.rear_ttc = float('inf')
            self.curb_state = 'NONE'; self.curb_hold_until = -1.0; self.curb_d = float('inf'); self.hist.clear(); self.v_cmd = 0.0
        return SetParametersResult(successful=True)

    def on_scan(self, msg):
        r = np.asarray(msg.ranges, dtype=float); a = msg.angle_min + np.arange(r.size) * msg.angle_increment
        ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max * 0.999)
        r, a = r[ok], a[ok]
        d_back = r * np.cos(a); y = -r * np.sin(a) + self.P('rear_lidar_y')   # 라이다 기준 후방거리, base_link 횡위치
        m = (d_back > -0.05) & (np.abs(y) < self.P('rear_halfwidth'))
        self.rear_hist.append((self.now(), float(d_back[m].min()) if m.any() else float('inf')))

    def rear_cmd(self, t):
        """후방 위협 상태기계. (속도 상한, 상태) 반환. 반응 지연만큼 과거 데이터로 판단한다."""
        t_ref = t - self.P('delay')
        win = [(tt, d) for tt, d in self.rear_hist if t_ref - 0.5 <= tt <= t_ref and math.isfinite(d)]
        # 거리가 물리적으로 불가능하게 튀면(상대속도 10 m/s 초과) 다른 물체가 옆에서 시야에 들어온 것 → 그 뒤부터 새로 추적
        for i in range(len(win) - 1, 0, -1):
            if abs(win[i][1] - win[i - 1][1]) > 0.3 + 10.0 * (win[i][0] - win[i - 1][0]):
                win = win[i:]; break
        if len(win) >= 3:
            ts = np.array([w[0] for w in win]); ds = np.array([w[1] for w in win])
            slope = float(np.polyfit(ts - ts[0], ds, 1)[0])      # d_r 변화율 = v_robot − v_obj
            v_obj = self.odom_v - slope; closing = -slope; d_ref = float(ds[-1])
            ttc = d_ref / closing if closing > 0.2 else float('inf')
            # 실제로 다가오는 물체만(closing>0.3): 옆 벽처럼 거리가 그대로인 물체는 v_obj=v_robot 로 보여 오인되므로 제외
            if v_obj > self.P('rear_min_speed') and closing > 0.3 and (ttc < self.P('rear_ttc') or d_ref < self.P('rear_d_trig')):
                if not self.rear_active:
                    self.rear_since = t
                self.rear_active = True; self.rear_ttc = ttc; self.rear_d = d_ref
        vis = any(t - 0.3 <= tt and math.isfinite(d) and d < 14.0 for tt, d in self.rear_hist)
        if self.rear_active:
            if vis:
                self.rear_seen_t = t
            if (not vis and t - self.rear_seen_t > self.P('rear_clear_hold')) or t - self.rear_since > self.P('rear_max_active'):
                self.rear_active = False; self.rear_stop = False; self.rear_d = float('inf'); self.rear_ttc = float('inf')
        if not self.rear_active:
            return float('inf'), 'CLEAR'
        at_edge = self.user.linear.y > 0.5       # 경로 추종부(지도 기반)가 'l 방향 가장자리 도달'을 linear.y 로 알림
        self.rear_stop = at_edge and (self.rear_ttc < self.P('rear_stop_ttc') or self.rear_d < self.P('rear_d_trig'))
        return (0.0, 'STOP') if self.rear_stop else (self.P('rear_v_pull'), 'PULL')

    def delayed_dmin(self):
        t_ref = self.now() - self.P('delay')
        val = float('inf')
        for t, d in self.hist:
            if t <= t_ref:
                val = d
            else:
                break
        return val

    def step(self):
        t = self.now()
        if self.last_t is None:
            self.last_t = t; return
        dt = max(1e-3, min(0.1, t - self.last_t)); self.last_t = t
        v_user = max(0.0, min(self.user.linear.x, self.P('max_speed')))
        a = self.P('decel'); margin = self.P('stop_margin')
        if self.P('enabled') and self.P('slope_comp') and self.pitch_f > 0.01:
            # 내리막: 가용 감속도로 계획하고, 평지 최고속도의 정지거리 D_ref 를 넘지 않게 최고속도를 낮춤
            tau0 = self.P('delay'); vmax = self.P('max_speed')
            a_plan = max(0.2, min(a, self.P('brake_cap') - 9.81 * math.sin(self.pitch_f)))
            d_ref = vmax * tau0 + vmax ** 2 / (2 * a)
            v_user = min(v_user, -a_plan * tau0 + math.sqrt((a_plan * tau0) ** 2 + 2 * a_plan * d_ref))
            a = a_plan
        if self.P('enabled') and self.P('curb_comp') and self.curb_state == 'SLOW' and (self.curb_d < self.P('curb_slow_dist') or t < self.curb_hold_until):
            v_user = min(v_user, self.P('curb_speed'))
        if self.P('enabled'):
            d_eff = self.delayed_dmin()
            tau = self.P('delay') if self.P('budget_delay') else 0.0
            # 허용속도 v: v·τ + v²/(2a) = d − margin 의 양근 → v = −aτ + sqrt(a²τ² + 2a(d − margin))
            if d_eff > margin:
                v_allow = -a * tau + math.sqrt((a * tau) ** 2 + 2 * a * (d_eff - margin))
            else:
                v_allow = 0.0
            v_target = min(v_user, v_allow)
            v_rear, rear_state = self.rear_cmd(t)
            v_target = min(v_target, v_rear)
            ttc = d_eff / max(self.odom_v, 1e-3)
            if v_allow <= 0.0:
                level = 3
            elif v_allow < v_user:
                level = 2
            elif ttc < self.P('warn_ttc'):
                level = 1
            else:
                level = 0
        else:
            d_eff = float('inf'); v_target = v_user; level = 0; rear_state = 'CLEAR'
        # 속도 변화율 제한 (감속은 decel, 가속은 accel)
        if v_target < self.v_cmd:
            self.v_cmd = max(v_target, self.v_cmd - a * dt)
        else:
            self.v_cmd = min(v_target, self.v_cmd + self.P('accel') * dt)
        cmd = Twist(); cmd.linear.x = self.v_cmd; cmd.angular.z = self.user.angular.z if self.v_cmd > 0.02 else 0.0
        self.pub_cmd.publish(cmd)
        s = String(); s.data = json.dumps(dict(t=round(t, 3), level=LEVELS[level], dmin=None if math.isinf(self.dmin_now) else round(self.dmin_now, 3),
                                            d_eff=None if math.isinf(d_eff) else round(d_eff, 3), v_cmd=round(self.v_cmd, 3), v_odom=round(self.odom_v, 3), rear=rear_state,
                                            rear_d=None if math.isinf(self.rear_d) else round(self.rear_d, 2), rear_ttc=None if math.isinf(self.rear_ttc) else round(self.rear_ttc, 2),
                                            pitch=round(math.degrees(self.pitch_f), 2), a_plan=round(a, 2), curb=self.curb_state, curb_h=round(self.curb_h, 3),
                                            curb_d=None if math.isinf(self.curb_d) else round(self.curb_d, 2)))
        self.pub_state.publish(s)


def main():
    rclpy.init()
    n = SafetyController()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
