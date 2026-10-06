#!/usr/bin/env python3
"""돌발상황 시나리오 러너. record: true 면 관찰 카메라 영상을 시행마다 mp4 로 저장. YAML 하나로 (프로파일 × 출현거리 × 보행속도 × 반복) 배치를 돌리고 CSV로 지표를 쓴다.
이벤트 타입: doorway (A1/E2 — 벽 뒤에서 보행자가 진로로 횡단 진입), bus_exit (B1/E1 — 차고지 출구에서 버스가 진로를 가로지름).
"""
import collections, csv, json, math, os, random, sys, time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter as RclParam
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import Imu
from gz.msgs10.physics_pb2 import Physics
from gz.msgs10.boolean_pb2 import Boolean


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def rect_circle_clearance(rx, ry, ryaw, L, W, cx, cy, r):
    """로봇 사각 footprint(L×W, 중심 rx,ry, yaw)와 보행자 원(cx,cy,r)의 이격(음수면 겹침)."""
    dx, dy = cx - rx, cy - ry
    c, s = math.cos(-ryaw), math.sin(-ryaw)
    lx, ly = c * dx - s * dy, s * dx + c * dy
    qx = max(abs(lx) - L / 2, 0.0); qy = max(abs(ly) - W / 2, 0.0)
    return math.hypot(qx, qy) - r


def rect_corners(cx, cy, yaw, L, W):
    c, s_ = math.cos(yaw), math.sin(yaw)
    return [(cx + c * a - s_ * b, cy + s_ * a + c * b) for a, b in ((L / 2, W / 2), (L / 2, -W / 2), (-L / 2, -W / 2), (-L / 2, W / 2))]


def _seg_pt(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay; t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def rect_rect_clearance(A, B):
    """볼록 사각형 두 개의 이격(겹치면 음수 = SAT 최소 침투깊이)."""
    pen = float('inf')
    for poly in (A, B):
        for i in range(4):
            ex, ey = poly[(i + 1) % 4][0] - poly[i][0], poly[(i + 1) % 4][1] - poly[i][1]; n = math.hypot(ex, ey); ax_, ay_ = -ey / n, ex / n
            pa = [x * ax_ + y * ay_ for x, y in A]; pb = [x * ax_ + y * ay_ for x, y in B]
            gap = max(min(pb) - max(pa), min(pa) - max(pb))
            if gap > 0:   # 분리됨 → 꼭짓점-변 최소거리
                return min(min(_seg_pt(p[0], p[1], q[0][0], q[0][1], q[1][0], q[1][1]) for p in P for q in zip(Q, Q[1:] + Q[:1]))
                           for P, Q in ((A, B), (B, A)))
            pen = min(pen, -gap)
    return -pen


class Runner(Node):
    def __init__(self, cfg):
        super().__init__('scenario_runner')
        self.cfg = cfg
        self.world = cfg.get('world', 'nanomob_town')
        self.robot_name = cfg.get('robot_name', 'nanomob'); self.ped_name = cfg.get('ped_name', 'ped1')
        self.poses = {}; self.quat = {}; self.odom_v = 0.0; self.safety = {}
        # ros_gz 브리지는 Pose_V→TFMessage 변환에서 이름을 떨어뜨리므로 gz 트랜스포트로 직접 구독 (정답 포즈)
        self.gz = GzNode()
        assert self.gz.subscribe(Pose_V, f'/world/{self.world}/dynamic_pose/info', self.on_poses), 'gz subscribe failed'
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom_v', m.twist.twist.linear.x), 10)
        self.create_subscription(String, '/nanomob/safety', lambda m: setattr(self, 'safety', json.loads(m.data)), 10)
        self.pub_user = self.create_publisher(Twist, '/nanomob/user_cmd', 10)
        self.pub_ped = self.create_publisher(Twist, '/model/ped1/cmd_vel', 10)
        self.pub_veh = {n: self.create_publisher(Twist, f'/model/{n}/cmd_vel', 10) for n in ('bus1', 'car1')}
        self.vn = 'bus1'; self.vsign = 1.0
        self.rec = None; self.imu = None
        self.imu_buf = collections.deque(maxlen=5)   # 100 Hz × 5 = 50 ms 평균(접촉 순간 수치 스파이크 제거)
        self.create_subscription(Imu, '/imu', self.on_imu, 50)
        self.cli_pose = self.create_client(SetEntityPose, f'/world/{self.world}/set_pose')
        self.cli_param = self.create_client(SetParameters, '/safety_controller/set_parameters')
        for cli, nm in ((self.cli_pose, 'set_pose'), (self.cli_param, 'set_parameters')):
            while not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f'waiting for {nm} service...')

    # ---------- 유틸 ----------
    def on_poses(self, msg):
        for p in msg.pose:
            if p.name in (self.robot_name, self.ped_name, 'bus1', 'car1'):
                q = p.orientation
                self.poses[p.name] = (p.position.x, p.position.y, p.position.z, yaw_of(q)); self.quat[p.name] = (q.w, q.x, q.y, q.z)

    def on_imu(self, m):
        self.imu = m; a = m.linear_acceleration; self.imu_buf.append((a.x, a.z))

    def imu_avg(self):
        if not self.imu_buf:
            return 0.0, 9.81
        return sum(b[0] for b in self.imu_buf) / len(self.imu_buf), sum(b[1] for b in self.imu_buf) / len(self.imu_buf)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def sleep(self, dt):
        t0 = self.now()
        while self.now() - t0 < dt:
            rclpy.spin_once(self, timeout_sec=0.01)

    def set_pose(self, name, x, y, z, yaw, pitch=0.0):
        req = SetEntityPose.Request()
        req.entity.name = name; req.entity.type = 2  # MODEL
        req.pose.position.x = x; req.pose.position.y = y; req.pose.position.z = z
        cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)   # q = Rz(yaw)·Ry(pitch)
        req.pose.orientation.w = cy * cp; req.pose.orientation.x = -sy * sp; req.pose.orientation.y = cy * sp; req.pose.orientation.z = sy * cp
        fut = self.cli_pose.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        return fut.result() is not None and fut.result().success

    def set_params(self, **kw):
        req = SetParameters.Request()
        for k, v in kw.items():
            pv = ParameterValue()
            if isinstance(v, bool):
                pv.type = ParameterType.PARAMETER_BOOL; pv.bool_value = v
            elif isinstance(v, (list, tuple)):
                pv.type = ParameterType.PARAMETER_DOUBLE_ARRAY; pv.double_array_value = [float(x) for x in v]
            elif isinstance(v, (int, float)):
                pv.type = ParameterType.PARAMETER_DOUBLE; pv.double_value = float(v)
            else:
                pv.type = ParameterType.PARAMETER_STRING; pv.string_value = str(v)
            req.parameters.append(Parameter(name=k, value=pv))
        fut = self.cli_param.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)

    def cmd_user(self, v, wz=0.0, at_edge=False):
        m = Twist(); m.linear.x = float(v); m.angular.z = float(wz); m.linear.y = 1.0 if at_edge else 0.0; self.pub_user.publish(m)

    def cmd_ped(self, vx, vy):
        m = Twist(); m.linear.x = float(vx); m.linear.y = float(vy); self.pub_ped.publish(m)

    def set_rtf(self, rtf):
        """시뮬 속도 상한. 녹화 중에는 파이썬 인코더가 프레임을 놓치지 않도록 실시간 이하로 늦춘다."""
        req = Physics(); req.real_time_factor = float(rtf)
        ok, rep = self.gz.request(f'/world/{self.world}/set_physics', req, Physics, Boolean, 2000)
        self.get_logger().info(f'real_time_factor → {rtf} ({"ok" if ok else "failed"})')

    def place_cam(self, pos, look):
        yaw = math.atan2(look[1] - pos[1], look[0] - pos[0]); pit = math.atan2(pos[2] - look[2], math.hypot(look[0] - pos[0], look[1] - pos[1]))
        self.set_pose('spectator', pos[0], pos[1], pos[2], yaw, pit)

    def cmd_bus(self, vx):
        m = Twist(); m.linear.x = float(vx) * self.vsign; self.pub_veh[self.vn].publish(m)   # 후진 차량은 모델 +x 가 후방이라 부호를 뒤집음

    # ---------- E1: 차고지 출구에서 버스가 담장 틈으로 진출해 보도(진로)를 가로지름 ----------
    def run_trial_bus(self, ev, prof, d_app, v_bus, v_robot, seed, tag=''):
        """경로 좌표계: 도로 선분 p0→p1 방향 u(종방향 s), 오른쪽 법선 n(횡방향 l, 도로 쪽 +). 기기는 l=robot_l(차고지 쪽 길가),
        담장은 l=fence_l, 버스는 담장 틈(s=cross_s)에서 +n 방향으로 진출. 버스 길이 방향이 +n, 폭이 u 방향이다."""
        p0 = ev['road_p0']; p1 = ev['road_p1']
        ux, uy = p1[0] - p0[0], p1[1] - p0[1]; ln = math.hypot(ux, uy); ux, uy = ux / ln, uy / ln
        nx, ny = uy, -ux
        yaw_path = math.atan2(uy, ux); yaw_bus = math.atan2(ny, nx)
        self.vn = vn = ev.get('vehicle', 'bus1'); reverse = ev.get('reverse', False); self.vsign = -1.0 if reverse else 1.0
        yaw_model = yaw_bus + (math.pi if reverse else 0.0)
        rl_ = ev['robot_l']; fence_l = rl_ - ev.get('fence_gap', 1.8); cs = ev['cross_s']
        hide = ev.get('hide_depth', 1.0); BL, BW = ev.get('veh_length', ev.get('bus_length', 11.0)), ev.get('veh_width', ev.get('bus_width', 2.5))
        gap = ev.get('fence_opening', 7.0)
        L, W = self.cfg['robot']['length'], self.cfg['robot']['width']
        v_robot = prof.get('speed', v_robot)
        to_path = lambda x, y: ((x - p0[0]) * ux + (y - p0[1]) * uy, (x - p0[0]) * nx + (y - p0[1]) * ny)
        from_path = lambda s_, l_: (p0[0] + ux * s_ + nx * l_, p0[1] + uy * s_ + ny * l_)

        self.cmd_user(0.0); self.cmd_bus(0.0); rec_args = None; rec_started = False
        self.set_params(delay=prof['delay'], decel=prof['decel'], enabled=prof.get('enabled', True),
                        slope_comp=prof.get('slope_comp', True), curb_comp=prof.get('curb_comp', True))
        for nm in ('fence_a', 'fence_b', 'row_a', 'row_b'):    # 쓰지 않는 물체는 지하로 치움
            self.set_pose(nm, 0.0, 0.0, -10.0, 0.0)
        self.set_pose('car1' if vn == 'bus1' else 'bus1', 0.0, 0.0, -10.0, 0.0)
        occ = ev.get('occluders', ['fence_a', 'fence_b']); occ_l = fence_l - ev.get('occluder_depth', 0.0) / 2
        for nm, sc in ((occ[0], cs - gap / 2 - 6.0), (occ[1], cs + gap / 2 + 6.0)):
            x, y = from_path(sc, occ_l); self.set_pose(nm, x, y, 0.0, yaw_path)
        bl0 = fence_l - hide - BL / 2          # 버스 앞면이 담장선 뒤 hide m 에 놓인 초기 위치(중심)
        bx, by = from_path(cs, bl0); self.set_pose(vn, bx, by, 0.0, yaw_model)
        rx0, ry0 = from_path(ev['start_s'], rl_)
        if self.rec is not None:   # 관찰 카메라: 도로 쪽 위에서 교차점을 내려다봄
            cx_, cy_ = from_path(cs, rl_)
            camx, camy = from_path(cs + ev.get('cam_ds', -3.0), rl_ + ev.get('cam_dl', 10.0)); camz = ev.get('cam_h', 6.0)
            tx, ty = from_path(cs - 2.0, rl_ - 0.5)
            yaw_c = math.atan2(ty - camy, tx - camx); pit = math.atan2(camz - 0.5, math.hypot(tx - camx, ty - camy))
            self.set_pose('spectator', camx, camy, camz, yaw_c, pit)
            rec_args = (os.path.join(self.cfg.get('video_dir', 'results/videos'), tag + '.mp4'),
                        f"{ev.get('title', 'E1 버스 진출')} | {prof.get('label', '')} | 출현거리 {d_app:.1f} m · {ev.get('veh_name', '버스')} {v_bus:.1f} m/s ({v_bus * 3.6:.0f} km/h)")
        self.set_pose(self.robot_name, rx0, ry0, 0.26, yaw_path)
        self.sleep(1.0)
        self.set_pose(self.robot_name, rx0, ry0, 0.26, yaw_path)
        self.sleep(0.5)
        self.set_params(reset_rear=True)    # 이전 시행의 후방·턱 상태 초기화

        t0 = self.now(); triggered = False; emerged = None; s_emerge = None; v_emerge = None
        min_clear = float('inf'); collided = False; stop_dist = None; t_stop = None; bus_done = False
        peak_decel = 0.0; v_hist = []; result = 'timeout'; v_impact = None; v_min = None; t_end = None
        stop_front_l = ev.get('stop_front_l')   # 지정되면 버스 앞면이 그 횡위치에 닿을 때 정지(진로를 막은 채 대기, E1b)
        while self.now() - t0 < ev.get('timeout', 40.0):
            rclpy.spin_once(self, timeout_sec=0.005)
            if self.robot_name not in self.poses or vn not in self.poses:
                continue
            rx, ry, _, ryaw = self.poses[self.robot_name]; bx, by, _, _ = self.poses[vn]
            t = self.now(); rs, rl = to_path(rx, ry); bs, bl = to_path(bx, by); bus_front = bl + BL / 2
            yaw_err = (yaw_path - ryaw + math.pi) % (2 * math.pi) - math.pi
            wz = 1.5 * yaw_err - 0.8 * (rl_ - rl)
            self.cmd_user(v_robot, max(-0.8, min(0.8, wz)))
            if rec_args is not None and not rec_started and rs >= ev.get('rec_from_s', -1e9):
                self.rec.start(*rec_args); rec_started = True   # 달리기 시작 구간은 건너뛰고 접근 구간부터 녹화
            if self.cfg.get('trace') and emerged is None and int(t * 4) != getattr(self, '_lt0', -1):
                self._lt0 = int(t * 4); self.get_logger().info(f'pre t={t - t0:5.2f} rs={rs:6.2f} bus_front={bus_front:6.2f} v={self.odom_v:4.2f} {self.safety}')
            front_s = rs + L / 2
            # 버스 앞면이 담장선을 넘는(=시야에 나타나는) 순간 기기 앞범퍼-버스 근접면 거리 ≈ d_app 가 되도록 트리거 시점을 보정
            if not triggered and front_s >= cs - BW / 2 - d_app - hide / max(v_bus, 1e-3) * self.odom_v:
                triggered = True; self.cmd_bus(v_bus)
            if triggered and emerged is None and bus_front >= fence_l:
                emerged = t; s_emerge = rs; v_emerge = self.odom_v
            if triggered and not bus_done and stop_front_l is not None and bus_front >= stop_front_l:
                self.cmd_bus(0.0); bus_done = True
            if self.rec is not None:
                self.rec.set_info(f"t={t - t0:4.1f}s  기기 {self.odom_v:.2f} m/s  안전단계 {self.safety.get('level', '-')}  전방거리 {self.safety.get('dmin') if self.safety.get('dmin') is not None else '-'}")
            if emerged is not None:
                clear = rect_rect_clearance(rect_corners(rx, ry, ryaw, L, W), rect_corners(bx, by, yaw_bus, BL, BW))
                min_clear = min(min_clear, clear)
                v_min = self.odom_v if v_min is None else min(v_min, self.odom_v)
                if self.cfg.get('trace') and int(t * 10) != getattr(self, '_lt', -1):
                    self._lt = int(t * 10); self.get_logger().info(f't={t - t0:5.2f} rs={rs:6.2f} rl={rl:5.2f} bus_s={bs:5.2f} front_l={bus_front:6.2f} rear_l={bl - BL / 2:6.2f} v={self.odom_v:4.2f} clr={clear:5.2f} {self.safety}')
                if clear <= 0.0:
                    collided = True; v_impact = self.odom_v; result = 'collision'; t_end = t; break
                v_hist.append((t, self.odom_v))
                while v_hist and t - v_hist[0][0] > 0.2:
                    v_hist.pop(0)
                if len(v_hist) >= 2 and t - v_hist[0][0] > 0.1:
                    peak_decel = max(peak_decel, -(self.odom_v - v_hist[0][1]) / (t - v_hist[0][0]))
                if stop_dist is None and self.odom_v < 0.03 and t - emerged > 0.3:
                    stop_dist = rs - s_emerge; t_stop = t - emerged; result = 'stopped'
                if rs > cs + BW / 2 + 4.0:
                    result = 'passed' if stop_dist is None else 'yield_pass'; t_end = t; break
                if stop_dist is not None and stop_front_l is not None and t - emerged > t_stop + 1.5:
                    t_end = t; break
        self.cmd_user(0.0); self.cmd_bus(0.0)
        if self.rec is not None:
            txt = {'collision': ('충돌 — 충돌속도 %.2f m/s' % (v_impact or 0), (255, 90, 90, 255)),
                   'passed': ('무정지 통과 — 최소 이격 %.2f m' % min_clear, (140, 230, 140, 255)),
                   'yield_pass': ('정지 후 재개 — 정지거리 %.2f m, 최소 이격 %.2f m' % (stop_dist or 0, min_clear), (140, 230, 140, 255)),
                   'stopped': ('정지 — 정지거리 %.2f m, 최소 이격 %.2f m' % (stop_dist or 0, min_clear), (140, 230, 140, 255)),
                   'timeout': ('시간 초과', (255, 255, 255, 255))}[result]
            self.rec.stop(*txt)
        return dict(result=result, collision=int(collided), min_clear=round(min_clear, 3) if emerged else None,
                    stop_dist=round(stop_dist, 3) if stop_dist is not None else None,
                    t_stop=round(t_stop, 3) if t_stop is not None else None, peak_decel=round(peak_decel, 3),
                    v_emerge=round(v_emerge, 3) if emerged else None, emerged=int(emerged is not None),
                    v_impact=round(v_impact, 3) if v_impact is not None else None, v_min=round(v_min, 3) if v_min is not None else None)

    # ---------- E5b / C1: 턱·단차 — 높이 측정 → 넘을 수 있으면 감속 통과, 아니면 정지 ----------
    def run_trial_curb(self, ev, prof, v_robot, seed, tag=''):
        """레인을 +y 로 직진. 턱 앞면 y=curb_y, 높이 curb_h. 지표: 통과 여부, 턱 접촉 속도, 최대 pitch/roll, IMU 상하·전후 충격, 정지 시 턱까지 거리."""
        x, y0, cy = ev['lane_x'], ev['start_y'], ev['curb_y']; L = self.cfg['robot']['length']; yaw = math.pi / 2
        v_robot = prof.get('speed', v_robot)
        self.cmd_user(0.0)
        self.set_params(delay=prof['delay'], decel=prof['decel'], enabled=prof.get('enabled', True),
                        slope_comp=prof.get('slope_comp', True), curb_comp=prof.get('curb_comp', True),
                        h_pass=prof.get('h_pass', 0.035), curb_speed=prof.get('curb_speed', 1.0))
        self.set_pose(self.ped_name, 0.0, 0.0, -5.0, 0.0)
        self.set_pose(self.robot_name, x, y0, 0.26, yaw); self.sleep(1.0)
        self.set_pose(self.robot_name, x, y0, 0.26, yaw); self.sleep(0.5)
        self.set_params(reset_rear=True)
        if self.rec is not None and 'cam_pos' in ev:
            self.place_cam(ev['cam_pos'], ev['cam_look'])
            self.rec.start(os.path.join(self.cfg.get('video_dir', 'results/videos'), tag + '.mp4'),
                           f"{ev.get('title', 'E5 턱')} | {prof.get('label', '')} | 턱 높이 {ev['curb_h'] * 100:.0f} cm")
        t0 = self.now(); v_contact = None; max_pitch = 0.0; max_roll = 0.0; az_peak = 0.0; ax_peak = 0.0; result = 'timeout'
        prog_t = t0; prog_y = y0; stop_gap = None; t_stop = None
        while self.now() - t0 < ev.get('timeout', 25.0):
            rclpy.spin_once(self, timeout_sec=0.005)
            if self.robot_name not in self.poses:
                continue
            rx, ry, rz, ryaw = self.poses[self.robot_name]; t = self.now()
            self.cmd_user(v_robot, max(-0.8, min(0.8, 1.5 * ((yaw - ryaw + math.pi) % (2 * math.pi) - math.pi) - 0.8 * (rx - x))))
            front = ry + L / 2
            if v_contact is None and front >= cy - 0.02:
                v_contact = self.odom_v
            q = self.quat.get(self.robot_name)
            if q is not None:
                w, qx, qy, qz = q
                roll = math.atan2(2 * (w * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy)); pitch = math.asin(max(-1, min(1, 2 * (w * qy - qz * qx))))
                max_pitch = max(max_pitch, abs(pitch)); max_roll = max(max_roll, abs(roll))   # ZYX 오일러 → 차체 기준 pitch/roll
            if self.imu is not None and front > cy - 1.0:
                ax_, az_ = self.imu_avg()
                az_peak = max(az_peak, abs(az_ - 9.81)); ax_peak = max(ax_peak, abs(ax_))
            if self.rec is not None and 'cam_pos' in ev:
                sf = self.safety
                self.rec.set_info(f"t={t - t0:4.1f}s  기기 {self.odom_v:.2f} m/s  턱 인식 {sf.get('curb', '-')} {sf.get('curb_h', 0) * 100:.1f} cm  턱까지 {max(cy - front, 0):.2f} m  차체 기울기 {math.degrees(max_pitch):.1f}°")
            if max(max_pitch, max_roll) > math.radians(30):
                result = 'tipped'; break
            if ry > prog_y + 0.05:
                prog_y = ry; prog_t = t
            if ry - L / 2 > cy + 0.3:          # 뒷바퀴까지 턱 위로 올라감
                result = 'crossed'; break
            if t - prog_t > 3.0:
                if front < cy - 0.05 and self.odom_v < 0.03:
                    result = 'stopped'; stop_gap = cy - front     # 턱 앞에서 스스로 멈춤(우회 필요)
                else:
                    result = 'stuck'                               # 턱에 걸려 못 올라감
                break
        self.cmd_user(0.0)
        if self.rec is not None and 'cam_pos' in ev:
            self.rec.stop(*{'crossed': ('통과 — 접촉 속도 %.2f m/s, 상하 충격 %.1f m/s²' % (v_contact or 0, az_peak), (140, 230, 140, 255)),
                            'stopped': ('턱 앞 정지(우회 필요) — 턱까지 %.2f m' % (stop_gap or 0), (140, 200, 255, 255)),
                            'stuck': ('턱에 걸림 — 접촉 속도 %.2f m/s, 충격 %.1f m/s²' % (v_contact or 0, max(az_peak, ax_peak)), (255, 90, 90, 255)),
                            'tipped': ('전복 위험 — 기울기 %.0f°' % math.degrees(max(max_pitch, max_roll)), (255, 90, 90, 255)),
                            'timeout': ('시간 초과', (255, 255, 255, 255))}[result])
        r3 = lambda v: round(v, 3) if v is not None else None
        return dict(result=result, v_contact=r3(v_contact), max_pitch_deg=round(math.degrees(max_pitch), 2), max_roll_deg=round(math.degrees(max_roll), 2),
                    az_peak=round(az_peak, 2), ax_peak=round(ax_peak, 2), stop_gap=r3(stop_gap))

    # ---------- E4: 보도 없는 이면도로에서 뒤차 접근 (후방 라이다 → 길 가장자리로 붙고 감속·정지, 통과 후 재출발) ----------
    def run_trial_rear(self, ev, prof, gap0, v_car, v_robot, seed, lat_off, tag=''):
        """경로 좌표계: 도로 선분 p0→p1 방향 u(종방향 s), 오른쪽 법선 n(횡방향 l, 길 가장자리 쪽 +). 도로 중심선이 l=0.
        기기는 길 가장자리 쪽 l=lat_off 를 주행, 뒤차는 차선 중심 car_l 로 u 방향 등속 접근. gap0 = 트리거 순간 차 앞면~기기 뒤 범퍼 거리."""
        p0 = ev['road_p0']; p1 = ev['road_p1']
        ux, uy = p1[0] - p0[0], p1[1] - p0[1]; ln = math.hypot(ux, uy); ux, uy = ux / ln, uy / ln
        nx, ny = uy, -ux; yaw_path = math.atan2(uy, ux)
        BL, BW = ev.get('veh_length', 4.5), ev.get('veh_width', 1.8); car_l = ev['car_l']
        L, W = self.cfg['robot']['length'], self.cfg['robot']['width']; v_robot = prof.get('speed', v_robot)
        shift = max(0.0, ev['edge_l'] - ev.get('edge_buffer', 0.15) - W / 2 - lat_off)   # 지도상 길가장자리까지 붙을 수 있는 횡이동량
        trig_s = ev['trigger_s']; k_yaw, k_lat = ev.get('k_yaw', 2.5), ev.get('k_lat', 1.6)
        to_path = lambda x, y: ((x - p0[0]) * ux + (y - p0[1]) * uy, (x - p0[0]) * nx + (y - p0[1]) * ny)
        from_path = lambda s_, l_: (p0[0] + ux * s_ + nx * l_, p0[1] + uy * s_ + ny * l_)
        self.vn = 'car1'; self.vsign = 1.0
        self.cmd_user(0.0); self.cmd_bus(0.0); rec_args = None; rec_started = False
        self.set_params(delay=prof['delay'], decel=prof['decel'], enabled=prof.get('enabled', True),
                        slope_comp=prof.get('slope_comp', True), curb_comp=prof.get('curb_comp', True))
        for nm in ('fence_a', 'fence_b', 'row_a', 'row_b', 'bus1'):
            self.set_pose(nm, 0.0, 0.0, -10.0, 0.0)
        cx0, cy0 = from_path(trig_s - L / 2 - gap0 - BL / 2, car_l); self.set_pose('car1', cx0, cy0, 0.0, yaw_path)
        rx0, ry0 = from_path(ev['start_s'], lat_off)
        if self.rec is not None:
            camx, camy = from_path(ev['cam_s'] + ev.get('cam_ds', -4.0), ev.get('cam_l', -6.0)); camz = ev.get('cam_h', 5.0)
            tx, ty = from_path(ev['cam_s'], (car_l + lat_off) / 2)
            self.set_pose('spectator', camx, camy, camz, math.atan2(ty - camy, tx - camx), math.atan2(camz - 0.5, math.hypot(tx - camx, ty - camy)))
            rec_args = (os.path.join(self.cfg.get('video_dir', 'results/videos'), tag + '.mp4'),
                        f"{ev.get('title', 'E4 뒤차 접근')} | {prof.get('label', '')} | 뒤차 간격 {gap0:.0f} m · {v_car:.1f} m/s ({v_car * 3.6:.0f} km/h) · 기기 길가 {lat_off:.1f} m")
        self.set_pose(self.robot_name, rx0, ry0, 0.26, yaw_path); self.sleep(1.0)
        self.set_pose(self.robot_name, rx0, ry0, 0.26, yaw_path); self.sleep(0.5)
        self.set_params(reset_rear=True)    # 순간이동으로 생긴 후방 센서 이력 제거

        t0 = self.now(); triggered = False; lat_t = lat_off; t_prev = t0
        det = None; s_det = None; v_det = None; d_det = None; stop_dist = None; t_stop = None
        min_clear = float('inf'); collided = False; v_impact = None; v_min = None; v_hist = []; peak_decel = 0.0
        passed = None; l_pass = None; v_pass = None; gap_pass = None; max_shift = 0.0; result = 'timeout'; t_resume = None
        while self.now() - t0 < ev.get('timeout', 40.0):
            rclpy.spin_once(self, timeout_sec=0.005)
            if self.robot_name not in self.poses or 'car1' not in self.poses:
                continue
            rx, ry, _, ryaw = self.poses[self.robot_name]; cx, cy, _, _ = self.poses['car1']
            t = self.now(); dt = max(1e-3, t - t_prev); t_prev = t
            rs, rl = to_path(rx, ry); cs_, cl = to_path(cx, cy)
            pull = self.safety.get('rear') in ('PULL', 'STOP')
            goal = lat_off + (shift if pull else 0.0)   # 지도 기반 가장자리 목표(붙기), 뒤차가 지나가면 원래 횡위치로 복귀
            lat_t += max(-0.8 * dt, min(0.8 * dt, goal - lat_t))
            yaw_err = (yaw_path - ryaw + math.pi) % (2 * math.pi) - math.pi
            self.cmd_user(v_robot, max(-1.0, min(1.0, k_yaw * yaw_err - k_lat * (lat_t - rl))), at_edge=pull and abs(rl - (lat_off + shift)) < 0.12)
            if rec_args is not None and not rec_started and rs >= ev.get('rec_from_s', -1e9):
                self.rec.start(*rec_args); rec_started = True
            if not triggered and rs >= trig_s:
                triggered = True; self.cmd_bus(v_car)
            max_shift = max(max_shift, rl - lat_off)
            if self.cfg.get('trace') and int(t * 4) != getattr(self, '_lt', -1):
                self._lt = int(t * 4); self.get_logger().info(f't={t - t0:5.2f} rs={rs:6.2f} rl={rl:5.2f} car_s={cs_:6.2f} v={self.odom_v:4.2f} {self.safety}')
            gap = (rs - L / 2) - (cs_ + BL / 2)          # 뒤차 앞면 ~ 기기 뒤 범퍼 종방향 거리(양수 = 아직 뒤)
            if triggered and det is None and self.safety.get('rear') in ('PULL', 'STOP'):
                det = t; s_det = rs; v_det = self.odom_v; d_det = gap
            if self.rec is not None:
                self.rec.set_info(f"t={t - t0:4.1f}s  기기 {self.odom_v:.2f} m/s  후방감지 {self.safety.get('rear', '-')}  뒤차까지 {max(gap, 0.0):4.1f} m  기기 횡이동 {rl - lat_off:+.2f} m")
            if triggered:
                clear = rect_rect_clearance(rect_corners(rx, ry, ryaw, L, W), rect_corners(cx, cy, yaw_path, BL, BW))
                min_clear = min(min_clear, clear); v_min = self.odom_v if v_min is None else min(v_min, self.odom_v)
                if clear <= 0.0:
                    collided = True; v_impact = self.odom_v; result = 'collision'; break
                if passed is None and cs_ >= rs:       # 차 중심이 기기와 나란해진 순간의 측면 상태
                    passed = t; l_pass = rl - lat_off; v_pass = self.odom_v; gap_pass = clear
                v_hist.append((t, self.odom_v))
                while v_hist and t - v_hist[0][0] > 0.2:
                    v_hist.pop(0)
                if len(v_hist) >= 2 and t - v_hist[0][0] > 0.1:
                    peak_decel = max(peak_decel, -(self.odom_v - v_hist[0][1]) / (t - v_hist[0][0]))
                if det is not None and stop_dist is None and self.odom_v < 0.03:
                    stop_dist = rs - s_det; t_stop = t - det
                if passed is not None and cs_ - BL / 2 > rs + L / 2 + 1.0:   # 뒤차가 완전히 앞질러 감
                    if t_resume is None:
                        t_resume = t
                    if self.odom_v > 1.2 or t - t_resume > 8.0:
                        result = 'safe'; break
        self.cmd_user(0.0); self.cmd_bus(0.0)
        if self.rec is not None:
            txt = {'collision': ('충돌 — 충돌속도 %.2f m/s' % (v_impact or 0), (255, 90, 90, 255)),
                   'safe': ('무충돌 — 추월 시 측면 이격 %.2f m, 기기 횡이동 %.2f m' % (gap_pass if gap_pass is not None else min_clear, l_pass or 0.0), (140, 230, 140, 255)),
                   'timeout': ('시간 초과', (255, 255, 255, 255))}[result]
            self.rec.stop(*txt)
        r3 = lambda x: round(x, 3) if x is not None else None
        return dict(result=result, collision=int(collided), min_clear=r3(min_clear) if triggered else None, stop_dist=r3(stop_dist), t_stop=r3(t_stop),
                    peak_decel=round(peak_decel, 3), v_emerge=r3(v_det), emerged=int(det is not None), v_impact=r3(v_impact), v_min=r3(v_min),
                    shift=r3(l_pass), v_pass=r3(v_pass), d_det=r3(d_det), gap_pass=r3(gap_pass))

    # ---------- 한 번의 시행 ----------
    def run_trial(self, ev, prof, d_app, v_ped, v_robot, seed, tag=''):
        """경로 좌표계: 파사드 선분 p0→p1 방향 u(종방향 s), 오른쪽 법선 n(횡방향 l, 벽에서 멀어지는 쪽)."""
        rng = random.Random(seed)
        p0 = ev['facade_p0']; p1 = ev['facade_p1']
        ux, uy = p1[0] - p0[0], p1[1] - p0[1]; ln = math.hypot(ux, uy); ux, uy = ux / ln, uy / ln
        nx, ny = (uy, -ux) if not ev.get('mirror') else (-uy, ux)   # mirror: 벽이 기기 오른쪽(보행자가 오른쪽에서 나옴)
        yaw_path = math.atan2(uy, ux); yaw_ped = math.atan2(ny, nx)
        offset = ev['path_offset']; door_s = ev['door_s'] + rng.uniform(-ev.get('door_jitter', 0.0), ev.get('door_jitter', 0.0))
        hide_depth = ev.get('hide_depth', 1.0); ped_r = ev.get('ped_radius', 0.22)
        L, W = self.cfg['robot']['length'], self.cfg['robot']['width']
        v_robot = prof.get('speed', v_robot)
        to_path = lambda x, y: ((x - p0[0]) * ux + (y - p0[1]) * uy, (x - p0[0]) * nx + (y - p0[1]) * ny)
        from_path = lambda s_, l_: (p0[0] + ux * s_ + nx * l_, p0[1] + uy * s_ + ny * l_)

        # 리셋
        self.cmd_user(0.0); self.cmd_ped(0.0, 0.0)
        self.set_params(delay=prof['delay'], decel=prof['decel'], enabled=prof.get('enabled', True),
                        slope_comp=prof.get('slope_comp', True), curb_comp=prof.get('curb_comp', True))
        pxs, pys = from_path(door_s, -hide_depth)
        z0 = ev.get('start_z', 0.0) + 0.26   # 경사 레인 상단 평지 위에서 출발(E5)
        self.set_pose(self.ped_name, pxs, pys, ev.get('ped_z', 0.0), yaw_ped)
        rxs, rys = from_path(ev['start_s'], offset)
        self.set_pose(self.robot_name, rxs, rys, z0, yaw_path)
        self.sleep(1.0)
        self.set_pose(self.robot_name, rxs, rys, z0, yaw_path)
        self.sleep(0.5)
        self.set_params(reset_rear=True)    # 이전 시행의 후방·턱 상태 초기화
        if self.rec is not None and 'cam_pos' in ev:
            self.place_cam(ev['cam_pos'], ev['cam_look'])
            self.rec.start(os.path.join(self.cfg.get('video_dir', 'results/videos'), tag + '.mp4'),
                           f"{ev.get('title', 'E2 출입구 출현')} | {prof.get('label', '')} | 출현거리 {d_app:.1f} m · 보행자 {v_ped:.1f} m/s")

        t0 = self.now(); triggered = False; emerged = None; s_emerge = None; v_emerge = None
        min_clear = float('inf'); collided = False; stop_dist = None; t_stop = None
        peak_decel = 0.0; v_hist = []; ped_done = False; result = 'timeout'; v_impact = None; v_min = None
        while self.now() - t0 < ev.get('timeout', 25.0):
            rclpy.spin_once(self, timeout_sec=0.005)
            if self.robot_name not in self.poses or self.ped_name not in self.poses:
                continue
            rx, ry, _, ryaw = self.poses[self.robot_name]; px, py, _, _ = self.poses[self.ped_name]
            t = self.now()
            rs, rl = to_path(rx, ry); ps, pl = to_path(px, py)
            # 경로 추종: 파사드 평행 직진, 횡오차 P 제어 (lat_err>0 → 오른쪽(+n)으로 → wz<0)
            yaw_err = (yaw_path - ryaw + math.pi) % (2 * math.pi) - math.pi
            lat_err = offset - rl
            wz = 1.5 * yaw_err - 0.8 * lat_err
            self.cmd_user(v_robot, max(-0.8, min(0.8, wz)))
            # 트리거: 보행자가 벽을 뚫고 나오는 시간(hide_depth/v_ped) 동안 로봇이 전진할 거리를 보정해
            # 출현 순간 앞범퍼-출입구 종방향 거리 ≈ d_app 가 되게 함
            front_s = rs + L / 2
            if not triggered and front_s >= door_s - d_app - hide_depth / max(v_ped, 1e-3) * self.odom_v:
                triggered = True; self.cmd_ped(v_ped, 0.0)
            if triggered and emerged is None and pl >= 0.0:
                emerged = t; s_emerge = rs; v_emerge = self.odom_v
            if self.rec is not None and 'cam_pos' in ev:
                sf = self.safety
                self.rec.set_info(f"t={t - t0:4.1f}s  기기 {self.odom_v:.2f} m/s  경사 {sf.get('pitch', 0):+.1f}°  계획감속 {sf.get('a_plan', '-')} m/s²  안전단계 {sf.get('level', '-')}")
            stop_l = ev.get('stop_at_l')   # 지정되면 보행자가 그 횡위치(진로 위)에서 멈춰 섬 → 정지거리 측정용
            if triggered and not ped_done and pl >= (stop_l if stop_l is not None else ev.get('cross_width', 3.0)):
                self.cmd_ped(0.0, 0.0); ped_done = True
            if emerged is not None:
                clear = rect_circle_clearance(rx, ry, ryaw, L, W, px, py, ped_r)
                min_clear = min(min_clear, clear)
                v_min = self.odom_v if v_min is None else min(v_min, self.odom_v)
                if clear <= 0.0:
                    collided = True; v_impact = self.odom_v; result = 'collision'
                    break
                v_hist.append((t, self.odom_v))
                while v_hist and t - v_hist[0][0] > 0.2:
                    v_hist.pop(0)
                if len(v_hist) >= 2 and t - v_hist[0][0] > 0.1:
                    peak_decel = max(peak_decel, -(self.odom_v - v_hist[0][1]) / (t - v_hist[0][0]))
                if stop_dist is None and self.odom_v < 0.03 and t - emerged > 0.3:
                    stop_dist = rs - s_emerge; t_stop = t - emerged; result = 'stopped'
                if rs > door_s + 6.0:
                    result = 'passed' if stop_dist is None else result
                    break
                if stop_dist is not None and ped_done and t - emerged > t_stop + 1.0:
                    break
        self.cmd_user(0.0); self.cmd_ped(0.0, 0.0)
        if self.rec is not None and 'cam_pos' in ev:
            self.rec.stop(*{'collision': ('충돌 — 충돌속도 %.2f m/s' % (v_impact or 0), (255, 90, 90, 255)),
                            'stopped': ('정지 — 정지거리 %.2f m, 최소 이격 %.2f m' % (stop_dist or 0, min_clear), (140, 230, 140, 255)),
                            'passed': ('통과', (255, 255, 255, 255)), 'timeout': ('시간 초과', (255, 255, 255, 255))}[result])
        return dict(result=result, collision=int(collided), min_clear=round(min_clear, 3) if emerged else None,
                    stop_dist=round(stop_dist, 3) if stop_dist is not None else None,
                    t_stop=round(t_stop, 3) if t_stop is not None else None, peak_decel=round(peak_decel, 3),
                    v_emerge=round(v_emerge, 3) if emerged else None, emerged=int(emerged is not None),
                    v_impact=round(v_impact, 3) if v_impact is not None else None, v_min=round(v_min, 3) if v_min is not None else None)

    def run_all(self):
        ev0 = self.cfg['event']; out = self.cfg.get('output', 'results/results.csv')
        os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
        new = not os.path.exists(out)
        typ = ev0['type']; rear = typ == 'rear_approach'; curb = typ == 'curb'
        variants = ev0.get('variants', [{}])   # 레인·조건 변형(E5: 경사별/턱 높이별)
        fields = ['event', 'profile', 'delay', 'decel', 'd_app', 'v_ped', 'v_robot', 'seed', 'result', 'collision', 'min_clear', 'stop_dist', 't_stop', 'peak_decel', 'v_emerge', 'emerged', 'v_impact', 'v_min']
        if 'variants' in ev0:
            fields.insert(1, 'variant')
        if rear:
            fields += ['robot_l', 'shift', 'v_pass', 'd_det', 'gap_pass']
        if curb:
            fields = ['event', 'variant', 'curb_h', 'profile', 'delay', 'v_robot', 'seed', 'result', 'v_contact', 'max_pitch_deg', 'max_roll_deg', 'az_peak', 'ax_peak', 'stop_gap']
        lats = ev0.get('robot_l_list', [0.0]) if rear else [None]
        if self.cfg.get('controller_params'):   # 센서 장착 위치 등(URDF 인자와 같게) — 배치 시작 때 한 번
            self.set_params(**self.cfg['controller_params'])
        n = 0; total = len(variants) * len(self.cfg['profiles']) * len(ev0.get('appearance_distances', [0])) * len(ev0.get('ped_speeds', [0])) * len(lats) * self.cfg.get('repeats', 1)
        with open(out, 'a', newline='') as f:
            w = csv.DictWriter(f, fieldnames=fields, restval='')
            if new:
                w.writeheader()
            for var in variants:
                ev = {**ev0, **var}; vname = var.get('name', '')
                for pname, prof in self.cfg['profiles'].items():
                    prof = dict(prof, label=prof.get('label', pname))
                    for lat in lats:
                        for d_app in ev.get('appearance_distances', [0]):
                            for v_ped in ev.get('ped_speeds', [0]):
                                for rep in range(self.cfg.get('repeats', 1)):
                                    seed = self.cfg.get('seed', 0) * 1000 + rep
                                    tag = f"{os.path.splitext(os.path.basename(out))[0]}" + (f"_{vname}" if vname else '') + f"_{pname}_d{d_app:g}_v{v_ped:g}" + (f"_l{lat:g}" if rear else '') + f"_r{rep}"
                                    extra = {}
                                    if curb:
                                        r = self.run_trial_curb(ev, prof, self.cfg['robot']['speed'], seed, tag)
                                        row = dict(event=typ, variant=vname, curb_h=ev['curb_h'], profile=pname, delay=prof['delay'],
                                                   v_robot=prof.get('speed', self.cfg['robot']['speed']), seed=seed, **r)
                                    else:
                                        if rear:   # d_app 열 = 트리거 순간 뒤차 간격, v_ped 열 = 뒤차 속도
                                            r = self.run_trial_rear(ev, prof, d_app, v_ped, self.cfg['robot']['speed'], seed, lat, tag)
                                            extra = dict(robot_l=lat)
                                        elif typ in ('bus_exit', 'car_reverse'):   # v_ped 열 = 상대 차량 속도
                                            r = self.run_trial_bus(ev, prof, d_app, v_ped, self.cfg['robot']['speed'], seed, tag)
                                        else:
                                            r = self.run_trial(ev, prof, d_app, v_ped, self.cfg['robot']['speed'], seed, tag)
                                        if 'variants' in ev0:
                                            extra['variant'] = vname
                                        row = dict(event=typ, profile=pname, delay=prof['delay'], decel=prof['decel'], d_app=d_app, v_ped=v_ped,
                                                   v_robot=prof.get('speed', self.cfg['robot']['speed']), seed=seed, **r, **extra)
                                    w.writerow(row); f.flush(); n += 1
                                    self.get_logger().info(f'[{n}/{total}] {vname} {pname} d_app={d_app} v_ped={v_ped}' + (f' lat={lat}' if rear else '') + f' → {r}')
        self.get_logger().info(f'done → {out}')


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--ros-args')]
    cfg_path = args[0] if args else os.path.join(os.path.dirname(__file__), '..', 'config', 'scenarios', 'E2_doorway.yaml')
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)
    rclpy.init()
    n = Runner(cfg)
    if cfg.get('record'):
        from nanomob_sim.recorder import Recorder
        n.rec = Recorder(); n.set_rtf(cfg.get('record_rtf', 0.8))
    n.set_parameters([RclParam('use_sim_time', RclParam.Type.BOOL, True)])
    try:
        n.run_all()
        if n.rec is not None:
            n.set_rtf(5.0)
    except KeyboardInterrupt:
        pass
    n.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
