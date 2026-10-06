#!/usr/bin/env python3
"""구동계 제동 한계 모델(시뮬 플랜트). 안전 제어기 명령(/nanomob/cmd_safe) → /cmd_vel.
gz DiffDrive 는 바퀴 속도를 이상적으로 추종해 경사에서도 제동거리가 늘지 않으므로, 실제 기기의 제동력 한계를 여기서 재현한다:
  내리막(차체 앞쪽이 숙여짐, pitch>0)에서 낼 수 있는 감속도 = brake_cap − g·sin(pitch)
평지(pitch=0)에서는 brake_cap(2.0) ≥ 제어기 감속도(1.5)라 명령이 그대로 통과한다.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

G = 9.81


def pitch_of(q):
    return math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))   # +: 앞이 숙여짐(내리막)


class SlopePlant(Node):
    def __init__(self):
        super().__init__('slope_plant')
        self.declare_parameter('brake_cap', 2.0)   # 평지 최대 제동 감속도 [m/s²] (타이어·모터 한계, 가정)
        self.pitch = 0.0; self.v = 0.0; self.last = None
        self.create_subscription(Imu, '/imu', lambda m: setattr(self, 'pitch', pitch_of(m.orientation)), 20)
        self.create_subscription(Twist, '/nanomob/cmd_safe', self.on_cmd, 20)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 20)

    def on_cmd(self, m):
        t = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.02 if self.last is None else max(1e-3, min(0.1, t - self.last)); self.last = t
        a_avail = max(0.1, self.get_parameter('brake_cap').value - G * math.sin(self.pitch))
        v_in = m.linear.x
        self.v = max(v_in, self.v - a_avail * dt) if v_in < self.v else v_in
        out = Twist(); out.linear.x = self.v; out.angular.z = m.angular.z
        self.pub.publish(out)


def main():
    rclpy.init(); n = SlopePlant()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
