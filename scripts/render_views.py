#!/usr/bin/env python3
"""관찰 카메라(spectator)를 지정 지점으로 옮겨 맵 렌더링 캡처 → results/map_views/*.png
사용: 시뮬을 띄운 상태에서 python3 scripts/render_views.py [출력 폴더]
"""
import math, os, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ros_gz_interfaces.srv import SetEntityPose
from PIL import Image as PImg

VIEWS = {   # 이름: (카메라 위치 x,y,z, 바라볼 점 x,y,z) — 가상 시가지 좌표
    '00_수직_전체': ((100, 20, 190), (100, 20.5, 0)),
    '01_전체_남서쪽상공': ((-40, -110, 80), (100, 20, 0)),
    'A_건물출입구_E2': ((9, 15, 5), (0, 25, 1)),
    'B_버스차고지_E1': ((66, 8, 8), (52, 20, 0)),
    'C_주차장보행로_E3': ((112, -72, 9), (112, -55, 0)),
    'D_이면도로_E4': ((200, -12, 9), (200, 20, 1)),
    'E_경사턱레인_E5': ((105, -40, 18), (122, 5, 0)),
}


class R(Node):
    def __init__(self):
        super().__init__('render_views'); self.img = None
        self.create_subscription(Image, '/spectator/image', lambda m: setattr(self, 'img', m), 5)
        self.cli = self.create_client(SetEntityPose, '/world/nanomob_town/set_pose')
        self.cli.wait_for_service(timeout_sec=10)

    def place(self, pos, look):
        yaw = math.atan2(look[1] - pos[1], look[0] - pos[0]); pit = math.atan2(pos[2] - look[2], math.hypot(look[0] - pos[0], look[1] - pos[1]))
        q = SetEntityPose.Request(); q.entity.name = 'spectator'; q.entity.type = 2
        q.pose.position.x, q.pose.position.y, q.pose.position.z = map(float, pos)
        cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pit / 2), math.sin(pit / 2)
        q.pose.orientation.w = cy * cp; q.pose.orientation.x = -sy * sp; q.pose.orientation.y = cy * sp; q.pose.orientation.z = sy * cp
        f = self.cli.call_async(q); rclpy.spin_until_future_complete(self, f, timeout_sec=5)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'results', 'map_views')
    os.makedirs(out, exist_ok=True); rclpy.init(); n = R()
    for name, (pos, look) in VIEWS.items():
        n.place(pos, look); n.img = None; t0 = time.time()
        while time.time() - t0 < 2.5 or n.img is None:   # 자리 옮긴 뒤 몇 프레임 버림
            rclpy.spin_once(n, timeout_sec=0.05)
        m = n.img; a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
        PImg.fromarray(a).save(os.path.join(out, name + '.png')); print('saved', name)
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
