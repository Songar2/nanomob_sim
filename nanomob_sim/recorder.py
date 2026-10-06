"""관찰 카메라(/spectator/image) → mp4 녹화. 러너와 같은 프로세스에서 별도 스레드로 돌며, 시행 단위로 start/stop 한다.
프레임은 시뮬 시간 30 Hz 로 도착하므로 mp4 도 30 fps 로 쓰면 실시간 배속과 일치. 한글 자막은 PIL 로 그린다.
"""
import os, subprocess, threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from PIL import Image as PILImage, ImageDraw, ImageFont
import imageio_ffmpeg

FONT = '/usr/share/fonts/truetype/nanum/NanumGothic.ttf'
FPS = 30


class Recorder(Node):
    def __init__(self):
        super().__init__('video_recorder')
        self.sub = self.create_subscription(Image, '/spectator/image', self.on_image, 100)
        self.lock = threading.Lock()
        self.proc = None; self.label = ''; self.info = ''; self.last_t = None; self.last_frame = None
        self.f_big = ImageFont.truetype(FONT, 26); self.f_small = ImageFont.truetype(FONT, 22)
        self.exec = SingleThreadedExecutor(); self.exec.add_node(self)
        self.th = threading.Thread(target=self.exec.spin, daemon=True); self.th.start()

    def start(self, path, label):
        with self.lock:
            self.label = label; self.info = ''; self.last_t = None; self.last_frame = None; self.size = None
            os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
            self.path = path; self.proc = 'pending'

    def set_info(self, info):
        self.info = info

    def _open(self, w, h):
        self.size = (w, h)
        self.proc = subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(), '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'bgr24',
                                      '-s', f'{w}x{h}', '-r', str(FPS), '-i', '-', '-an', '-vcodec', 'libx264', '-preset', 'fast',
                                      '-crf', '20', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', self.path], stdin=subprocess.PIPE)

    def _draw(self, rgb, banner=None):
        im = PILImage.fromarray(rgb); d = ImageDraw.Draw(im, 'RGBA'); w, h = im.size
        d.rectangle([0, 0, w, 78], fill=(0, 0, 0, 150))
        d.text((16, 6), self.label, font=self.f_big, fill=(255, 255, 255, 255))
        d.text((16, 42), self.info, font=self.f_small, fill=(255, 225, 120, 255))
        if banner:
            d.rectangle([0, h - 90, w, h], fill=(0, 0, 0, 190))
            d.text((16, h - 70), banner[0], font=self.f_big, fill=banner[1])
        return np.asarray(im)[:, :, ::-1]

    def on_image(self, msg):
        with self.lock:
            if self.proc is None:
                return
            rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
            if self.proc == 'pending':
                self._open(msg.width, msg.height)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if self.last_t is not None and t - self.last_t > 1.5 / FPS:     # 누락 프레임은 직전 프레임으로 채움
                for _ in range(int(round((t - self.last_t) * FPS)) - 1):
                    self.proc.stdin.write(self.last_frame.tobytes())
            frame = np.ascontiguousarray(self._draw(rgb))
            self.proc.stdin.write(frame.tobytes()); self.last_t = t; self.last_frame = frame; self.raw = rgb

    def stop(self, banner_text='', color=(255, 255, 255, 255), hold=1.5):
        with self.lock:
            p = self.proc; self.proc = None
            if p is None or p == 'pending':
                return
            if banner_text and getattr(self, 'raw', None) is not None:
                fr = np.ascontiguousarray(self._draw(self.raw, (banner_text, color)))
                for _ in range(int(hold * FPS)):
                    p.stdin.write(fr.tobytes())
            p.stdin.close(); p.wait()
