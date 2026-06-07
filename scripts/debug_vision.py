#!/usr/bin/env python3
"""
debug_vision.py — 비전 파이프라인 실시간 디버그 모니터

구독 토픽:
  /camera/image              RGB 이미지 수신 주기 측정
  /camera/depth_image        깊이 이미지 수신 주기 측정
  /camera/image/camera_info  카메라 내부 파라미터 (1회)
  /vision/detection_results  YOLO 탐지 결과 JSON

실행:
  python3 scripts/debug_vision.py
  python3 scripts/debug_vision.py --save-log
  python3 scripts/debug_vision.py --save-log --log-dir /tmp/debug_logs
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
import json
import time
import os
import argparse
from datetime import datetime
from collections import deque

# ANSI 색상
_R = '\033[91m'; _G = '\033[92m'; _Y = '\033[93m'
_C = '\033[96m'; _b = '\033[1m';  _d = '\033[2m'; _n = '\033[0m'

def red(s):    return f"{_R}{s}{_n}"
def green(s):  return f"{_G}{s}{_n}"
def yellow(s): return f"{_Y}{s}{_n}"
def cyan(s):   return f"{_C}{s}{_n}"
def bold(s):   return f"{_b}{s}{_n}"
def dim(s):    return f"{_d}{s}{_n}"

def strip_ansi(text: str) -> str:
    for esc in (_R, _G, _Y, _C, _b, _d, _n):
        text = text.replace(esc, '')
    return text


class VisionDebugNode(Node):
    def __init__(self, log_fp=None):
        super().__init__('vision_debug_monitor')
        self._log_fp       = log_fp
        self._img_times    = deque(maxlen=30)
        self._depth_times  = deque(maxlen=30)
        self._info_received = False
        self._event_count  = 0

        self.create_subscription(Image,      '/camera/image',             self._on_img,   10)
        self.create_subscription(Image,      '/camera/depth_image',       self._on_depth, 10)
        self.create_subscription(CameraInfo, '/camera/image/camera_info', self._on_info,   1)
        self.create_subscription(String,     '/vision/detection_results', self._on_det,   10)
        self.create_timer(5.0, self._status_tick)

        self._print([
            '',
            f"{'━'*54}",
            f"  Vision Debug Monitor  {datetime.now():%Y-%m-%d %H:%M:%S}",
            f"  /camera/image  /camera/depth_image",
            f"  /camera/image/camera_info  /vision/detection_results",
            f"{'━'*54}",
            '',
        ])

    # ─────────────────────────────────────────────────────────────
    def _ts(self) -> str:
        return datetime.now().strftime('%H:%M:%S.%f')[:-3]

    def _print(self, lines):
        for l in lines:
            print(l)
            if self._log_fp:
                self._log_fp.write(f"[{self._ts()}] {strip_ansi(l)}\n")

    def _hz(self, dq) -> float | None:
        now = time.time()
        recent = [t for t in dq if now - t < 2.0]
        if len(recent) < 2:
            return None
        return (len(recent) - 1) / (recent[-1] - recent[0])

    # ─────────────────────────────────────────────────────────────
    def _on_img(self, _):   self._img_times.append(time.time())
    def _on_depth(self, _): self._depth_times.append(time.time())

    def _on_info(self, msg: CameraInfo):
        if self._info_received:
            return
        self._info_received = True
        K = msg.k
        self._print([
            '',
            f"{bold('[카메라 내부파라미터]')}",
            f"  fx={K[0]:.2f}  fy={K[4]:.2f}  cx={K[2]:.2f}  cy={K[5]:.2f}",
            f"  해상도: {msg.width}×{msg.height}",
        ])

    def _on_det(self, msg: String):
        self._event_count += 1
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self._print([red(f"[{self._ts()}] JSON 파싱 오류: {e}")])
            return

        n    = data.get('num_detections', 0)
        objs = data.get('objects', [])

        lines = [
            '',
            f"{bold(cyan(f'[{self._ts()}]'))} {'━'*18} DETECTION #{self._event_count} {'━'*18}",
        ]

        if n == 0:
            lines.append(yellow("  탐지 없음  — 카메라 시야·조명·YOLO 모델 확인"))
            self._print(lines)
            return

        lines.append(f"  탐지 수: {bold(str(n))}")

        for i, obj in enumerate(objs):
            cls  = obj.get('class_name', '?')
            conf = obj.get('confidence', 0.0)
            c2d  = obj.get('center_2d', {})
            dist = obj.get('distance_m', -1.0)
            pb   = obj.get('position_3d_base_frame', {})
            pc   = obj.get('position_3d_camera_frame', {})

            u, v = c2d.get('u', '?'), c2d.get('v', '?')
            bx   = pb.get('X')
            by   = pb.get('Y')
            bz   = pb.get('Z')

            cls_s  = green(bold(cls)) if cls == 'bottle' else bold(cls)
            conf_s = (green if conf >= 0.80 else yellow)(f'{conf:.3f}')

            coord_ok  = bx is not None and by is not None and bz is not None
            in_range  = (coord_ok
                         and 0.25 <= bx <= 0.90
                         and 0.25 <= bz <= 0.55)

            status_s = (green("✔ shelf 범위 내") if in_range
                        else yellow("⚠ shelf 범위 밖") if coord_ok
                        else red("✘ TF 변환 실패 — camera_link TF 확인"))

            dist_s = cyan(f'{dist:.3f} m') if dist > 0 else red('N/A')
            sep    = '┌' if i == 0 else '├'

            block = [
                f"  {sep}─ #{i}  {cls_s}  (conf: {conf_s})",
                f"  │  2D: ({u}, {v}) px    depth: {dist_s}",
            ]
            if coord_ok:
                block.append(
                    f"  │  base_link: X={bold(f'{bx:.3f}')}  "
                    f"Y={bold(f'{by:.3f}')}  Z={bold(f'{bz:.3f}')}"
                )
                if pc.get('X') is not None:
                    block.append(
                        f"  │  cam_frame: X={pc['X']:.3f}  "
                        f"Y={pc['Y']:.3f}  Z={pc['Z']:.3f}"
                    )
            else:
                block.append(f"  │  base_link: {red('좌표 없음')}")
            block.append(f"  └─ {status_s}")
            lines.extend(block)

        self._print(lines)

    def _status_tick(self):
        img_hz   = self._hz(self._img_times)
        depth_hz = self._hz(self._depth_times)

        def hz_s(hz):
            if hz is None:
                return red("수신 없음")
            return (green if hz >= 5.0 else yellow)(f'{hz:.1f} Hz')

        info_s = green("수신됨") if self._info_received else yellow("대기 중")

        self._print([
            dim(f"\n[토픽 상태]  image: {hz_s(img_hz)}  "
                f"depth: {hz_s(depth_hz)}  "
                f"camera_info: {info_s}  "
                f"탐지 이벤트: {self._event_count}회"),
        ])

    def destroy_node(self):
        if self._log_fp:
            self._log_fp.write(f"[{self._ts()}] === Session End ===\n")
            self._log_fp.close()
        super().destroy_node()


def main():
    ap = argparse.ArgumentParser(description='비전 파이프라인 실시간 디버그 모니터')
    ap.add_argument('--save-log',  action='store_true', help='파일에 로그 저장')
    ap.add_argument('--log-dir', default=None,
                    help='로그 저장 디렉터리 (기본: Arm_Module_Assembler/logs/)')
    args = ap.parse_args()

    log_fp = None
    if args.save_log:
        log_dir = args.log_dir or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), '..', 'logs')
        os.makedirs(log_dir, exist_ok=True)
        fname = os.path.join(log_dir, f"vision_{datetime.now():%Y%m%d_%H%M%S}.log")
        log_fp = open(fname, 'a', buffering=1)
        print(f"로그 파일: {fname}")

    rclpy.init()
    node = VisionDebugNode(log_fp=log_fp)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(dim('\n[종료] Ctrl+C'))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
