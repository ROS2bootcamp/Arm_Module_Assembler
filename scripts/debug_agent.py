#!/usr/bin/env python3
"""
debug_agent.py — LLM Agent / MoveIt 모듈 실시간 상태 모니터

구독 토픽:
  /llm_agent/log      Agent P1~P4 단계 로그 (String)
  /moveit_module/log  MoveIt 실행 로그 (String)
  /moveit_status      서비스 응답 상태 JSON (String)
  /rosout             ROS 시스템 로그 (agent_node / yolo_detector WARN/ERROR 필터)

실행:
  python3 scripts/debug_agent.py
  python3 scripts/debug_agent.py --save-log
  python3 scripts/debug_agent.py --save-log --log-dir /tmp/debug_logs
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import Log
import json
import os
import argparse
from datetime import datetime

# ANSI 색상
_R = '\033[91m'; _G = '\033[92m'; _Y = '\033[93m'; _B = '\033[94m'
_M = '\033[95m'; _C = '\033[96m'; _b = '\033[1m';  _d = '\033[2m'; _n = '\033[0m'

def red(s):     return f"{_R}{s}{_n}"
def green(s):   return f"{_G}{s}{_n}"
def yellow(s):  return f"{_Y}{s}{_n}"
def blue(s):    return f"{_B}{s}{_n}"
def magenta(s): return f"{_M}{s}{_n}"
def cyan(s):    return f"{_C}{s}{_n}"
def bold(s):    return f"{_b}{s}{_n}"
def dim(s):     return f"{_d}{s}{_n}"

def strip_ansi(text: str) -> str:
    for esc in (_R, _G, _Y, _B, _M, _C, _b, _d, _n):
        text = text.replace(esc, '')
    return text

# 단계별 스타일
_PHASE_STYLE = {
    'P1': (cyan,    '🔍 P1 [탐색]'),
    'P2': (blue,    '🤏 P2 [집기]'),
    'P3': (magenta, '⬆  P3 [들기]'),
    'P4': (green,   '📦 P4 [놓기]'),
}


def _detect_phase(text: str) -> str | None:
    for p in ('P1', 'P2', 'P3', 'P4'):
        if f'{p}:' in text:
            return p
    return None


class AgentDebugNode(Node):
    def __init__(self, log_fp=None):
        super().__init__('agent_debug_monitor')
        self._log_fp = log_fp

        self.create_subscription(String, '/llm_agent/log',     self._on_agent_log,  20)
        self.create_subscription(String, '/moveit_module/log', self._on_moveit_log, 20)
        self.create_subscription(String, '/moveit_status',     self._on_status,     10)
        self.create_subscription(Log,    '/rosout',            self._on_rosout,     50)

        self._print([
            '',
            f"{'━'*54}",
            f"  Agent Debug Monitor  {datetime.now():%Y-%m-%d %H:%M:%S}",
            f"  /llm_agent/log  /moveit_module/log  /moveit_status",
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

    def _status_icon(self, text: str) -> str:
        tl = text.lower()
        if any(w in tl for w in ('성공', 'success', 'detected', 'confirmed', 'complete')):
            return green('✔')
        if any(w in tl for w in ('실패', 'fail', 'timeout', 'error', 'not detected')):
            return red('✘')
        if any(w in tl for w in ('retry', '재시작', 'warn')):
            return yellow('⚠')
        return dim('→')

    # ─────────────────────────────────────────────────────────────
    def _on_agent_log(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        # 세션 시작·종료 구분선
        if 'Session start' in text:
            self._print([
                '',
                f"{'━'*54}",
                f"  {bold('▶ NEW SESSION')}  {cyan(text)}",
                f"{'━'*54}",
            ])
            return

        if any(w in text for w in ('세션 완료', '최종 실패', '예외 발생')):
            end_s = green(bold(f'● {text}')) if '완료' in text else red(bold(f'● {text}'))
            self._print([f"  {end_s}", f"{'─'*54}", ''])
            return

        phase = _detect_phase(text)
        icon  = self._status_icon(text)

        if phase and phase in _PHASE_STYLE:
            col, label = _PHASE_STYLE[phase]
            phase_s = col(bold(f'[{label}]'))
            self._print([f"  {icon} {phase_s}  {text}"])
        else:
            self._print([f"  {icon} {text}"])

    def _on_moveit_log(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        icon = (green('✔') if 'success=True' in text
                else red('✘')   if 'success=False' in text or 'fail' in text.lower()
                else dim('·'))
        self._print([f"  {icon} {dim('[MoveIt]')} {text}"])

    def _on_status(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        cmd     = data.get('cmd_echo', data.get('cmd', '?'))
        success = data.get('success', False)
        err     = data.get('error_message', '')
        ec      = data.get('error_code', 0)
        phase   = data.get('phase', '')

        result_s = (
            green('SUCCESS')
            if success
            else red(f'FAIL  code={ec}  {err}')
        )
        phase_s = f"  phase={phase}" if phase else ""

        self._print([
            '',
            f"  {bold(yellow('[서비스 응답]'))} "
            f"cmd={bold(cmd)}{phase_s}  →  {result_s}",
        ])

    def _on_rosout(self, msg: Log):
        # agent_node, yolo_detector, moveit_module_node 의 WARN / ERROR 만 표시
        name = msg.name or ''
        relevant = any(n in name for n in (
            'agent_node', 'yolo_detector', 'moveit_module_node'
        ))
        if not relevant or msg.level < 30:
            return

        prefix = (yellow('[WARN]')  if msg.level == 30
                  else red('[ERROR]') if msg.level >= 40
                  else dim('[LOG]'))
        node_s = name.split('/')[-1]
        self._print([f"  {prefix} [{node_s}] {msg.msg}"])

    # ─────────────────────────────────────────────────────────────
    def destroy_node(self):
        if self._log_fp:
            self._log_fp.write(f"[{self._ts()}] === Session End ===\n")
            self._log_fp.close()
        super().destroy_node()


def main():
    ap = argparse.ArgumentParser(description='Agent / MoveIt 모듈 실시간 상태 모니터')
    ap.add_argument('--save-log',  action='store_true', help='파일에 로그 저장')
    ap.add_argument('--log-dir', default=None,
                    help='로그 저장 디렉터리 (기본: Arm_Module_Assembler/logs/)')
    args = ap.parse_args()

    log_fp = None
    if args.save_log:
        log_dir = args.log_dir or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), '..', 'logs')
        os.makedirs(log_dir, exist_ok=True)
        fname = os.path.join(log_dir, f"agent_{datetime.now():%Y%m%d_%H%M%S}.log")
        log_fp = open(fname, 'a', buffering=1)
        print(f"로그 파일: {fname}")

    rclpy.init()
    node = AgentDebugNode(log_fp=log_fp)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(dim('\n[종료] Ctrl+C'))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
