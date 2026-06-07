"""Parse MoveIt commands, dispatch to handlers, build a status dict.

Kept free of ROS imports so it can be unit-tested with mock handlers. The node
injects the real handler callables; tests inject fakes.

Two entry points:
  * ``handle(raw)``         — a raw JSON command string (debug / topic path).
  * ``handle_request(cmd, params_json)`` — the ``MoveItExecute`` service path:
    parses ``params_json``, strips robot-fixed keys (D16), merges ``cmd``.
Both converge on ``handle_command(dict)``.

Status dict (LLM_Agent contract):
    {"cmd_echo": str, "success": bool, "phase": str,
     "error_code": int, "error_message": str, "timestamp": float}
The LLM_Agent moveit_client only requires ``success`` and ``error_message``;
the rest is informational but always populated.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Callable, Dict, Optional


# MoveItErrorCodes.SUCCESS == 1 in moveit_msgs, but the LLM_Agent treats
# error_code 0 as success (DESIGN 4.5). We follow the agent's contract: 0 == OK.
ERR_OK = 0
ERR_GENERIC = -1
ERR_BAD_JSON = -2
ERR_UNKNOWN_CMD = -3
ERR_MISSING_FIELD = -4

# Robot-fixed identity values the server owns (DECISIONS D16). Stripped from
# incoming service requests so the module's config defaults win over any
# (possibly stale) values the agent includes; only dynamic values (object pose /
# dimensions, waypoints, target pose, distances) are taken from the request.
ROBOT_FIXED_KEYS = frozenset({
    "hand_frame", "grasp_frame_transform",
    "arm_group_name", "hand_group_name", "eef_name",
    "hand_open_pose", "hand_close_pose", "arm_home_pose",
    "world_frame", "surface_link",
})

PHASE_MAP = {
    "scan": "P1",
    "pick": "P2",
    "lift": "P3",
    "place": "P4",
    "release": "P4",
    "home": "P4",
}


@dataclass
class HandlerResult:
    """Returned by every command handler."""

    success: bool
    error_code: int = ERR_OK
    error_message: str = ""

    @classmethod
    def ok(cls) -> "HandlerResult":
        return cls(True, ERR_OK, "")

    @classmethod
    def fail(cls, message: str, code: int = ERR_GENERIC) -> "HandlerResult":
        return cls(False, code, message)


# A handler takes the parsed command dict and returns a HandlerResult.
Handler = Callable[[dict], HandlerResult]


class CommandRouter:
    def __init__(self, handlers: Dict[str, Handler],
                 logger: Optional[Callable[[str], None]] = None):
        self._handlers = handlers
        self._log = logger or (lambda _msg: None)

    def handle(self, raw: str) -> dict:
        """Process one raw JSON command string (debug / topic path)."""
        try:
            command = json.loads(raw)
        except (json.JSONDecodeError, TypeError) as exc:
            return self._status("unknown", HandlerResult.fail(
                f"invalid JSON: {exc}", ERR_BAD_JSON))

        if not isinstance(command, dict):
            return self._status("unknown", HandlerResult.fail(
                "command must be a JSON object", ERR_BAD_JSON))

        return self.handle_command(command)

    def handle_request(self, cmd: str, params_json: str) -> dict:
        """Service path: MoveItExecute Request(cmd, params_json) -> status dict.

        Parses ``params_json``, strips robot-fixed keys (server is authoritative,
        D16), merges ``cmd``, then dispatches.
        """
        try:
            params = json.loads(params_json) if params_json else {}
        except (json.JSONDecodeError, TypeError) as exc:
            return self._status(cmd or "unknown", HandlerResult.fail(
                f"invalid params_json: {exc}", ERR_BAD_JSON))

        if not isinstance(params, dict):
            return self._status(cmd or "unknown", HandlerResult.fail(
                "params_json must be a JSON object", ERR_BAD_JSON))

        command = {k: v for k, v in params.items() if k not in ROBOT_FIXED_KEYS}
        command["cmd"] = cmd
        return self.handle_command(command)

    def handle_command(self, command: dict) -> dict:
        """Dispatch a parsed command dict to its handler, return a status dict."""
        cmd = command.get("cmd")
        if not cmd:
            return self._status("unknown", HandlerResult.fail(
                "missing 'cmd' field", ERR_MISSING_FIELD))

        handler = self._handlers.get(cmd)
        if handler is None:
            return self._status(cmd, HandlerResult.fail(
                f"unknown command '{cmd}'", ERR_UNKNOWN_CMD))

        self._log(f"executing cmd='{cmd}'")
        try:
            result = handler(command)
        except Exception as exc:  # noqa: BLE001 - isolate handler faults
            self._log(f"handler '{cmd}' raised: {exc}")
            result = HandlerResult.fail(f"{type(exc).__name__}: {exc}", ERR_GENERIC)

        if not isinstance(result, HandlerResult):
            result = HandlerResult.fail(
                f"handler '{cmd}' returned non-HandlerResult", ERR_GENERIC)

        self._log(f"cmd='{cmd}' success={result.success} msg='{result.error_message}'")
        return self._status(cmd, result)

    @staticmethod
    def _status(cmd: str, result: HandlerResult) -> dict:
        return {
            "cmd_echo": cmd,
            "success": result.success,
            "phase": PHASE_MAP.get(cmd, ""),
            "error_code": result.error_code,
            "error_message": result.error_message,
            "timestamp": time.time(),
        }
