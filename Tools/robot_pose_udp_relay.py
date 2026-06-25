#!/usr/bin/env python3
"""Relay normalized robot probe poses from a PC to Quest over UDP."""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Tuple


@dataclass
class RobotClockMapper:
    """Maps robot controller timestamps into the PC Unix clock domain."""

    max_samples: int = 50

    def __post_init__(self) -> None:
        self._pairs: List[Tuple[float, float]] = []

    def update(self, robot_time_s: float, pc_time_s: float) -> None:
        self._pairs.append((robot_time_s, pc_time_s))
        if len(self._pairs) > self.max_samples:
            self._pairs.pop(0)

    def map_to_pc(self, robot_time_s: float) -> float:
        if not self._pairs:
            return time.time()
        if len(self._pairs) == 1:
            robot0, pc0 = self._pairs[0]
            return pc0 + (robot_time_s - robot0)

        robot_mean = sum(pair[0] for pair in self._pairs) / len(self._pairs)
        pc_mean = sum(pair[1] for pair in self._pairs) / len(self._pairs)
        numerator = sum((robot - robot_mean) * (pc - pc_mean) for robot, pc in self._pairs)
        denominator = sum((robot - robot_mean) * (robot - robot_mean) for robot, _ in self._pairs)
        if abs(denominator) < 1e-12:
            return pc_mean + (robot_time_s - robot_mean)

        scale = numerator / denominator
        offset = pc_mean - scale * robot_mean
        return scale * robot_time_s + offset


class TimeSyncServer(threading.Thread):
    """Replies to Quest NTP-like time synchronization requests."""

    def __init__(self, host: str, port: int) -> None:
        super().__init__(daemon=True)
        self._host = host
        self._port = port
        self._stop_event = threading.Event()
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((host, port))
        self._socket.settimeout(0.2)

    def run(self) -> None:
        while not self._stop_event.is_set():
            try:
                data, address = self._socket.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                return

            pc_receive_unix_s = time.time()
            try:
                packet = json.loads(data.decode("utf-8"))
            except json.JSONDecodeError:
                continue

            if packet.get("type") != "time_sync_req":
                continue

            response = {
                "type": "time_sync_resp",
                "seq": int(packet.get("seq", 0)),
                "quest_send_unix_s": float(packet.get("quest_send_unix_s", 0.0)),
                "pc_receive_unix_s": pc_receive_unix_s,
                "pc_send_unix_s": time.time(),
            }
            self._socket.sendto(json.dumps(response, separators=(",", ":")).encode("utf-8"), address)

    def stop(self) -> None:
        self._stop_event.set()
        self._socket.close()


def quaternion_yaw(degrees: float) -> List[float]:
    half = math.radians(degrees) * 0.5
    return [0.0, math.sin(half), 0.0, math.cos(half)]


def iter_mock_poses(rate_hz: float) -> Iterable[Dict[str, Any]]:
    seq = 0
    start_mono = time.monotonic()
    period = 1.0 / max(rate_hz, 0.001)
    while True:
        now_mono = time.monotonic()
        t = now_mono - start_mono
        yield {
            "seq": seq,
            "robot_time_s": t,
            "position": [0.1 * math.sin(t), 0.0, 0.4 + 0.05 * math.cos(t)],
            "rotation_xyzw": quaternion_yaw(30.0 * math.sin(t)),
            "pose_valid": True,
        }
        seq += 1
        time.sleep(period)


def iter_stdin_poses() -> Iterable[Dict[str, Any]]:
    for line in sys.stdin:
        stripped = line.strip()
        if not stripped:
            continue
        yield json.loads(stripped)


def iter_udp_json_poses(host: str, port: int) -> Iterable[Dict[str, Any]]:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    while True:
        data, _ = sock.recvfrom(65535)
        yield json.loads(data.decode("utf-8"))


def build_robot_pose_packet(
    source: Dict[str, Any],
    seq: int,
    pc_receive_unix_s: float,
    mapper: RobotClockMapper,
    robot_time_mode: str,
) -> Dict[str, Any]:
    robot_time_s = float(source.get("robot_time_s", pc_receive_unix_s))
    mapper.update(robot_time_s, pc_receive_unix_s)

    if "sample_pc_unix_s" in source:
        sample_pc_unix_s = float(source["sample_pc_unix_s"])
    elif robot_time_mode == "unix":
        sample_pc_unix_s = robot_time_s
    else:
        sample_pc_unix_s = mapper.map_to_pc(robot_time_s)

    return {
        "type": "robot_pose",
        "seq": int(source.get("seq", seq)),
        "robot_time_s": robot_time_s,
        "sample_pc_unix_s": sample_pc_unix_s,
        "pc_receive_unix_s": pc_receive_unix_s,
        "pc_send_unix_s": time.time(),
        "position": list(source["position"]),
        "rotation_xyzw": list(source["rotation_xyzw"]),
        "pose_valid": bool(source.get("pose_valid", True)),
    }


def validate_pose_packet(packet: Dict[str, Any]) -> Optional[str]:
    position = packet.get("position")
    rotation = packet.get("rotation_xyzw")
    if not isinstance(position, list) or len(position) != 3:
        return "position must be a 3-element list"
    if not isinstance(rotation, list) or len(rotation) != 4:
        return "rotation_xyzw must be a 4-element list"
    return None


def run(args: argparse.Namespace) -> None:
    sync_server = TimeSyncServer(args.sync_bind_host, args.sync_port)
    sync_server.start()
    mapper = RobotClockMapper()
    out_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    quest_address = (args.quest_host, args.quest_pose_port)

    if args.mode == "mock":
        pose_iter = iter_mock_poses(args.rate_hz)
    elif args.mode == "stdin":
        pose_iter = iter_stdin_poses()
    else:
        pose_iter = iter_udp_json_poses(args.robot_bind_host, args.robot_listen_port)

    print(f"Sending robot poses to {quest_address[0]}:{quest_address[1]}", flush=True)
    print(f"Listening for Quest time sync on {args.sync_bind_host}:{args.sync_port}", flush=True)

    seq = 0
    try:
        for source in pose_iter:
            pc_receive_unix_s = time.time()
            packet = build_robot_pose_packet(source, seq, pc_receive_unix_s, mapper, args.robot_time_mode)
            error = validate_pose_packet(packet)
            if error is not None:
                print(f"Skipping invalid robot pose: {error}", file=sys.stderr, flush=True)
                continue

            packet["pc_send_unix_s"] = time.time()
            payload = json.dumps(packet, separators=(",", ":")).encode("utf-8")
            out_socket.sendto(payload, quest_address)
            seq += 1
    finally:
        sync_server.stop()
        out_socket.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quest-host", required=True, help="Quest IP address on the local network.")
    parser.add_argument("--quest-pose-port", type=int, default=15000, help="Quest UDP port for robot_pose packets.")
    parser.add_argument("--sync-bind-host", default="0.0.0.0", help="PC local bind address for time_sync_req packets.")
    parser.add_argument("--sync-port", type=int, default=15001, help="PC UDP port for Quest time synchronization requests.")
    parser.add_argument("--mode", choices=("mock", "stdin", "udp-json"), default="mock", help="Robot pose input mode.")
    parser.add_argument("--rate-hz", type=float, default=50.0, help="Mock pose publish rate.")
    parser.add_argument("--robot-bind-host", default="0.0.0.0", help="PC local bind address for udp-json input mode.")
    parser.add_argument("--robot-listen-port", type=int, default=16000, help="PC UDP port for normalized robot JSON input.")
    parser.add_argument(
        "--robot-time-mode",
        choices=("monotonic", "unix"),
        default="monotonic",
        help="Interpret robot_time_s as controller monotonic seconds or Unix seconds.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
