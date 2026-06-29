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


ROBOT_FRAME_UNITY = "unity"
ROBOT_FRAME_Z_FLIP = "robot-z-flip"


ROBOT_Z_FLIP_TO_UNITY_MATRIX: List[List[float]] = [
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, -1.0],
]


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


def quaternion_from_rvec(rx: float, ry: float, rz: float) -> List[float]:
    angle = math.sqrt(rx * rx + ry * ry + rz * rz)
    if angle < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]

    half = angle * 0.5
    scale = math.sin(half) / angle
    return [rx * scale, ry * scale, rz * scale, math.cos(half)]


def matrix_vector_multiply(matrix: List[List[float]], vector: List[float]) -> List[float]:
    return [
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    ]


def matrix_multiply(a: List[List[float]], b: List[List[float]]) -> List[List[float]]:
    return [
        [
            a[row][0] * b[0][col] + a[row][1] * b[1][col] + a[row][2] * b[2][col]
            for col in range(3)
        ]
        for row in range(3)
    ]


def matrix_transpose(matrix: List[List[float]]) -> List[List[float]]:
    return [
        [matrix[0][0], matrix[1][0], matrix[2][0]],
        [matrix[0][1], matrix[1][1], matrix[2][1]],
        [matrix[0][2], matrix[1][2], matrix[2][2]],
    ]


def quaternion_to_matrix(quaternion_xyzw: List[float]) -> List[List[float]]:
    x, y, z, w = quaternion_xyzw
    length = math.sqrt(x * x + y * y + z * z + w * w)
    if length < 1e-12:
        return [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]

    x /= length
    y /= length
    z /= length
    w /= length
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def quaternion_from_matrix(matrix: List[List[float]]) -> List[float]:
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (matrix[2][1] - matrix[1][2]) / s
        y = (matrix[0][2] - matrix[2][0]) / s
        z = (matrix[1][0] - matrix[0][1]) / s
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        s = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        w = (matrix[2][1] - matrix[1][2]) / s
        x = 0.25 * s
        y = (matrix[0][1] + matrix[1][0]) / s
        z = (matrix[0][2] + matrix[2][0]) / s
    elif matrix[1][1] > matrix[2][2]:
        s = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        w = (matrix[0][2] - matrix[2][0]) / s
        x = (matrix[0][1] + matrix[1][0]) / s
        y = 0.25 * s
        z = (matrix[1][2] + matrix[2][1]) / s
    else:
        s = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        w = (matrix[1][0] - matrix[0][1]) / s
        x = (matrix[0][2] + matrix[2][0]) / s
        y = (matrix[1][2] + matrix[2][1]) / s
        z = 0.25 * s

    length = math.sqrt(x * x + y * y + z * z + w * w)
    if length < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [x / length, y / length, z / length, w / length]


def convert_pose_to_unity_frame(
    position: List[float],
    rotation_xyzw: List[float],
    robot_frame: str,
) -> Tuple[List[float], List[float]]:
    if robot_frame == ROBOT_FRAME_UNITY:
        return list(position), list(rotation_xyzw)
    if robot_frame != ROBOT_FRAME_Z_FLIP:
        raise ValueError(f"Unsupported robot frame: {robot_frame}")

    basis = ROBOT_Z_FLIP_TO_UNITY_MATRIX
    converted_position = matrix_vector_multiply(basis, position)
    converted_rotation = matrix_multiply(
        matrix_multiply(basis, quaternion_to_matrix(rotation_xyzw)),
        matrix_transpose(basis),
    )
    return converted_position, quaternion_from_matrix(converted_rotation)


def parse_udp_csv_pose(
    payload: bytes | str,
    robot_frame: str = ROBOT_FRAME_Z_FLIP,
) -> Dict[str, Any]:
    if isinstance(payload, bytes):
        text = payload.decode("utf-8").strip()
    else:
        text = payload.strip()

    if (text.startswith("b'") and text.endswith("'")) or (text.startswith('b"') and text.endswith('"')):
        text = text[2:-1]

    try:
        values = [float(part.strip()) for part in text.split(",") if part.strip()]
    except ValueError as exc:
        raise ValueError(f"UDP CSV pose contains a non-numeric field: {text}") from exc

    if len(values) < 15:
        raise ValueError(f"UDP CSV pose requires at least 15 numbers, got {len(values)}.")

    pose_raw = values[3:9]
    normalized_data = values[12:15]
    position, rotation_xyzw = convert_pose_to_unity_frame(
        pose_raw[:3],
        quaternion_from_rvec(pose_raw[3], pose_raw[4], pose_raw[5]),
        robot_frame,
    )
    now = time.time()

    return {
        "robot_time_s": now,
        "sample_pc_unix_s": now,
        "position": position,
        "rotation_xyzw": rotation_xyzw,
        "pose_raw": pose_raw,
        "normalized_data": normalized_data,
        "pose_valid": True,
    }


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


def iter_udp_csv_poses(host: str, port: int, robot_frame: str) -> Iterable[Dict[str, Any]]:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    while True:
        data, _ = sock.recvfrom(65535)
        try:
            yield parse_udp_csv_pose(data, robot_frame=robot_frame)
        except ValueError as exc:
            print(f"Skipping invalid UDP CSV pose: {exc}", file=sys.stderr, flush=True)


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

    packet = {
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

    if "pose_raw" in source:
        packet["pose_raw"] = list(source["pose_raw"])
    if "normalized_data" in source:
        packet["normalized_data"] = list(source["normalized_data"])

    return packet


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
    elif args.mode == "udp-json":
        pose_iter = iter_udp_json_poses(args.robot_bind_host, args.robot_listen_port)
    else:
        pose_iter = iter_udp_csv_poses(args.robot_bind_host, args.robot_listen_port, args.robot_frame)

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
    parser.add_argument(
        "--mode",
        choices=("mock", "stdin", "udp-json", "udp-csv-pose"),
        default="mock",
        help="Robot pose input mode.",
    )
    parser.add_argument("--rate-hz", type=float, default=50.0, help="Mock pose publish rate.")
    parser.add_argument("--robot-bind-host", default="127.0.0.1", help="PC local bind address for robot UDP input modes.")
    parser.add_argument("--robot-listen-port", type=int, default=8080, help="PC UDP port for robot input.")
    parser.add_argument(
        "--robot-frame",
        choices=(ROBOT_FRAME_Z_FLIP, ROBOT_FRAME_UNITY),
        default=ROBOT_FRAME_Z_FLIP,
        help=(
            "Coordinate convention for udp-csv-pose input. The default keeps robot X/Y unchanged "
            "and flips Z before sending poses to Unity."
        ),
    )
    parser.add_argument(
        "--robot-time-mode",
        choices=("monotonic", "unix"),
        default="monotonic",
        help="Interpret robot_time_s as controller monotonic seconds or Unix seconds.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
