#!/usr/bin/env python3
"""Visualize probe capture ground-truth poses and camera overlays."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageDraw, ImageFont


CAMERAS = ("left", "right")


@dataclass
class CameraSample:
    frame_id: int
    camera: str
    image_path: str
    intrinsics: Dict[str, float]
    camera_from_probe: Dict[str, Any]
    normalized_data: Optional[List[float]]
    rtt_s: Optional[float]
    time_delta_s: Optional[float]


def load_frames(session_dir: Path) -> List[Dict[str, Any]]:
    frames_path = session_dir / "frames.jsonl"
    frames: List[Dict[str, Any]] = []
    with frames_path.open("r", encoding="utf-8-sig") as handle:
        for line_number, line in enumerate(handle, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            try:
                frames.append(json.loads(stripped))
            except json.JSONDecodeError as exc:
                raise ValueError(f"Invalid JSON on {frames_path}:{line_number}: {exc}") from exc
    return frames


def load_optional_json(path: Path) -> Optional[Dict[str, Any]]:
    if not path.exists():
        return None
    with path.open("r", encoding="utf-8-sig") as handle:
        return json.load(handle)


def as_float_list(value: Any, length: int) -> Optional[List[float]]:
    if not isinstance(value, list) or len(value) < length:
        return None
    try:
        return [float(value[index]) for index in range(length)]
    except (TypeError, ValueError):
        return None


def collect_camera_samples(frames: Iterable[Dict[str, Any]], cameras: Sequence[str]) -> List[CameraSample]:
    requested = set(cameras)
    samples: List[CameraSample] = []
    for frame in frames:
        frame_id = int(frame.get("frame_id", -1))
        root_timing = frame.get("robot_timing") if isinstance(frame.get("robot_timing"), dict) else {}
        for camera in CAMERAS:
            if camera not in requested:
                continue
            camera_record = frame.get(camera)
            if not isinstance(camera_record, dict):
                continue
            camera_from_probe = camera_record.get("T_C_P_gt")
            intrinsics = camera_record.get("intrinsics")
            image_path = camera_record.get("image_path")
            if not isinstance(camera_from_probe, dict) or not isinstance(intrinsics, dict) or not image_path:
                continue
            timing = camera_record.get("robot_timing")
            if not isinstance(timing, dict):
                timing = root_timing
            normalized = as_float_list(timing.get("normalized_data"), 3) if isinstance(timing, dict) else None
            samples.append(
                CameraSample(
                    frame_id=frame_id,
                    camera=camera,
                    image_path=str(image_path),
                    intrinsics={key: float(value) for key, value in intrinsics.items() if isinstance(value, (int, float))},
                    camera_from_probe=camera_from_probe,
                    normalized_data=normalized,
                    rtt_s=optional_float(timing.get("pc_quest_rtt_s")) if isinstance(timing, dict) else None,
                    time_delta_s=optional_float(timing.get("robot_time_delta_s")) if isinstance(timing, dict) else None,
                )
            )
    return samples


def optional_float(value: Any) -> Optional[float]:
    if isinstance(value, (int, float)) and math.isfinite(float(value)):
        return float(value)
    return None


def project_point(point_xyz: Sequence[float], intrinsics: Dict[str, float]) -> Optional[Tuple[float, float]]:
    if len(point_xyz) < 3:
        return None
    x = float(point_xyz[0])
    y = float(point_xyz[1])
    z = float(point_xyz[2])
    if z <= 0.0:
        return None
    try:
        fx = float(intrinsics["fx"])
        fy = float(intrinsics["fy"])
        cx = float(intrinsics["cx"])
        cy = float(intrinsics["cy"])
    except KeyError:
        return None
    return (fx * x / z + cx, fy * y / z + cy)


def read_raw_rgb(path: Path, width: int, height: int) -> np.ndarray:
    expected_size = width * height * 3
    data = np.fromfile(path, dtype=np.uint8)
    if data.size != expected_size:
        raise ValueError(f"{path} has {data.size} bytes, expected {expected_size} for {width}x{height} RGB.")
    return data.reshape((height, width, 3))


def quaternion_xyzw_to_matrix(quaternion: Sequence[float]) -> np.ndarray:
    x, y, z, w = [float(value) for value in quaternion[:4]]
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return np.eye(3, dtype=np.float64)
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def pose_positions(frames: Sequence[Dict[str, Any]], pose_key: str) -> Tuple[np.ndarray, np.ndarray]:
    frame_ids: List[int] = []
    positions: List[List[float]] = []
    for frame in frames:
        pose = frame.get(pose_key)
        if not isinstance(pose, dict):
            continue
        position = as_float_list(pose.get("position"), 3)
        if position is None:
            continue
        frame_ids.append(int(frame.get("frame_id", len(frame_ids))))
        positions.append(position)
    if not positions:
        return np.array([], dtype=np.int64), np.empty((0, 3), dtype=np.float64)
    return np.array(frame_ids, dtype=np.int64), np.array(positions, dtype=np.float64)


def timing_arrays(frames: Sequence[Dict[str, Any]]) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    frame_ids: List[int] = []
    normalized: List[List[float]] = []
    rtt: List[float] = []
    delta: List[float] = []
    for frame in frames:
        timing = frame.get("robot_timing")
        if not isinstance(timing, dict):
            continue
        frame_id = int(frame.get("frame_id", len(frame_ids)))
        norm = as_float_list(timing.get("normalized_data"), 3)
        rtt_s = optional_float(timing.get("pc_quest_rtt_s"))
        delta_s = optional_float(timing.get("robot_time_delta_s"))
        if norm is not None:
            frame_ids.append(frame_id)
            normalized.append(norm)
            rtt.append(rtt_s if rtt_s is not None else np.nan)
            delta.append(delta_s if delta_s is not None else np.nan)
    if not frame_ids:
        return np.array([]), np.empty((0, 3)), np.array([]), np.array([])
    return np.array(frame_ids), np.array(normalized), np.array(rtt), np.array(delta)


def save_trajectory_plot(frames: Sequence[Dict[str, Any]], output_path: Path) -> None:
    frame_ids, positions = pose_positions(frames, "T_B_P")
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    if positions.size:
        ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], linewidth=1.5)
        ax.scatter(positions[0, 0], positions[0, 1], positions[0, 2], c="green", label="start")
        ax.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], c="red", label="end")
    ax.set_title(f"T_B_P trajectory ({len(frame_ids)} valid samples)")
    ax.set_xlabel("base X (m)")
    ax.set_ylabel("base Y (m)")
    ax.set_zlabel("base Z (m)")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def save_position_timeseries(frames: Sequence[Dict[str, Any]], output_path: Path) -> None:
    frame_ids, positions = pose_positions(frames, "T_B_P")
    fig, ax = plt.subplots(figsize=(10, 5))
    if positions.size:
        labels = ("x", "y", "z")
        for axis, label in enumerate(labels):
            ax.plot(frame_ids, positions[:, axis], label=label)
    ax.set_title("T_B_P position over frames")
    ax.set_xlabel("frame_id")
    ax.set_ylabel("position (m)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def save_normalized_data_plot(frames: Sequence[Dict[str, Any]], output_path: Path) -> None:
    frame_ids, normalized, _, _ = timing_arrays(frames)
    fig, ax = plt.subplots(figsize=(10, 5))
    if normalized.size:
        for index in range(normalized.shape[1]):
            ax.plot(frame_ids, normalized[:, index], label=f"norm{index}")
    ax.set_title("Normalized coil data")
    ax.set_xlabel("frame_id")
    ax.set_ylabel("normalized value")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def save_timing_quality_plot(frames: Sequence[Dict[str, Any]], output_path: Path) -> None:
    frame_ids, _, rtt, delta = timing_arrays(frames)
    valid = np.array([1 if frame.get("robot_pose_valid") else 0 for frame in frames], dtype=np.float64)
    all_frame_ids = np.array([int(frame.get("frame_id", index)) for index, frame in enumerate(frames)])

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=False)
    if frame_ids.size:
        axes[0].plot(frame_ids, rtt * 1000.0, label="pc_quest_rtt_ms")
        axes[1].plot(frame_ids, delta * 1000.0, label="robot_time_delta_ms", color="tab:orange")
    if all_frame_ids.size:
        axes[2].plot(all_frame_ids, valid, drawstyle="steps-post", color="tab:green", label="robot_pose_valid")
    axes[0].set_ylabel("RTT (ms)")
    axes[1].set_ylabel("delta (ms)")
    axes[2].set_ylabel("valid")
    axes[2].set_xlabel("frame_id")
    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")
    fig.suptitle("Robot timing quality")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def draw_overlay(image: np.ndarray, sample: CameraSample) -> Optional[Image.Image]:
    width = int(sample.intrinsics.get("width", image.shape[1]))
    height = int(sample.intrinsics.get("height", image.shape[0]))
    pose = sample.camera_from_probe
    position = as_float_list(pose.get("position"), 3)
    if position is None:
        return None
    origin = project_point(position, sample.intrinsics)
    if origin is None:
        return None
    margin = 64.0
    if origin[0] < -margin or origin[0] > width + margin or origin[1] < -margin or origin[1] > height + margin:
        return None

    output = Image.fromarray(image, mode="RGB")
    draw = ImageDraw.Draw(output)
    font = ImageFont.load_default()
    u, v = origin
    radius = 8
    draw.ellipse((u - radius, v - radius, u + radius, v + radius), outline="yellow", width=3)
    draw.line((u - 16, v, u + 16, v), fill="yellow", width=2)
    draw.line((u, v - 16, u, v + 16), fill="yellow", width=2)

    quaternion = as_float_list(pose.get("rotation_xyzw"), 4)
    if quaternion is not None:
        rotation = quaternion_xyzw_to_matrix(quaternion)
        origin_3d = np.array(position, dtype=np.float64)
        colors = ("red", "lime", "cyan")
        axis_length_m = 0.04
        for axis_index, color in enumerate(colors):
            endpoint_3d = origin_3d + rotation[:, axis_index] * axis_length_m
            endpoint = project_point(endpoint_3d.tolist(), sample.intrinsics)
            if endpoint is not None:
                draw.line((u, v, endpoint[0], endpoint[1]), fill=color, width=4)

    norm_text = ""
    if sample.normalized_data is not None:
        norm_text = " norm=" + ",".join(f"{value:.3f}" for value in sample.normalized_data)
    label = f"frame={sample.frame_id} {sample.camera} z={position[2]:.3f}m{norm_text}"
    text_box = draw.textbbox((0, 0), label, font=font)
    draw.rectangle((8, 8, 16 + text_box[2], 18 + text_box[3]), fill=(0, 0, 0))
    draw.text((12, 12), label, fill="white", font=font)
    return output


def save_overlay_images(
    session_dir: Path,
    samples: Sequence[CameraSample],
    output_dir: Path,
    max_images: int,
    stride: int,
) -> Dict[str, int]:
    counts = {camera: 0 for camera in CAMERAS}
    skipped = {camera: 0 for camera in CAMERAS}
    selected_by_camera = {camera: 0 for camera in CAMERAS}
    for sample in samples:
        if sample.frame_id % max(stride, 1) != 0:
            continue
        if counts[sample.camera] >= max_images:
            continue

        selected_by_camera[sample.camera] += 1
        raw_path = session_dir / sample.image_path
        if not raw_path.exists():
            print(f"[warning] Missing raw image: {raw_path}")
            skipped[sample.camera] += 1
            continue
        width = int(sample.intrinsics.get("width", 1280))
        height = int(sample.intrinsics.get("height", 1280))
        try:
            image = read_raw_rgb(raw_path, width=width, height=height)
            overlay = draw_overlay(image, sample)
        except Exception as exc:
            print(f"[warning] Could not render {raw_path}: {exc}")
            skipped[sample.camera] += 1
            continue
        if overlay is None:
            skipped[sample.camera] += 1
            continue
        camera_output_dir = output_dir / "overlays" / sample.camera
        camera_output_dir.mkdir(parents=True, exist_ok=True)
        overlay_path = camera_output_dir / f"{sample.frame_id:06d}.jpg"
        overlay.save(overlay_path, "JPEG", quality=92)
        counts[sample.camera] += 1
    for camera in CAMERAS:
        print(
            f"{camera}: selected {selected_by_camera[camera]}, "
            f"wrote {counts[camera]} overlays, skipped {skipped[camera]}"
        )
    return counts


def write_summary(
    session_dir: Path,
    output_dir: Path,
    frames: Sequence[Dict[str, Any]],
    samples: Sequence[CameraSample],
    overlay_counts: Dict[str, int],
    anchor_init: Optional[Dict[str, Any]],
) -> None:
    valid_count = sum(1 for frame in frames if frame.get("robot_pose_valid"))
    summary = {
        "session_dir": str(session_dir),
        "frame_count": len(frames),
        "robot_pose_valid_count": valid_count,
        "camera_sample_count": len(samples),
        "overlay_counts": overlay_counts,
        "anchor_loaded": anchor_init is not None,
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")


def normalize_camera_arg(camera: str) -> List[str]:
    if camera == "both":
        return list(CAMERAS)
    return [camera]


def run(args: argparse.Namespace) -> None:
    session_dir = Path(args.session).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve() if args.output_dir else session_dir / "visualization"
    output_dir.mkdir(parents=True, exist_ok=True)

    frames = load_frames(session_dir)
    anchor_init = load_optional_json(session_dir / "anchor_init.json")
    cameras = normalize_camera_arg(args.camera)
    samples = collect_camera_samples(frames, cameras)

    save_trajectory_plot(frames, output_dir / "trajectory_3d.png")
    save_position_timeseries(frames, output_dir / "position_timeseries.png")
    save_normalized_data_plot(frames, output_dir / "normalized_data.png")
    save_timing_quality_plot(frames, output_dir / "timing_quality.png")
    overlay_counts = save_overlay_images(
        session_dir=session_dir,
        samples=samples,
        output_dir=output_dir,
        max_images=max(args.max_images, 0),
        stride=max(args.stride, 1),
    )
    write_summary(session_dir, output_dir, frames, samples, overlay_counts, anchor_init)

    valid_count = sum(1 for frame in frames if frame.get("robot_pose_valid"))
    print(f"Loaded {len(frames)} frames; {valid_count} robot poses valid.")
    print(f"Wrote visualization output to {output_dir}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True, help="Capture session directory containing frames.jsonl.")
    parser.add_argument("--output-dir", default=None, help="Visualization output directory. Defaults to <session>/visualization.")
    parser.add_argument("--max-images", type=int, default=30, help="Maximum overlay images to write per camera.")
    parser.add_argument("--camera", choices=("left", "right", "both"), default="both", help="Camera overlays to render.")
    parser.add_argument("--stride", type=int, default=1, help="Only render overlay candidates whose frame_id is divisible by stride.")
    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
