import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))

from visualize_probe_capture import (
    collect_camera_samples,
    load_frames,
    project_point,
    read_raw_rgb,
)


class VisualizeProbeCaptureTests(unittest.TestCase):
    def test_load_frames_reads_jsonl_records(self):
        with tempfile.TemporaryDirectory() as tmp:
            session = Path(tmp)
            frames_path = session / "frames.jsonl"
            frames_path.write_text('{"frame_id": 1, "robot_pose_valid": true}\n', encoding="utf-8")

            frames = load_frames(session)

        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0]["frame_id"], 1)

    def test_collect_camera_samples_finds_valid_camera_gt(self):
        frame = {
            "frame_id": 7,
            "robot_pose_valid": True,
            "robot_timing": {"normalized_data": [0.1, 0.2, 0.3]},
            "left": {
                "image_path": "left/7.raw",
                "intrinsics": {"width": 1280, "height": 1280, "fx": 10.0, "fy": -10.0, "cx": 5.0, "cy": 6.0},
                "T_C_P_gt": {"position": [0.1, 0.2, 1.0], "rotation_xyzw": [0.0, 0.0, 0.0, 1.0]},
            },
            "right": None,
        }

        samples = collect_camera_samples([frame], cameras=["left", "right"])

        self.assertEqual(len(samples), 1)
        self.assertEqual(samples[0].camera, "left")
        self.assertEqual(samples[0].frame_id, 7)
        self.assertEqual(samples[0].normalized_data, [0.1, 0.2, 0.3])

    def test_project_point_uses_negative_fy_intrinsics(self):
        point = project_point(
            [0.1, 0.2, 1.0],
            {"width": 1280, "height": 1280, "fx": 10.0, "fy": -10.0, "cx": 5.0, "cy": 6.0},
        )

        self.assertEqual(point, (6.0, 4.0))

    def test_project_point_rejects_points_behind_camera(self):
        self.assertIsNone(project_point([0.0, 0.0, -1.0], {"fx": 1.0, "fy": 1.0, "cx": 0.0, "cy": 0.0}))

    def test_read_raw_rgb_reshapes_image(self):
        with tempfile.TemporaryDirectory() as tmp:
            raw_path = Path(tmp) / "0.raw"
            raw_path.write_bytes(bytes([255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255]))

            image = read_raw_rgb(raw_path, width=2, height=2)

        self.assertEqual(image.shape, (2, 2, 3))
        np.testing.assert_array_equal(image[0, 0], np.array([255, 0, 0], dtype=np.uint8))


if __name__ == "__main__":
    unittest.main()
