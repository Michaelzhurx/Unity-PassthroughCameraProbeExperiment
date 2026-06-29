import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))

from robot_pose_udp_relay import (
    RobotClockMapper,
    build_robot_pose_packet,
    parse_udp_csv_pose,
    quaternion_from_rvec,
)


class RobotPoseUdpRelayTests(unittest.TestCase):
    def test_robot_clock_mapper_is_monotonic(self):
        mapper = RobotClockMapper()
        mapper.update(10.0, 100.0)
        mapper.update(10.1, 100.1)
        mapper.update(10.2, 100.2)

        self.assertAlmostEqual(mapper.map_to_pc(10.15), 100.15, places=6)
        self.assertLess(mapper.map_to_pc(10.15), mapper.map_to_pc(10.16))

    def test_build_robot_pose_packet_uses_unix_robot_time(self):
        packet = build_robot_pose_packet(
            {
                "robot_time_s": 1234.5,
                "position": [1.0, 2.0, 3.0],
                "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
                "pose_valid": True,
            },
            seq=7,
            pc_receive_unix_s=2000.0,
            mapper=RobotClockMapper(),
            robot_time_mode="unix",
        )

        self.assertEqual(packet["type"], "robot_pose")
        self.assertEqual(packet["seq"], 7)
        self.assertEqual(packet["sample_pc_unix_s"], 1234.5)
        self.assertEqual(packet["pc_receive_unix_s"], 2000.0)

    def test_build_robot_pose_packet_preserves_pose_raw_and_normalized_data(self):
        packet = build_robot_pose_packet(
            {
                "robot_time_s": 1.0,
                "position": [1.0, 2.0, 3.0],
                "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
                "pose_raw": [1.0, 2.0, 3.0, 0.1, 0.2, 0.3],
                "normalized_data": [0.1, 0.2, 0.3],
            },
            seq=1,
            pc_receive_unix_s=2.0,
            mapper=RobotClockMapper(),
            robot_time_mode="unix",
        )

        self.assertEqual(packet["pose_raw"], [1.0, 2.0, 3.0, 0.1, 0.2, 0.3])
        self.assertEqual(packet["normalized_data"], [0.1, 0.2, 0.3])

    def test_parse_udp_csv_pose_extracts_pose_and_normalized_data(self):
        source = parse_udp_csv_pose(
            b"0,0,0,0.2467379377246327,-0.3582989448524151,0.0845082706030868,"
            b"-3.0948875609443247,-0.4125843204678963,0.016862245449159756,"
            b"0.05033317394554615,0.0485895574092865,0.04960751533508301,"
            b"0.5059804481779453,0.5073896793143372,0.506232533445658"
        )

        self.assertEqual(
            source["position"],
            [0.2467379377246327, -0.3582989448524151, -0.0845082706030868],
        )
        self.assertEqual(
            source["pose_raw"],
            [
                0.2467379377246327,
                -0.3582989448524151,
                0.0845082706030868,
                -3.0948875609443247,
                -0.4125843204678963,
                0.016862245449159756,
            ],
        )
        self.assertEqual(
            source["normalized_data"],
            [0.5059804481779453, 0.5073896793143372, 0.506232533445658],
        )
        self.assertIn("sample_pc_unix_s", source)

    def test_parse_udp_csv_pose_converts_robot_base_to_unity_frame_by_flipping_z(self):
        source = parse_udp_csv_pose(
            b"0,0,0,1,2,3,1.5707963267948966,0,0,7,8,9,0.1,0.2,0.3"
        )

        self.assertEqual(source["position"], [1.0, 2.0, -3.0])
        self.assertAlmostEqual(source["rotation_xyzw"][0], -0.7071067811865475, places=6)
        self.assertAlmostEqual(source["rotation_xyzw"][1], 0.0, places=6)
        self.assertAlmostEqual(source["rotation_xyzw"][2], 0.0, places=6)
        self.assertAlmostEqual(source["rotation_xyzw"][3], 0.7071067811865476, places=6)
        self.assertEqual(source["pose_raw"], [1.0, 2.0, 3.0, 1.5707963267948966, 0.0, 0.0])

    def test_parse_udp_csv_pose_can_keep_unity_frame_without_conversion(self):
        source = parse_udp_csv_pose(
            b"0,0,0,1,2,3,0,0,0,7,8,9,0.1,0.2,0.3",
            robot_frame="unity",
        )

        self.assertEqual(source["position"], [1.0, 2.0, 3.0])
        self.assertEqual(source["rotation_xyzw"], [0.0, 0.0, 0.0, 1.0])

    def test_parse_udp_csv_pose_accepts_python_bytes_repr_text(self):
        source = parse_udp_csv_pose(
            "b'0,0,0,1,2,3,0,0,0,7,8,9,0.1,0.2,0.3'"
        )

        self.assertEqual(source["position"], [1.0, 2.0, -3.0])
        self.assertEqual(source["normalized_data"], [0.1, 0.2, 0.3])

    def test_parse_udp_csv_pose_rejects_short_or_non_numeric_payloads(self):
        with self.assertRaises(ValueError):
            parse_udp_csv_pose(b"0,0,0,1,2")

        with self.assertRaises(ValueError):
            parse_udp_csv_pose(b"0,0,0,1,2,not-a-number,0,0,0,0,0,0,0,0,0")

    def test_quaternion_from_rvec_returns_unit_quaternion(self):
        quat = quaternion_from_rvec(-3.0948875609443247, -0.4125843204678963, 0.016862245449159756)

        self.assertAlmostEqual(sum(value * value for value in quat), 1.0, places=6)

    def test_quaternion_from_zero_rvec_returns_identity(self):
        self.assertEqual(quaternion_from_rvec(0.0, 0.0, 0.0), [0.0, 0.0, 0.0, 1.0])


if __name__ == "__main__":
    unittest.main()
