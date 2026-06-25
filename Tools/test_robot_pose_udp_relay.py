import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))

from robot_pose_udp_relay import RobotClockMapper, build_robot_pose_packet


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


if __name__ == "__main__":
    unittest.main()
