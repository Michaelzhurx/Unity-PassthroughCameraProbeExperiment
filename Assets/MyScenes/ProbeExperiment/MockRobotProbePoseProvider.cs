using UnityEngine;

namespace PassthroughCameraSamples.ProbeExperiment
{
    /// <summary>
    /// Inspector-configured robot pose provider used to test the Quest-side capture pipeline.
    /// </summary>
    public class MockRobotProbePoseProvider : RobotProbePoseProvider
    {
        [SerializeField] private Vector3 m_baseFromProbePosition;
        [SerializeField] private Vector3 m_baseFromProbeEulerDegrees;
        [SerializeField] private bool m_poseValid = true;

        /// <summary>
        /// Returns the inspector pose as T_B_P for any requested camera timestamp.
        /// </summary>
        public override bool TryGetBaseFromProbeAt(
            double timestampUnixSeconds,
            out Pose baseFromProbe,
            out RobotPoseTimingInfo timingInfo)
        {
            baseFromProbe = new Pose(
                m_baseFromProbePosition,
                Quaternion.Euler(m_baseFromProbeEulerDegrees));

            timingInfo = new RobotPoseTimingInfo
            {
                valid = m_poseValid,
                mode = "mock_current",
                invalidReason = m_poseValid ? string.Empty : "mock_invalid",
                requestedTimestampUnixSeconds = timestampUnixSeconds,
                robotTimestampSeconds = timestampUnixSeconds,
                pcTimestampUnixSeconds = timestampUnixSeconds,
                beforeTimestampUnixSeconds = timestampUnixSeconds,
                afterTimestampUnixSeconds = timestampUnixSeconds,
                timeDeltaSeconds = 0.0,
                interpolationSpanSeconds = 0.0,
                pcQuestOffsetSeconds = 0.0,
                pcQuestRttSeconds = 0.0,
                sequence = 0
            };

            return m_poseValid;
        }
    }
}
