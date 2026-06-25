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
        /// Returns the inspector pose as T_B_P and uses Unity realtime as a placeholder timestamp.
        /// </summary>
        public override bool TryGetBaseFromProbe(out Pose baseFromProbe, out double robotTimestampSeconds)
        {
            baseFromProbe = new Pose(
                m_baseFromProbePosition,
                Quaternion.Euler(m_baseFromProbeEulerDegrees));
            robotTimestampSeconds = Time.realtimeSinceStartupAsDouble;
            return m_poseValid;
        }
    }
}
