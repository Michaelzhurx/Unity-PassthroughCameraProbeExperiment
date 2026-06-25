using UnityEngine;

namespace PassthroughCameraSamples.ProbeExperiment
{
    /// <summary>
    /// Describes how a robot pose sample was matched to a camera timestamp.
    /// </summary>
    public struct RobotPoseTimingInfo
    {
        public bool valid;
        public string mode;
        public string invalidReason;
        public double requestedTimestampUnixSeconds;
        public double robotTimestampSeconds;
        public double pcTimestampUnixSeconds;
        public double beforeTimestampUnixSeconds;
        public double afterTimestampUnixSeconds;
        public double timeDeltaSeconds;
        public double interpolationSpanSeconds;
        public double pcQuestOffsetSeconds;
        public double pcQuestRttSeconds;
        public int sequence;
    }

    /// <summary>
    /// Abstract source of robot-provided probe pose samples.
    /// Implementations should return T_B_P in the Unity-compatible calibrated robot base frame and timing data for that sample.
    /// </summary>
    public abstract class RobotProbePoseProvider : MonoBehaviour
    {
        /// <summary>
        /// Tries to read the probe pose in the robot base frame at a target camera timestamp.
        /// Robot SDK poses from right-handed frames must be converted by the concrete provider before returning here.
        /// </summary>
        public abstract bool TryGetBaseFromProbeAt(
            double timestampUnixSeconds,
            out Pose baseFromProbe,
            out RobotPoseTimingInfo timingInfo);
    }
}
