using UnityEngine;

namespace PassthroughCameraSamples.ProbeExperiment
{
    /// <summary>
    /// Abstract source of robot-provided probe pose samples.
    /// Implementations should return T_B_P in the Unity-compatible calibrated robot base frame and the robot-side timestamp for that sample.
    /// </summary>
    public abstract class RobotProbePoseProvider : MonoBehaviour
    {
        /// <summary>
        /// Tries to read the current probe pose in the robot base frame.
        /// Robot SDK poses from right-handed frames must be converted by the concrete provider before returning here.
        /// </summary>
        public abstract bool TryGetBaseFromProbe(out Pose baseFromProbe, out double robotTimestampSeconds);
    }
}
