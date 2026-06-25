using UnityEngine;

namespace PassthroughCameraSamples.ProbeExperiment
{
    /// <summary>
    /// Pure helpers for adjusting the marker-to-robot-base calibration pose.
    /// </summary>
    public static class MarkerBaseCalibrationMath
    {
        /// <summary>
        /// Applies left-stick motion to the marker-frame X/Y translation of T_M_B.
        /// </summary>
        public static Pose ApplyMarkerPlaneStickDelta(
            Pose markerFromBase,
            Vector2 stick,
            float metersPerSecond,
            float deltaTimeSeconds)
        {
            Vector3 position = markerFromBase.position;
            float scale = metersPerSecond * deltaTimeSeconds;
            position.x += stick.x * scale;
            position.y += stick.y * scale;
            return new Pose(position, markerFromBase.rotation);
        }
    }
}
