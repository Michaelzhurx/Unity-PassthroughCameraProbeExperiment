using Meta.XR;
using Unity.Collections;
using UnityEngine;

namespace PassthroughCameraSamples.ProbeExperiment
{
    /// <summary>
    /// Base class for AprilTag detectors that consume passthrough camera frames.
    /// </summary>
    public abstract class AprilTagDetectorBehaviour : MonoBehaviour
    {
        /// <summary>
        /// True when the detector backend can be called on the current platform.
        /// </summary>
        public abstract bool IsAvailable { get; }

        /// <summary>
        /// Human-readable detector status for debug UI.
        /// </summary>
        public abstract string Status { get; }

        /// <summary>
        /// Attempts to detect the configured AprilTag and estimate T_C_M from pixels and intrinsics.
        /// </summary>
        public abstract bool TryDetect(
            NativeArray<Color32> pixels,
            Vector2Int resolution,
            PassthroughCameraAccess.CameraIntrinsics intrinsics,
            float tagSizeMeters,
            int targetTagId,
            out AprilTagDetection detection);
    }
}
