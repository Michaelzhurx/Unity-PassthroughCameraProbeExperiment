using UnityEngine;

namespace PassthroughCameraSamples.ProbeExperiment
{
    /// <summary>
    /// Immutable result returned by the AprilTag detector for one accepted tag.
    /// </summary>
    public readonly struct AprilTagDetection
    {
        /// <summary>
        /// Creates a detection result in the passthrough camera coordinate frame.
        /// </summary>
        public AprilTagDetection(int id, Pose cameraFromMarker, float reprojectionErrorPixels, Vector2[] corners)
        {
            Id = id;
            CameraFromMarker = cameraFromMarker;
            ReprojectionErrorPixels = reprojectionErrorPixels;
            Corners = corners;
        }

        /// <summary>
        /// AprilTag id decoded from the tag family.
        /// </summary>
        public int Id { get; }

        /// <summary>
        /// Marker pose relative to the detecting camera, T_C_M.
        /// </summary>
        public Pose CameraFromMarker { get; }

        /// <summary>
        /// Reprojection error reported by the native detector in image pixels.
        /// </summary>
        public float ReprojectionErrorPixels { get; }

        /// <summary>
        /// Detected image-space tag corners in pixels.
        /// </summary>
        public Vector2[] Corners { get; }
    }
}
