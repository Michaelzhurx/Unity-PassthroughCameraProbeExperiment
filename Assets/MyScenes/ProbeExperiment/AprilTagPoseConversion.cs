using UnityEngine;

namespace PassthroughCameraSamples.ProbeExperiment
{
    /// <summary>
    /// Converts AprilTag/OpenCV optical-frame poses into Unity camera-frame poses.
    /// </summary>
    public static class AprilTagPoseConversion
    {
        /// <summary>
        /// Converts T_C_M from AprilTag camera/tag coordinates into the Unity camera/marker convention.
        /// </summary>
        public static Pose ConvertCameraFromMarkerCvToUnity(Pose cameraFromMarkerCv)
        {
            Vector3 cvPosition = cameraFromMarkerCv.position;
            Vector3 unityPosition = new Vector3(cvPosition.x, -cvPosition.y, cvPosition.z);

            Matrix4x4 cvRotation = Matrix4x4.Rotate(cameraFromMarkerCv.rotation);
            Matrix4x4 reflectY = Matrix4x4.identity;
            reflectY.m11 = -1f;
            Matrix4x4 unityRotation = reflectY * cvRotation * reflectY;

            return new Pose(unityPosition, unityRotation.rotation);
        }
    }
}
