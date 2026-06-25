using System;
using System.Runtime.InteropServices;
using Meta.XR;
using Unity.Collections;
using UnityEngine;

namespace PassthroughCameraSamples.ProbeExperiment
{
    /// <summary>
    /// Calls the Android native AprilTag detector plugin and adapts its result to Unity Pose data.
    /// </summary>
    public class AprilTagNativeDetector : AprilTagDetectorBehaviour
    {
        private const string NativeLibraryName = "apriltag_unity";

        [SerializeField] private bool m_enableInEditor;
        [SerializeField] private bool m_flipInputVertically = true;

        private byte[] m_rgbaBuffer;
        private string m_status = "Not initialized.";
        private bool m_nativeChecked;
        private bool m_nativeUnavailable;

        public override bool IsAvailable
        {
            get
            {
                EnsureNativeChecked();
                return !m_nativeUnavailable;
            }
        }

        public override string Status => m_status;

        private void Awake()
        {
            EnsureNativeChecked();
        }

        /// <summary>
        /// Converts the current Unity Color32 frame to RGBA bytes, then calls the native tag36h11 detector.
        /// </summary>
        public override bool TryDetect(
            NativeArray<Color32> pixels,
            Vector2Int resolution,
            PassthroughCameraAccess.CameraIntrinsics intrinsics,
            float tagSizeMeters,
            int targetTagId,
            out AprilTagDetection detection)
        {
            detection = default;
            EnsureNativeChecked();

#if UNITY_EDITOR
            if (!m_enableInEditor)
            {
                m_status = "AprilTag native detector is disabled in Editor.";
                return false;
            }
#endif

            if (m_nativeUnavailable)
            {
                return false;
            }

            if (!pixels.IsCreated || pixels.Length == 0)
            {
                m_status = "Input image is empty.";
                return false;
            }

            int expectedPixels = resolution.x * resolution.y;
            if (pixels.Length < expectedPixels)
            {
                m_status = $"Input image has {pixels.Length} pixels, expected {expectedPixels}.";
                return false;
            }

            int byteCount = expectedPixels * 4;
            if (m_rgbaBuffer == null || m_rgbaBuffer.Length != byteCount)
            {
                m_rgbaBuffer = new byte[byteCount];
            }

            for (int i = 0; i < expectedPixels; i++)
            {
                Color32 c = pixels[i];
                int baseIndex = i * 4;
                m_rgbaBuffer[baseIndex] = c.r;
                m_rgbaBuffer[baseIndex + 1] = c.g;
                m_rgbaBuffer[baseIndex + 2] = c.b;
                m_rgbaBuffer[baseIndex + 3] = c.a;
            }

            ProbeExperimentImageGeometry.ComputeCurrentResolutionIntrinsics(
                intrinsics.FocalLength.x,
                intrinsics.FocalLength.y,
                intrinsics.PrincipalPoint.x,
                intrinsics.PrincipalPoint.y,
                intrinsics.SensorResolution.x,
                intrinsics.SensorResolution.y,
                resolution.x,
                resolution.y,
                out float imageFx,
                out float imageFy,
                out float imageCx,
                out float imageCy);

            try
            {
                int result = apriltag_unity_detect_tag36h11(
                    m_rgbaBuffer,
                    resolution.x,
                    resolution.y,
                    imageFx,
                    imageFy,
                    imageCx,
                    imageCy,
                    tagSizeMeters,
                    targetTagId,
                    m_flipInputVertically ? 1 : 0,
                    out NativeDetection nativeDetection);

                if (result == 0)
                {
                    m_status = "No matching AprilTag detected.";
                    return false;
                }

                if (result < 0)
                {
                    m_status = $"AprilTag native detector returned error {result}.";
                    return false;
                }

                detection = nativeDetection.ToDetection();
                m_status = $"Detected tag {detection.Id}, reprojection error {detection.ReprojectionErrorPixels:F3}px. FlipY={m_flipInputVertically}.";
                return true;
            }
            catch (DllNotFoundException)
            {
                m_nativeUnavailable = true;
                m_status = $"Missing native library: lib{NativeLibraryName}.so.";
                return false;
            }
            catch (EntryPointNotFoundException)
            {
                m_nativeUnavailable = true;
                m_status = $"Missing native entry point: {nameof(apriltag_unity_detect_tag36h11)}.";
                return false;
            }
        }

        /// <summary>
        /// Calls a lightweight native function to verify that the library and expected ABI are loadable.
        /// </summary>
        public bool CheckNativeAvailability()
        {
            EnsureNativeChecked();
            return !m_nativeUnavailable;
        }

        private void EnsureNativeChecked()
        {
            if (m_nativeChecked)
            {
                return;
            }

            m_nativeChecked = true;

#if UNITY_EDITOR
            if (!m_enableInEditor)
            {
                m_nativeUnavailable = true;
                m_status = "AprilTag native detector is disabled in Editor.";
                return;
            }
#endif

            try
            {
                int result = apriltag_unity_smoke_test();
                if (result > 0)
                {
                    m_nativeUnavailable = false;
                    m_status = "AprilTag native detector loaded.";
                    return;
                }

                m_nativeUnavailable = true;
                m_status = $"AprilTag native smoke test failed: {result}.";
            }
            catch (DllNotFoundException)
            {
                m_nativeUnavailable = true;
                m_status = $"Missing native library: lib{NativeLibraryName}.so.";
            }
            catch (EntryPointNotFoundException)
            {
                m_nativeUnavailable = true;
                m_status = $"Missing native entry point: {nameof(apriltag_unity_smoke_test)}.";
            }
        }

        /// <summary>
        /// Native C ABI smoke test expected from libapriltag_unity.so.
        /// </summary>
        [DllImport(NativeLibraryName, CallingConvention = CallingConvention.Cdecl)]
        private static extern int apriltag_unity_smoke_test();

        /// <summary>
        /// Native C ABI entry point expected from libapriltag_unity.so.
        /// </summary>
        [DllImport(NativeLibraryName, CallingConvention = CallingConvention.Cdecl)]
        private static extern int apriltag_unity_detect_tag36h11(
            byte[] rgba,
            int width,
            int height,
            float fx,
            float fy,
            float cx,
            float cy,
            float tagSizeMeters,
            int targetTagId,
            int flipInputVertically,
            out NativeDetection detection);

        [StructLayout(LayoutKind.Sequential)]
        private struct NativeDetection
        {
            public int id;
            public float px;
            public float py;
            public float pz;
            public float qx;
            public float qy;
            public float qz;
            public float qw;
            public float reprojectionErrorPixels;
            public float corner0x;
            public float corner0y;
            public float corner1x;
            public float corner1y;
            public float corner2x;
            public float corner2y;
            public float corner3x;
            public float corner3y;

            /// <summary>
            /// Converts the packed native struct into the managed AprilTagDetection value.
            /// </summary>
            public AprilTagDetection ToDetection()
            {
                var corners = new[]
                {
                    new Vector2(corner0x, corner0y),
                    new Vector2(corner1x, corner1y),
                    new Vector2(corner2x, corner2y),
                    new Vector2(corner3x, corner3y)
                };

                Pose cameraFromMarkerCv = new Pose(
                    new Vector3(px, py, pz),
                    new Quaternion(qx, qy, qz, qw));
                Pose cameraFromMarkerUnity = AprilTagPoseConversion.ConvertCameraFromMarkerCvToUnity(cameraFromMarkerCv);

                return new AprilTagDetection(id, cameraFromMarkerUnity, reprojectionErrorPixels, corners);
            }
        }
    }
}
