using PassthroughCameraSamples.ProbeExperiment;
using UnityEditor;
using UnityEngine;

namespace PassthroughCameraSamples.ProbeExperiment.Tests
{
    /// <summary>
    /// Small editor-only smoke test for pose composition and pose JSON serialization.
    /// </summary>
    public static class ProbeExperimentPoseMathSelfTest
    {
        /// <summary>
        /// Runs the self test from the Unity menu item.
        /// </summary>
        [MenuItem("Tools/Probe Experiment/Run Pose Math Self Test")]
        public static void Run()
        {
            Pose worldFromCamera = new Pose(new Vector3(1f, 0f, 0f), Quaternion.identity);
            Pose worldFromBase = new Pose(new Vector3(1f, 2f, 0f), Quaternion.identity);
            Pose baseFromProbe = new Pose(new Vector3(0f, 0f, 3f), Quaternion.identity);

            Pose cameraFromProbe = ProbeExperimentPoseMath.ComputeCameraFromProbe(
                worldFromCamera,
                worldFromBase,
                baseFromProbe);

            AssertApproximately(cameraFromProbe.position, new Vector3(0f, 2f, 3f), "cameraFromProbe.position");
            AssertApproximately(cameraFromProbe.rotation, Quaternion.identity, "cameraFromProbe.rotation");
            ProbeExperimentImageGeometry.ComputeVerticalFlipIntrinsics(
                480f,
                481f,
                320f,
                200f,
                400,
                out float flippedFx,
                out float flippedFy,
                out float flippedCx,
                out float flippedCy);
            AssertApproximately(flippedFx, 480f, "flippedFx");
            AssertApproximately(flippedFy, -481f, "flippedFy");
            AssertApproximately(flippedCx, 320f, "flippedCx");
            AssertApproximately(flippedCy, 199f, "flippedCy");

            ProbeExperimentImageGeometry.ComputeCurrentResolutionIntrinsics(
                500f,
                500f,
                500f,
                500f,
                1000,
                1000,
                800,
                600,
                out float currentFx,
                out float currentFy,
                out float currentCx,
                out float currentCy);
            AssertApproximately(currentFx, 400f, "currentFx");
            AssertApproximately(currentFy, 400f, "currentFy");
            AssertApproximately(currentCx, 400f, "currentCx");
            AssertApproximately(currentCy, 300f, "currentCy");

            Pose markerFromBase = new Pose(
                new Vector3(1f, 2f, 3f),
                Quaternion.Euler(0f, 30f, 0f));
            Pose adjustedMarkerFromBase = MarkerBaseCalibrationMath.ApplyMarkerPlaneStickDelta(
                markerFromBase,
                new Vector2(0.5f, -0.25f),
                0.02f,
                2f);
            AssertApproximately(adjustedMarkerFromBase.position, new Vector3(1.02f, 1.99f, 3f), "adjustedMarkerFromBase.position");
            AssertApproximately(adjustedMarkerFromBase.rotation, markerFromBase.rotation, "adjustedMarkerFromBase.rotation");

            Pose unityCameraFromMarker = AprilTagPoseConversion.ConvertCameraFromMarkerCvToUnity(
                new Pose(new Vector3(0.1f, 0.2f, 1.5f), Quaternion.identity));
            AssertApproximately(unityCameraFromMarker.position, new Vector3(0.1f, -0.2f, 1.5f), "unityCameraFromMarker.position");
            AssertApproximately(unityCameraFromMarker.rotation, Quaternion.identity, "unityCameraFromMarker.rotation");

            var robotBuffer = new RobotPoseSampleBuffer(8, 5.0);
            robotBuffer.Add(new RobotPoseSample
            {
                sequence = 1,
                poseValid = true,
                baseFromProbe = new Pose(Vector3.zero, Quaternion.identity),
                robotTimestampSeconds = 10.0,
                samplePcTimestampUnixSeconds = 100.0,
                pcReceiveUnixSeconds = 100.0,
                pcSendUnixSeconds = 100.0
            });
            robotBuffer.Add(new RobotPoseSample
            {
                sequence = 2,
                poseValid = true,
                baseFromProbe = new Pose(new Vector3(1f, 0f, 0f), Quaternion.Euler(0f, 90f, 0f)),
                robotTimestampSeconds = 10.1,
                samplePcTimestampUnixSeconds = 100.1,
                pcReceiveUnixSeconds = 100.1,
                pcSendUnixSeconds = 100.1
            });

            bool interpolated = robotBuffer.TrySampleAt(
                100.05,
                99.95,
                0.1,
                0.1,
                0.002,
                out Pose interpolatedPose,
                out RobotPoseTimingInfo interpolatedTiming);
            if (!interpolated)
            {
                throw new System.Exception($"Robot buffer interpolation failed: {interpolatedTiming.invalidReason}");
            }

            AssertApproximately(interpolatedPose.position, new Vector3(0.5f, 0f, 0f), "interpolatedPose.position");
            AssertApproximately(interpolatedTiming.robotTimestampSeconds, 10.05f, "interpolatedTiming.robotTimestampSeconds");
            if (interpolatedTiming.mode != "interpolated")
            {
                throw new System.Exception($"Expected interpolated mode, got {interpolatedTiming.mode}.");
            }

            bool tooOld = robotBuffer.TrySampleAt(
                99.0,
                98.9,
                0.1,
                0.1,
                0.002,
                out _,
                out RobotPoseTimingInfo tooOldTiming);
            if (tooOld || tooOldTiming.invalidReason != "too_old")
            {
                throw new System.Exception($"Expected too_old, got valid={tooOld} reason={tooOldTiming.invalidReason}.");
            }

            string matrixJson = ProbeExperimentJson.WritePoseMatrix(worldFromCamera);
            if (!matrixJson.Contains("\"position\"") || !matrixJson.Contains("\"rotation_xyzw\"") || !matrixJson.Contains("\"matrix4x4\""))
            {
                throw new System.Exception("Pose JSON does not include the expected fields.");
            }

            Debug.Log("Probe experiment pose math self test passed.");
        }

        /// <summary>
        /// Fails when two positions differ by more than a small floating-point tolerance.
        /// </summary>
        private static void AssertApproximately(Vector3 actual, Vector3 expected, string label)
        {
            if ((actual - expected).magnitude > 0.0001f)
            {
                throw new System.Exception($"{label}: expected {expected}, got {actual}");
            }
        }

        /// <summary>
        /// Fails when two rotations differ by more than a small angular tolerance.
        /// </summary>
        private static void AssertApproximately(Quaternion actual, Quaternion expected, string label)
        {
            if (Quaternion.Angle(actual, expected) > 0.0001f)
            {
                throw new System.Exception($"{label}: expected {expected.eulerAngles}, got {actual.eulerAngles}");
            }
        }

        /// <summary>
        /// Fails when two scalar values differ by more than a small tolerance.
        /// </summary>
        private static void AssertApproximately(float actual, float expected, string label)
        {
            if (Mathf.Abs(actual - expected) > 0.0001f)
            {
                throw new System.Exception($"{label}: expected {expected}, got {actual}");
            }
        }

        /// <summary>
        /// Fails when two double values differ by more than a small tolerance.
        /// </summary>
        private static void AssertApproximately(double actual, double expected, string label)
        {
            if (System.Math.Abs(actual - expected) > 0.0001)
            {
                throw new System.Exception($"{label}: expected {expected}, got {actual}");
            }
        }
    }
}
