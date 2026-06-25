using System.Collections.Generic;
using System.Text;
using Meta.XR;
using Unity.Collections;
using UnityEngine;

namespace PassthroughCameraSamples.ProbeExperiment
{
    /// <summary>
    /// Uses the first stable AprilTag observations to lock the robot base pose in Unity world coordinates.
    /// </summary>
    public class FiducialAnchorInitializer : MonoBehaviour
    {
        [SerializeField] private AprilTagDetectorBehaviour m_detector;
        [SerializeField] private int m_targetTagId;
        [SerializeField] private float m_tagSizeMeters = 0.08f;
        [SerializeField] private Vector3 m_markerFromBasePosition;
        [SerializeField] private Vector3 m_markerFromBaseEulerDegrees;
        [SerializeField] private int m_anchorLockMinFrames = 15;
        [SerializeField] private float m_anchorMaxPositionStdMeters = 0.01f;
        [SerializeField] private float m_anchorMaxRotationStdDegrees = 2f;

        private readonly List<Pose> m_worldFromBaseSamples = new();
        private readonly List<Pose> m_worldFromMarkerSamples = new();
        private readonly List<AprilTagDetection> m_detectionSamples = new();
        private Pose m_lockedWorldFromBase;
        private Pose m_lockedWorldFromMarker;
        private Pose m_calibrationWorldFromMarker;
        private float m_lockedPositionStdMeters;
        private float m_lockedRotationMaxDegrees;
        private string m_status = "Waiting for AprilTag.";

        public bool IsLocked { get; private set; }
        public Pose WorldFromBase => m_lockedWorldFromBase;
        public string Status => m_status;
        public int StableSampleCount => m_worldFromBaseSamples.Count;
        public float LockedPositionStdMeters => m_lockedPositionStdMeters;
        public float LockedRotationMaxDegrees => m_lockedRotationMaxDegrees;
        public int TargetTagId => m_targetTagId;
        public float TagSizeMeters => m_tagSizeMeters;
        public bool DetectorAvailable => m_detector != null && m_detector.IsAvailable;
        public string DetectorStatus => m_detector != null ? m_detector.Status : "AprilTag detector reference is missing.";
        public bool HasLatestMarkerObservation { get; private set; }
        public Pose LatestWorldFromMarker { get; private set; }
        public AprilTagDetection LatestDetection { get; private set; }
        public bool HasCalibrationWorldFromMarker { get; private set; }
        public Pose CalibrationWorldFromMarker => HasCalibrationWorldFromMarker ? m_calibrationWorldFromMarker : LatestWorldFromMarker;
        public int MarkerAnchorSampleCount => m_worldFromMarkerSamples.Count;

        /// <summary>
        /// Physical transform from the AprilTag marker frame to the robot base frame, T_M_B.
        /// </summary>
        public Pose MarkerFromBase => new Pose(m_markerFromBasePosition, Quaternion.Euler(m_markerFromBaseEulerDegrees));

        /// <summary>
        /// Updates the measured marker-to-base transform used by subsequent anchor samples.
        /// </summary>
        public void SetMarkerFromBase(Pose markerFromBase, bool resetAnchorSamples = true)
        {
            m_markerFromBasePosition = markerFromBase.position;
            m_markerFromBaseEulerDegrees = markerFromBase.rotation.eulerAngles;

            if (resetAnchorSamples)
            {
                ClearAnchorSamples();
            }
        }

        /// <summary>
        /// Clears collected anchor samples so future locking uses the latest calibration.
        /// </summary>
        public void ClearAnchorSamples()
        {
            m_worldFromBaseSamples.Clear();
            m_detectionSamples.Clear();
            IsLocked = false;
            m_lockedWorldFromBase = default;
            m_lockedWorldFromMarker = default;
            m_lockedPositionStdMeters = 0f;
            m_lockedRotationMaxDegrees = 0f;
        }

        /// <summary>
        /// Clears the fixed marker pose used only for in-headset manual calibration visualization.
        /// </summary>
        public void ClearCalibrationWorldFromMarker()
        {
            HasCalibrationWorldFromMarker = false;
            m_calibrationWorldFromMarker = default;
            m_worldFromMarkerSamples.Clear();
        }

        /// <summary>
        /// Locks T_W_B from the fixed tag anchor and the manually adjusted T_M_B.
        /// </summary>
        public bool TryLockBaseFromCalibration()
        {
            if (IsLocked)
            {
                return true;
            }

            if (!HasCalibrationWorldFromMarker)
            {
                m_status = "Cannot lock robot base before the AprilTag marker is anchored.";
                return false;
            }

            m_lockedWorldFromMarker = m_calibrationWorldFromMarker;
            m_lockedWorldFromBase = ProbeExperimentPoseMath.ComputeWorldFromBase(
                m_calibrationWorldFromMarker,
                MarkerFromBase);
            IsLocked = true;
            m_status = "Robot base anchor locked from manual marker-base calibration.";
            return true;
        }

        /// <summary>
        /// Attempts one anchor update from the latest passthrough frame.
        /// When enough stable samples are collected, locks T_W_B and returns true.
        /// </summary>
        public bool TryUpdateAnchor(
            NativeArray<Color32> pixels,
            Vector2Int resolution,
            PassthroughCameraAccess.CameraIntrinsics intrinsics,
            Pose worldFromCamera,
            bool collectAnchorSamples = true)
        {
            if (IsLocked)
            {
                return true;
            }

            if (!collectAnchorSamples && HasCalibrationWorldFromMarker)
            {
                m_status = "AprilTag marker anchored. Adjust the base candidate axis and press left Y to anchor base.";
                return false;
            }

            if (m_detector == null)
            {
                m_status = "AprilTag detector reference is missing.";
                return false;
            }

            if (!m_detector.TryDetect(pixels, resolution, intrinsics, m_tagSizeMeters, m_targetTagId, out AprilTagDetection detection))
            {
                m_status = m_detector.Status;
                return false;
            }

            Pose worldFromMarker = ProbeExperimentPoseMath.Compose(worldFromCamera, detection.CameraFromMarker);
            LatestWorldFromMarker = worldFromMarker;
            LatestDetection = detection;
            HasLatestMarkerObservation = true;

            if (!collectAnchorSamples)
            {
                m_worldFromMarkerSamples.Add(worldFromMarker);
                TrimPoseSamples(m_worldFromMarkerSamples);

                if (m_worldFromMarkerSamples.Count < m_anchorLockMinFrames)
                {
                    m_status = $"Collecting AprilTag marker anchor samples {m_worldFromMarkerSamples.Count}/{m_anchorLockMinFrames}.";
                    return false;
                }

                Pose averagedMarker = AveragePoses(m_worldFromMarkerSamples);
                float markerPositionStd = ComputePositionStd(m_worldFromMarkerSamples, averagedMarker.position);
                float markerRotationMax = ComputeRotationMax(m_worldFromMarkerSamples, averagedMarker.rotation);

                if (markerPositionStd > m_anchorMaxPositionStdMeters || markerRotationMax > m_anchorMaxRotationStdDegrees)
                {
                    m_status = $"Marker anchor unstable: position std {markerPositionStd:F4}m, rotation max {markerRotationMax:F2}deg.";
                    return false;
                }

                m_calibrationWorldFromMarker = averagedMarker;
                m_lockedWorldFromMarker = averagedMarker;
                m_lockedPositionStdMeters = markerPositionStd;
                m_lockedRotationMaxDegrees = markerRotationMax;
                HasCalibrationWorldFromMarker = true;
                m_status = "AprilTag marker anchored. Adjust the base candidate axis and press left Y to anchor base.";
                return false;
            }

            Pose worldFromBase = ProbeExperimentPoseMath.ComputeWorldFromBase(worldFromMarker, MarkerFromBase);
            m_worldFromBaseSamples.Add(worldFromBase);
            m_detectionSamples.Add(detection);

            TrimSamples();

            if (m_worldFromBaseSamples.Count < m_anchorLockMinFrames)
            {
                m_status = $"Collecting AprilTag samples {m_worldFromBaseSamples.Count}/{m_anchorLockMinFrames}.";
                return false;
            }

            Pose averaged = AveragePoses(m_worldFromBaseSamples);
            float positionStd = ComputePositionStd(m_worldFromBaseSamples, averaged.position);
            float rotationMax = ComputeRotationMax(m_worldFromBaseSamples, averaged.rotation);

            if (positionStd > m_anchorMaxPositionStdMeters || rotationMax > m_anchorMaxRotationStdDegrees)
            {
                m_status = $"Anchor samples unstable: position std {positionStd:F4}m, rotation max {rotationMax:F2}deg.";
                return false;
            }

            m_lockedWorldFromBase = averaged;
            m_lockedWorldFromMarker = worldFromMarker;
            m_lockedPositionStdMeters = positionStd;
            m_lockedRotationMaxDegrees = rotationMax;
            IsLocked = true;
            m_status = "AprilTag anchor locked.";
            return true;
        }

        /// <summary>
        /// Builds the one-time anchor initialization record written to anchor_init.json.
        /// </summary>
        public string BuildAnchorInitJson()
        {
            var sb = new StringBuilder(1024);
            sb.Append('{');
            sb.Append("\"tag_family\":\"tag36h11\",");
            sb.Append("\"target_tag_id\":");
            sb.Append(m_targetTagId);
            sb.Append(",\"tag_size_meters\":");
            ProbeExperimentJson.AppendFloat(sb, m_tagSizeMeters);
            sb.Append(",\"sample_count\":");
            sb.Append(m_worldFromBaseSamples.Count > 0 ? m_worldFromBaseSamples.Count : m_worldFromMarkerSamples.Count);
            sb.Append(",\"manual_marker_base_calibration\":");
            sb.Append(m_worldFromBaseSamples.Count == 0 && HasCalibrationWorldFromMarker ? "true" : "false");
            sb.Append(",\"marker_anchor_sample_count\":");
            sb.Append(m_worldFromMarkerSamples.Count);
            sb.Append(",\"position_std_meters\":");
            ProbeExperimentJson.AppendFloat(sb, m_lockedPositionStdMeters);
            sb.Append(",\"rotation_max_degrees\":");
            ProbeExperimentJson.AppendFloat(sb, m_lockedRotationMaxDegrees);
            sb.Append(",\"T_M_B\":");
            ProbeExperimentJson.WritePoseMatrix(sb, MarkerFromBase);
            sb.Append(",\"T_W_M_locked\":");
            ProbeExperimentJson.WritePoseMatrix(sb, m_lockedWorldFromMarker);
            sb.Append(",\"T_W_B_locked\":");
            ProbeExperimentJson.WritePoseMatrix(sb, m_lockedWorldFromBase);
            sb.Append('}');
            return sb.ToString();
        }

        /// <summary>
        /// Keeps only the rolling window used for the stability check.
        /// </summary>
        private void TrimSamples()
        {
            TrimPoseSamples(m_worldFromBaseSamples);

            while (m_detectionSamples.Count > m_anchorLockMinFrames)
            {
                m_detectionSamples.RemoveAt(0);
            }
        }

        /// <summary>
        /// Keeps only the rolling pose window used for stability checks.
        /// </summary>
        private void TrimPoseSamples(List<Pose> poses)
        {
            while (poses.Count > m_anchorLockMinFrames)
            {
                poses.RemoveAt(0);
            }
        }

        /// <summary>
        /// Averages positions linearly and quaternions with hemisphere correction.
        /// </summary>
        private static Pose AveragePoses(IReadOnlyList<Pose> poses)
        {
            Vector3 position = Vector3.zero;
            Vector4 rotationAccumulator = Vector4.zero;
            Quaternion reference = poses[0].rotation;

            foreach (Pose pose in poses)
            {
                position += pose.position;
                Quaternion rotation = pose.rotation;
                if (Quaternion.Dot(reference, rotation) < 0f)
                {
                    rotation = new Quaternion(-rotation.x, -rotation.y, -rotation.z, -rotation.w);
                }

                rotationAccumulator += new Vector4(rotation.x, rotation.y, rotation.z, rotation.w);
            }

            position /= poses.Count;
            rotationAccumulator /= poses.Count;
            Quaternion averagedRotation = new Quaternion(
                rotationAccumulator.x,
                rotationAccumulator.y,
                rotationAccumulator.z,
                rotationAccumulator.w);
            averagedRotation.Normalize();
            return new Pose(position, averagedRotation);
        }

        /// <summary>
        /// Computes the RMS position spread of the current anchor samples.
        /// </summary>
        private static float ComputePositionStd(IReadOnlyList<Pose> poses, Vector3 mean)
        {
            float sumSquared = 0f;
            foreach (Pose pose in poses)
            {
                float distance = Vector3.Distance(pose.position, mean);
                sumSquared += distance * distance;
            }

            return Mathf.Sqrt(sumSquared / poses.Count);
        }

        /// <summary>
        /// Computes the maximum angular deviation from the averaged anchor rotation.
        /// </summary>
        private static float ComputeRotationMax(IReadOnlyList<Pose> poses, Quaternion mean)
        {
            float maxAngle = 0f;
            foreach (Pose pose in poses)
            {
                maxAngle = Mathf.Max(maxAngle, Quaternion.Angle(mean, pose.rotation));
            }

            return maxAngle;
        }
    }
}
