using System;
using System.Collections;
using System.IO;
using System.Text;
using Meta.XR;
using Meta.XR.Samples;
using Unity.Collections;
using UnityEngine;
using UnityEngine.UI;

namespace PassthroughCameraSamples.ProbeExperiment
{
    /// <summary>
    /// Coordinates AprilTag anchor initialization, stereo camera capture, robot pose sampling, and JSONL logging.
    /// </summary>
    [MetaCodeSample("PassthroughCameraApiSamples-ProbeExperiment")]
    public class ProbeExperimentCaptureManager : MonoBehaviour
    {
        private enum ExperimentState
        {
            WaitingForMarker,
            AnchorLocked,
            Recording,
            Stopped
        }

        [SerializeField] private PassthroughCameraAccess m_leftCameraAccess;
        [SerializeField] private PassthroughCameraAccess m_rightCameraAccess;
        [SerializeField] private FiducialAnchorInitializer m_anchorInitializer;
        [SerializeField] private MarkerBaseCalibrationController m_markerBaseCalibrationController;
        [SerializeField] private RobotProbePoseProvider m_robotProbePoseProvider;
        [SerializeField] private Text m_debugText;
        [SerializeField] private string m_outputFolderName = "ProbeExperimentCaptures";
        [SerializeField] private float m_captureIntervalSeconds = 1f / 30f;
        [SerializeField] private bool m_recordLeftCamera = true;
        [SerializeField] private bool m_recordRightCamera = true;
        [SerializeField] private bool m_requireMarkerBaseCalibration = true;

        private ExperimentState m_state = ExperimentState.WaitingForMarker;
        private string m_sessionRoot;
        private string m_leftImageFolder;
        private string m_rightImageFolder;
        private string m_framesJsonlPath;
        private string m_anchorInitJsonPath;
        private StreamWriter m_frameWriter;
        private int m_frameIndex;
        private float m_nextCaptureTime;
        private bool m_anchorInitWritten;

        /// <summary>
        /// Waits for requested passthrough cameras, creates the output session, and opens frames.jsonl.
        /// </summary>
        private IEnumerator Start()
        {
            if (!ValidateReferences())
            {
                enabled = false;
                yield break;
            }

            while (!AreRequestedCamerasPlaying())
            {
                yield return null;
            }

            CreateSessionFolders();
            SetupMarkerBaseCalibration();
            WriteCalibrationJson();
            m_frameWriter = new StreamWriter(m_framesJsonlPath, append: false, Encoding.UTF8);
            UpdateDebugText("WaitingForMarker");
        }

        /// <summary>
        /// Runs the capture state machine and maps controller buttons to single-frame or continuous capture.
        /// </summary>
        private void Update()
        {
            if (!AreRequestedCamerasPlaying())
            {
                return;
            }

            if (m_state == ExperimentState.WaitingForMarker)
            {
                TryInitializeAnchor();
            }

            if (InputManager.IsButtonADownOrPinchStarted())
            {
                CaptureFrameIfReady(singleFrame: true);
            }

            if (InputManager.IsButtonBDownOrMiddleFingerPinchStarted())
            {
                ToggleRecording();
            }

            if (m_state == ExperimentState.Recording && Time.time >= m_nextCaptureTime)
            {
                CaptureFrameIfReady(singleFrame: false);
                m_nextCaptureTime += m_captureIntervalSeconds;

                if (Time.time > m_nextCaptureTime + m_captureIntervalSeconds)
                {
                    m_nextCaptureTime = Time.time + m_captureIntervalSeconds;
                }
            }
        }

        /// <summary>
        /// Releases the frame log writer when the component is disabled.
        /// </summary>
        private void OnDisable()
        {
            CloseFrameWriter();
        }

        /// <summary>
        /// Releases the frame log writer when the app exits.
        /// </summary>
        private void OnApplicationQuit()
        {
            CloseFrameWriter();
        }

        /// <summary>
        /// Verifies all required scene references and capture options before recording starts.
        /// </summary>
        private bool ValidateReferences()
        {
            if (m_leftCameraAccess == null && m_rightCameraAccess == null)
            {
                Debug.LogError("ProbeExperiment: At least one PassthroughCameraAccess reference is required.", this);
                UpdateDebugText("Missing camera access");
                return false;
            }

            if (m_anchorInitializer == null)
            {
                Debug.LogError("ProbeExperiment: FiducialAnchorInitializer reference is required.", this);
                UpdateDebugText("Missing anchor initializer");
                return false;
            }

            if (Application.platform == RuntimePlatform.Android && !m_anchorInitializer.DetectorAvailable)
            {
                string status = m_anchorInitializer.DetectorStatus;
                Debug.LogError($"ProbeExperiment: AprilTag detector is not available. {status}", this);
                UpdateDebugText(status);
                return false;
            }

            if (m_robotProbePoseProvider == null)
            {
                Debug.LogError("ProbeExperiment: RobotProbePoseProvider reference is required.", this);
                UpdateDebugText("Missing robot pose provider");
                return false;
            }

            if (!m_recordLeftCamera && !m_recordRightCamera)
            {
                Debug.LogError("ProbeExperiment: At least one camera must be enabled for recording.", this);
                UpdateDebugText("No recording camera enabled");
                return false;
            }

            return true;
        }

        /// <summary>
        /// Checks whether every enabled passthrough camera is already streaming.
        /// </summary>
        private bool AreRequestedCamerasPlaying()
        {
            bool leftReady = !m_recordLeftCamera || (m_leftCameraAccess != null && m_leftCameraAccess.IsPlaying);
            bool rightReady = !m_recordRightCamera || (m_rightCameraAccess != null && m_rightCameraAccess.IsPlaying);
            return leftReady && rightReady;
        }

        /// <summary>
        /// Feeds the latest anchor camera frame to the fiducial initializer until T_W_B is locked.
        /// </summary>
        private void TryInitializeAnchor()
        {
            if (m_anchorInitializer.IsLocked)
            {
                m_state = ExperimentState.AnchorLocked;
                WriteAnchorInitJson();
                UpdateDebugText("AnchorLocked");
                return;
            }

            PassthroughCameraAccess anchorCamera = m_leftCameraAccess != null ? m_leftCameraAccess : m_rightCameraAccess;
            if (anchorCamera == null || !anchorCamera.IsPlaying || !anchorCamera.IsUpdatedThisFrame)
            {
                return;
            }

            NativeArray<Color32> pixels = anchorCamera.GetColors();
            Pose worldFromCamera = anchorCamera.GetCameraPose();
            bool collectAnchorSamples = !m_requireMarkerBaseCalibration;
            bool locked = m_anchorInitializer.TryUpdateAnchor(
                pixels,
                anchorCamera.CurrentResolution,
                anchorCamera.Intrinsics,
                worldFromCamera,
                collectAnchorSamples);

            if (!locked)
            {
                if (!collectAnchorSamples
                    && m_markerBaseCalibrationController != null
                    && m_anchorInitializer.HasCalibrationWorldFromMarker)
                {
                    UpdateDebugText(m_markerBaseCalibrationController.Status);
                }
                else
                {
                    UpdateDebugText(m_anchorInitializer.Status);
                }

                return;
            }

            m_state = ExperimentState.AnchorLocked;
            WriteAnchorInitJson();
            UpdateDebugText("AnchorLocked");
        }

        /// <summary>
        /// Starts or stops continuous recording after the AprilTag anchor has been locked.
        /// </summary>
        private void ToggleRecording()
        {
            if (!m_anchorInitializer.IsLocked)
            {
                UpdateDebugText("Cannot record before anchor is locked.");
                return;
            }

            if (m_state == ExperimentState.Recording)
            {
                m_state = ExperimentState.AnchorLocked;
                FlushFrames();
                UpdateDebugText("Recording stopped");
                return;
            }

            if (m_state == ExperimentState.AnchorLocked || m_state == ExperimentState.Stopped)
            {
                m_state = ExperimentState.Recording;
                m_nextCaptureTime = Time.time;
                UpdateDebugText("Recording");
            }
        }

        /// <summary>
        /// Captures one synchronized log record using the current camera frames and current robot pose sample.
        /// </summary>
        private void CaptureFrameIfReady(bool singleFrame)
        {
            if (!m_anchorInitializer.IsLocked)
            {
                UpdateDebugText("Cannot capture before anchor is locked.");
                return;
            }

            if (singleFrame && m_state == ExperimentState.WaitingForMarker)
            {
                return;
            }

            try
            {
                CameraFrameData leftFrame = default;
                CameraFrameData rightFrame = default;
                bool hasLeft = m_recordLeftCamera && CaptureSingleCameraFrame(m_leftCameraAccess, "left", m_leftImageFolder, out leftFrame);
                bool hasRight = m_recordRightCamera && CaptureSingleCameraFrame(m_rightCameraAccess, "right", m_rightImageFolder, out rightFrame);
                RobotPoseFrameData leftRobot = QueryRobotPoseForFrame(hasLeft, leftFrame);
                RobotPoseFrameData rightRobot = QueryRobotPoseForFrame(hasRight, rightFrame);

                WriteFrameJsonLine(
                    hasLeft,
                    leftFrame,
                    leftRobot,
                    hasRight,
                    rightFrame,
                    rightRobot);

                m_frameIndex++;
                UpdateDebugText($"Saved frame {m_frameIndex - 1}");
            }
            catch (Exception e)
            {
                Debug.LogError($"ProbeExperiment: Failed to capture frame {m_frameIndex}. {e}", this);
                UpdateDebugText($"Capture failed: {m_frameIndex}");
            }
        }

        /// <summary>
        /// Queries the robot provider for the pose that matches one captured camera frame timestamp.
        /// </summary>
        private RobotPoseFrameData QueryRobotPoseForFrame(bool hasFrame, CameraFrameData frame)
        {
            if (!hasFrame)
            {
                return new RobotPoseFrameData
                {
                    valid = false,
                    timingInfo = new RobotPoseTimingInfo
                    {
                        valid = false,
                        mode = "invalid",
                        invalidReason = "no_camera_frame"
                    }
                };
            }

            bool valid = m_robotProbePoseProvider.TryGetBaseFromProbeAt(
                frame.timestampUnixSeconds,
                out Pose baseFromProbe,
                out RobotPoseTimingInfo timingInfo);

            return new RobotPoseFrameData
            {
                valid = valid,
                baseFromProbe = baseFromProbe,
                timingInfo = timingInfo
            };
        }

        /// <summary>
        /// Saves one camera frame as raw RGB and returns the timestamp, intrinsics, and T_W_C pose for logging.
        /// </summary>
        private bool CaptureSingleCameraFrame(
            PassthroughCameraAccess cameraAccess,
            string cameraName,
            string imageFolder,
            out CameraFrameData frameData)
        {
            frameData = default;

            if (cameraAccess == null || !cameraAccess.IsPlaying || !cameraAccess.IsUpdatedThisFrame)
            {
                Debug.LogWarning($"ProbeExperiment: {cameraName} camera is not updated this frame. Skip.", this);
                return false;
            }

            Vector2Int resolution = cameraAccess.CurrentResolution;
            NativeArray<Color32> pixels = cameraAccess.GetColors();
            if (!pixels.IsCreated || pixels.Length == 0)
            {
                throw new InvalidOperationException($"{cameraName} GetColors() returned empty data.");
            }

            string relativeImagePath = $"{cameraName}/{m_frameIndex}.raw";
            string absoluteImagePath = Path.Combine(imageFolder, $"{m_frameIndex}.raw");
            byte[] rgbBytes = ConvertColor32ToRgbFlipVertical(pixels, resolution.x, resolution.y);
            File.WriteAllBytes(absoluteImagePath, rgbBytes);

            frameData = new CameraFrameData
            {
                cameraName = cameraName,
                relativeImagePath = relativeImagePath,
                timestampUnixSeconds = ProbeExperimentJson.ToUnixSeconds(cameraAccess.Timestamp),
                worldFromCamera = cameraAccess.GetCameraPose(),
                intrinsics = cameraAccess.Intrinsics,
                resolution = resolution
            };

            return true;
        }

        /// <summary>
        /// Converts Unity Color32 pixels to RGB byte order and flips vertically for conventional image coordinates.
        /// </summary>
        private static byte[] ConvertColor32ToRgbFlipVertical(NativeArray<Color32> pixels, int width, int height)
        {
            int pixelCount = width * height;
            byte[] rgb = new byte[pixelCount * 3];

            for (int y = 0; y < height; y++)
            {
                int dstY = height - 1 - y;
                for (int x = 0; x < width; x++)
                {
                    int srcIdx = y * width + x;
                    int dstIdx = dstY * width + x;
                    Color32 c = pixels[srcIdx];
                    int dstBase = dstIdx * 3;
                    rgb[dstBase] = c.r;
                    rgb[dstBase + 1] = c.g;
                    rgb[dstBase + 2] = c.b;
                }
            }

            return rgb;
        }

        /// <summary>
        /// Writes one JSONL frame entry containing head pose, anchor pose, robot pose, and per-camera data.
        /// </summary>
        private void WriteFrameJsonLine(
            bool hasLeft,
            CameraFrameData leftFrame,
            RobotPoseFrameData leftRobot,
            bool hasRight,
            CameraFrameData rightFrame,
            RobotPoseFrameData rightRobot)
        {
            var sb = new StringBuilder(4096);
            Pose worldFromBase = m_anchorInitializer.WorldFromBase;
            OVRPose headPose = OVRPlugin.GetNodePoseStateImmediate(OVRPlugin.Node.Head).Pose.ToOVRPose();
            RobotPoseFrameData referenceRobot = hasLeft ? leftRobot : hasRight ? rightRobot : default;
            string referenceCamera = hasLeft ? "left" : hasRight ? "right" : "none";

            sb.Append('{');
            sb.Append("\"frame_id\":");
            sb.Append(m_frameIndex);
            sb.Append(",\"unity_time\":");
            ProbeExperimentJson.AppendDouble(sb, Time.realtimeSinceStartupAsDouble);
            sb.Append(",\"anchor_locked\":true");
            sb.Append(",\"robot_pose_valid\":");
            sb.Append(referenceRobot.valid ? "true" : "false");
            sb.Append(",\"robot_reference_camera\":");
            ProbeExperimentJson.WriteString(sb, referenceCamera);
            sb.Append(",\"robot_timestamp_seconds\":");
            ProbeExperimentJson.AppendDouble(sb, referenceRobot.timingInfo.robotTimestampSeconds);
            sb.Append(",\"robot_timing\":");
            WriteRobotTimingInfo(sb, referenceRobot.timingInfo);
            sb.Append(",\"T_W_H\":");
            ProbeExperimentJson.WritePoseMatrix(sb, new Pose(headPose.position, headPose.orientation));
            sb.Append(",\"T_W_B\":");
            ProbeExperimentJson.WritePoseMatrix(sb, worldFromBase);
            sb.Append(",\"T_B_P\":");
            WritePoseOrNull(sb, referenceRobot.valid, referenceRobot.baseFromProbe);

            WriteCameraFrameSection(sb, "left", hasLeft, leftFrame, leftRobot, worldFromBase);
            WriteCameraFrameSection(sb, "right", hasRight, rightFrame, rightRobot, worldFromBase);

            sb.Append('}');
            m_frameWriter.WriteLine(sb.ToString());
        }

        /// <summary>
        /// Appends one camera subsection and computes T_C_P_gt when the robot pose sample is valid.
        /// </summary>
        private static void WriteCameraFrameSection(
            StringBuilder sb,
            string name,
            bool hasFrame,
            CameraFrameData frame,
            RobotPoseFrameData robotFrame,
            Pose worldFromBase)
        {
            sb.Append(",\"");
            sb.Append(name);
            sb.Append("\":");

            if (!hasFrame)
            {
                sb.Append("null");
                return;
            }

            sb.Append('{');
            sb.Append("\"image_path\":");
            ProbeExperimentJson.WriteString(sb, frame.relativeImagePath);
            sb.Append(",\"timestamp_unix_s\":");
            ProbeExperimentJson.AppendDouble(sb, frame.timestampUnixSeconds);
            sb.Append(",\"robot_pose_valid\":");
            sb.Append(robotFrame.valid ? "true" : "false");
            sb.Append(",\"robot_timing\":");
            WriteRobotTimingInfo(sb, robotFrame.timingInfo);
            sb.Append(",\"T_B_P_at_camera_time\":");
            WritePoseOrNull(sb, robotFrame.valid, robotFrame.baseFromProbe);
            sb.Append(",\"T_W_C\":");
            ProbeExperimentJson.WritePoseMatrix(sb, frame.worldFromCamera);
            sb.Append(",\"image_vertical_flip\":true");
            sb.Append(",\"pose_camera_frame\":\"passthrough_original\"");
            sb.Append(",\"intrinsics_convention\":\"vertical_flip_same_camera_negative_fy\"");
            sb.Append(",\"intrinsics\":");
            ProbeExperimentJson.WriteVerticalFlipIntrinsics(sb, frame.intrinsics, frame.resolution);
            sb.Append(",\"intrinsics_original\":");
            ProbeExperimentJson.WriteIntrinsics(sb, frame.intrinsics, frame.resolution);
            sb.Append(",\"intrinsics_flipped_same_camera\":");
            ProbeExperimentJson.WriteVerticalFlipIntrinsics(sb, frame.intrinsics, frame.resolution);
            sb.Append(",\"T_C_P_gt\":");

            if (robotFrame.valid)
            {
                Pose cameraFromProbe = ProbeExperimentPoseMath.ComputeCameraFromProbe(
                    frame.worldFromCamera,
                    worldFromBase,
                    robotFrame.baseFromProbe);
                ProbeExperimentJson.WritePoseMatrix(sb, cameraFromProbe);
            }
            else
            {
                sb.Append("null");
            }

            sb.Append('}');
        }

        /// <summary>
        /// Appends robot timing metadata used to audit camera/robot timestamp alignment.
        /// </summary>
        private static void WriteRobotTimingInfo(StringBuilder sb, RobotPoseTimingInfo timingInfo)
        {
            sb.Append('{');
            sb.Append("\"valid\":");
            sb.Append(timingInfo.valid ? "true" : "false");
            sb.Append(",\"mode\":");
            ProbeExperimentJson.WriteString(sb, timingInfo.mode);
            sb.Append(",\"invalid_reason\":");
            ProbeExperimentJson.WriteString(sb, timingInfo.invalidReason);
            sb.Append(",\"requested_timestamp_unix_s\":");
            ProbeExperimentJson.AppendDouble(sb, timingInfo.requestedTimestampUnixSeconds);
            sb.Append(",\"robot_timestamp_seconds\":");
            ProbeExperimentJson.AppendDouble(sb, timingInfo.robotTimestampSeconds);
            sb.Append(",\"pc_timestamp_unix_s\":");
            ProbeExperimentJson.AppendDouble(sb, timingInfo.pcTimestampUnixSeconds);
            sb.Append(",\"before_timestamp_unix_s\":");
            ProbeExperimentJson.AppendDouble(sb, timingInfo.beforeTimestampUnixSeconds);
            sb.Append(",\"after_timestamp_unix_s\":");
            ProbeExperimentJson.AppendDouble(sb, timingInfo.afterTimestampUnixSeconds);
            sb.Append(",\"robot_time_delta_s\":");
            ProbeExperimentJson.AppendDouble(sb, timingInfo.timeDeltaSeconds);
            sb.Append(",\"interpolation_span_s\":");
            ProbeExperimentJson.AppendDouble(sb, timingInfo.interpolationSpanSeconds);
            sb.Append(",\"pc_quest_offset_s\":");
            ProbeExperimentJson.AppendDouble(sb, timingInfo.pcQuestOffsetSeconds);
            sb.Append(",\"pc_quest_rtt_s\":");
            ProbeExperimentJson.AppendDouble(sb, timingInfo.pcQuestRttSeconds);
            sb.Append(",\"pose_raw\":");
            ProbeExperimentJson.WriteFloatArrayOrNull(sb, timingInfo.poseRaw);
            sb.Append(",\"normalized_data\":");
            ProbeExperimentJson.WriteFloatArrayOrNull(sb, timingInfo.normalizedData);
            sb.Append(",\"sequence\":");
            sb.Append(timingInfo.sequence);
            sb.Append('}');
        }

        /// <summary>
        /// Appends a pose JSON object or null when no timestamp-aligned robot pose is available.
        /// </summary>
        private static void WritePoseOrNull(StringBuilder sb, bool valid, Pose pose)
        {
            if (valid)
            {
                ProbeExperimentJson.WritePoseMatrix(sb, pose);
            }
            else
            {
                sb.Append("null");
            }
        }

        /// <summary>
        /// Creates a timestamped session folder with left and right image subfolders.
        /// </summary>
        private void CreateSessionFolders()
        {
            string sessionName = DateTime.UtcNow.ToString("yyyyMMdd_HHmmss");
            m_sessionRoot = Path.Combine(Application.persistentDataPath, m_outputFolderName, sessionName);
            m_leftImageFolder = Path.Combine(m_sessionRoot, "left");
            m_rightImageFolder = Path.Combine(m_sessionRoot, "right");
            m_framesJsonlPath = Path.Combine(m_sessionRoot, "frames.jsonl");
            m_anchorInitJsonPath = Path.Combine(m_sessionRoot, "anchor_init.json");

            Directory.CreateDirectory(m_sessionRoot);
            Directory.CreateDirectory(m_leftImageFolder);
            Directory.CreateDirectory(m_rightImageFolder);
            Debug.Log($"ProbeExperiment: Output root = {m_sessionRoot}", this);
        }

        /// <summary>
        /// Creates or initializes the runtime marker-base calibration controller.
        /// </summary>
        private void SetupMarkerBaseCalibration()
        {
            if (!m_requireMarkerBaseCalibration)
            {
                return;
            }

            if (m_markerBaseCalibrationController == null)
            {
                m_markerBaseCalibrationController = GetComponent<MarkerBaseCalibrationController>();
            }

            if (m_markerBaseCalibrationController == null)
            {
                m_markerBaseCalibrationController = gameObject.AddComponent<MarkerBaseCalibrationController>();
            }

            m_markerBaseCalibrationController.Initialize(m_anchorInitializer, m_sessionRoot);
        }

        /// <summary>
        /// Writes static experiment calibration values that should be constant for the full session.
        /// </summary>
        private void WriteCalibrationJson()
        {
            var sb = new StringBuilder(2048);
            sb.Append('{');
            sb.Append("\"tag_family\":\"tag36h11\",");
            sb.Append("\"target_tag_id\":");
            sb.Append(m_anchorInitializer.TargetTagId);
            sb.Append(",\"tag_size_meters\":");
            ProbeExperimentJson.AppendFloat(sb, m_anchorInitializer.TagSizeMeters);
            sb.Append(",\"T_M_B\":");
            ProbeExperimentJson.WritePoseMatrix(sb, m_anchorInitializer.MarkerFromBase);
            sb.Append(",\"record_left_camera\":");
            sb.Append(m_recordLeftCamera ? "true" : "false");
            sb.Append(",\"record_right_camera\":");
            sb.Append(m_recordRightCamera ? "true" : "false");
            sb.Append('}');

            File.WriteAllText(Path.Combine(m_sessionRoot, "calibration.json"), sb.ToString());
        }

        /// <summary>
        /// Writes anchor_init.json exactly once after the fiducial anchor is locked.
        /// </summary>
        private void WriteAnchorInitJson()
        {
            if (m_anchorInitWritten)
            {
                return;
            }

            WriteCalibrationJson();
            File.WriteAllText(m_anchorInitJsonPath, m_anchorInitializer.BuildAnchorInitJson());
            m_anchorInitWritten = true;
        }

        /// <summary>
        /// Flushes buffered JSONL data to storage without closing the writer.
        /// </summary>
        private void FlushFrames()
        {
            m_frameWriter?.Flush();
        }

        /// <summary>
        /// Flushes and closes frames.jsonl safely.
        /// </summary>
        private void CloseFrameWriter()
        {
            if (m_frameWriter == null)
            {
                return;
            }

            m_frameWriter.Flush();
            m_frameWriter.Dispose();
            m_frameWriter = null;
        }

        /// <summary>
        /// Updates the optional scene debug text if it is assigned.
        /// </summary>
        private void UpdateDebugText(string text)
        {
            if (m_debugText != null)
            {
                m_debugText.text = text;
            }
        }

        private struct CameraFrameData
        {
            public string cameraName;
            public string relativeImagePath;
            public double timestampUnixSeconds;
            public Pose worldFromCamera;
            public PassthroughCameraAccess.CameraIntrinsics intrinsics;
            public Vector2Int resolution;
        }

        private struct RobotPoseFrameData
        {
            public bool valid;
            public Pose baseFromProbe;
            public RobotPoseTimingInfo timingInfo;
        }
    }
}
