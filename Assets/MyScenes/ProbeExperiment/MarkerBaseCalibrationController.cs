using System;
using System.IO;
using System.Text;
using UnityEngine;

namespace PassthroughCameraSamples.ProbeExperiment
{
    /// <summary>
    /// Lets the user align the marker-to-base transform in-headset before anchor locking starts.
    /// </summary>
    public class MarkerBaseCalibrationController : MonoBehaviour
    {
        [SerializeField] private bool m_enableCalibration = true;
        [SerializeField] private float m_axisLengthMeters = 0.15f;
        [SerializeField] private float m_lineWidthMeters = 0.008f;
        [SerializeField] private float m_coarseMetersPerSecond = 0.02f;
        [SerializeField] private float m_fineMetersPerSecond = 0.002f;
        [SerializeField] private float m_stickDeadZone = 0.15f;

        private const string CoordinateConventionNote =
            "T_M_B is expressed in Unity/AprilTag marker coordinates. Robot T_B_P providers must convert robot right-handed SDK output into the same Unity-compatible base frame before logging.";

        private FiducialAnchorInitializer m_anchorInitializer;
        private string m_sessionRoot;
        private Transform m_markerAxisRoot;
        private Transform m_baseAxisRoot;
        private Material m_axisMaterial;
        private bool m_fineMode;

        public bool IsCalibrationSaved { get; private set; }
        public string Status { get; private set; } = "Marker-base calibration is waiting for AprilTag.";
        public string CalibrationFilePath { get; private set; }

        /// <summary>
        /// Connects this controller to the active anchor initializer and output session.
        /// </summary>
        public void Initialize(FiducialAnchorInitializer anchorInitializer, string sessionRoot)
        {
            m_anchorInitializer = anchorInitializer;
            m_sessionRoot = sessionRoot;
            CalibrationFilePath = Path.Combine(m_sessionRoot, "marker_base_calibration.json");
            EnsureAxisObjects();
        }

        /// <summary>
        /// Updates controller input, marker/base axis visualization, and save requests.
        /// </summary>
        private void Update()
        {
            if (!m_enableCalibration || m_anchorInitializer == null)
            {
                SetAxesActive(false);
                return;
            }

            if (!m_anchorInitializer.HasCalibrationWorldFromMarker)
            {
                SetAxesActive(false);
                Status = m_anchorInitializer.Status;
                return;
            }

            SetAxesActive(true);
            HandleModeToggle();
            HandleStickAdjustment();
            HandleSaveRequest();
            UpdateAxisPoses();
            UpdateStatusText();
        }

        /// <summary>
        /// Destroys runtime-only world-space axis objects owned by this controller.
        /// </summary>
        private void OnDestroy()
        {
            DestroyAxisRoot(m_markerAxisRoot);
            DestroyAxisRoot(m_baseAxisRoot);
        }

        /// <summary>
        /// Creates axis renderers lazily so the scene does not need manually authored calibration objects.
        /// </summary>
        private void EnsureAxisObjects()
        {
            if (m_markerAxisRoot != null && m_baseAxisRoot != null)
            {
                return;
            }

            m_axisMaterial = CreateAxisMaterial();
            m_markerAxisRoot = CreateAxisRoot("MarkerAxis");
            m_baseAxisRoot = CreateAxisRoot("BaseCandidateAxis");
            SetAxesActive(false);
        }

        /// <summary>
        /// Toggles between coarse and fine adjustment speed with the left X button.
        /// </summary>
        private void HandleModeToggle()
        {
            if (InputManager.IsLeftButtonXDown())
            {
                m_fineMode = !m_fineMode;
            }
        }

        /// <summary>
        /// Applies left-stick X/Y motion to the marker-frame translation of T_M_B.
        /// </summary>
        private void HandleStickAdjustment()
        {
            if (m_anchorInitializer.IsLocked)
            {
                return;
            }

            Vector2 stick = InputManager.GetLeftThumbstick();
            if (stick.sqrMagnitude < m_stickDeadZone * m_stickDeadZone)
            {
                return;
            }

            float speed = m_fineMode ? m_fineMetersPerSecond : m_coarseMetersPerSecond;
            Pose adjusted = MarkerBaseCalibrationMath.ApplyMarkerPlaneStickDelta(
                m_anchorInitializer.MarkerFromBase,
                stick,
                speed,
                Time.deltaTime);
            m_anchorInitializer.SetMarkerFromBase(adjusted);
            IsCalibrationSaved = false;
        }

        /// <summary>
        /// Saves the current T_M_B when the left Y button is pressed.
        /// </summary>
        private void HandleSaveRequest()
        {
            if (m_anchorInitializer.IsLocked)
            {
                return;
            }

            if (InputManager.IsLeftButtonYDown())
            {
                SaveCalibration();
            }
        }

        /// <summary>
        /// Writes the calibrated marker-to-base transform to the current capture session.
        /// </summary>
        public void SaveCalibration()
        {
            if (m_anchorInitializer == null)
            {
                return;
            }

            if (!m_anchorInitializer.HasCalibrationWorldFromMarker)
            {
                Status = "Cannot anchor robot base before the AprilTag marker anchor is locked.";
                return;
            }

            if (string.IsNullOrEmpty(m_sessionRoot))
            {
                m_sessionRoot = Application.persistentDataPath;
                CalibrationFilePath = Path.Combine(m_sessionRoot, "marker_base_calibration.json");
            }

            if (!m_anchorInitializer.TryLockBaseFromCalibration())
            {
                Status = m_anchorInitializer.Status;
                return;
            }

            Directory.CreateDirectory(m_sessionRoot);
            File.WriteAllText(CalibrationFilePath, BuildCalibrationJson(), Encoding.UTF8);
            IsCalibrationSaved = true;
        }

        /// <summary>
        /// Builds the JSON record used to document the manual marker-base calibration.
        /// </summary>
        private string BuildCalibrationJson()
        {
            var sb = new StringBuilder(1024);
            sb.Append('{');
            sb.Append("\"saved_unity_time\":");
            ProbeExperimentJson.AppendDouble(sb, Time.realtimeSinceStartupAsDouble);
            sb.Append(",\"saved_utc_unix_s\":");
            ProbeExperimentJson.AppendDouble(sb, ProbeExperimentJson.ToUnixSeconds(DateTime.UtcNow));
            sb.Append(",\"step_mode\":");
            ProbeExperimentJson.WriteString(sb, m_fineMode ? "fine" : "coarse");
            sb.Append(",\"coordinate_convention\":");
            ProbeExperimentJson.WriteString(sb, CoordinateConventionNote);
            sb.Append(",\"T_M_B\":");
            ProbeExperimentJson.WritePoseMatrix(sb, m_anchorInitializer.MarkerFromBase);
            sb.Append('}');
            return sb.ToString();
        }

        /// <summary>
        /// Updates the world poses of the marker and base candidate axes.
        /// </summary>
        private void UpdateAxisPoses()
        {
            Pose worldFromMarker = m_anchorInitializer.CalibrationWorldFromMarker;
            Pose worldFromBaseCandidate = ProbeExperimentPoseMath.Compose(
                worldFromMarker,
                m_anchorInitializer.MarkerFromBase);

            ApplyPose(m_markerAxisRoot, worldFromMarker);
            ApplyPose(m_baseAxisRoot, worldFromBaseCandidate);
        }

        /// <summary>
        /// Writes a concise status string for the capture manager debug text.
        /// </summary>
        private void UpdateStatusText()
        {
            Pose markerFromBase = m_anchorInitializer.MarkerFromBase;
            string mode = m_fineMode ? "Fine" : "Coarse";
            string saved = IsCalibrationSaved ? "saved" : "unsaved";
            string anchorState = m_anchorInitializer.IsLocked ? "base-anchored" : "tag-anchored";
            Status = $"Calibrate T_M_B {mode} {saved} {anchorState} pos=({markerFromBase.position.x:F4}, {markerFromBase.position.y:F4}, {markerFromBase.position.z:F4})";
        }

        /// <summary>
        /// Creates one colored three-axis root with X/Y/Z line renderers and labels.
        /// </summary>
        private Transform CreateAxisRoot(string axisName)
        {
            var root = new GameObject(axisName).transform;
            CreateAxisLine(root, "X", Vector3.right, Color.red);
            CreateAxisLine(root, "Y", Vector3.up, Color.green);
            CreateAxisLine(root, "Z", Vector3.forward, Color.blue);
            return root;
        }

        /// <summary>
        /// Creates one local-space axis line and endpoint label.
        /// </summary>
        private void CreateAxisLine(Transform root, string label, Vector3 direction, Color color)
        {
            var lineObject = new GameObject($"{label}Axis");
            lineObject.transform.SetParent(root, worldPositionStays: false);
            var line = lineObject.AddComponent<LineRenderer>();
            line.useWorldSpace = false;
            line.positionCount = 2;
            line.SetPosition(0, Vector3.zero);
            line.SetPosition(1, direction * m_axisLengthMeters);
            line.startWidth = m_lineWidthMeters;
            line.endWidth = m_lineWidthMeters;
            line.material = m_axisMaterial;
            line.startColor = color;
            line.endColor = color;

            var labelObject = new GameObject($"{label}Label");
            labelObject.transform.SetParent(root, worldPositionStays: false);
            labelObject.transform.localPosition = direction * (m_axisLengthMeters + 0.02f);
            TextMesh text = labelObject.AddComponent<TextMesh>();
            text.text = label;
            text.anchor = TextAnchor.MiddleCenter;
            text.alignment = TextAlignment.Center;
            text.characterSize = 0.025f;
            text.fontSize = 48;
            text.color = color;
        }

        /// <summary>
        /// Creates the simple unlit material shared by all axis line renderers.
        /// </summary>
        private static Material CreateAxisMaterial()
        {
            Shader shader = Shader.Find("Sprites/Default");
            if (shader == null)
            {
                shader = Shader.Find("Hidden/Internal-Colored");
            }

            return new Material(shader);
        }

        /// <summary>
        /// Applies a Unity pose to an axis root.
        /// </summary>
        private static void ApplyPose(Transform target, Pose pose)
        {
            target.SetPositionAndRotation(pose.position, pose.rotation);
        }

        /// <summary>
        /// Shows or hides all calibration axes.
        /// </summary>
        private void SetAxesActive(bool active)
        {
            if (m_markerAxisRoot != null)
            {
                m_markerAxisRoot.gameObject.SetActive(active);
            }

            if (m_baseAxisRoot != null)
            {
                m_baseAxisRoot.gameObject.SetActive(active);
            }
        }

        /// <summary>
        /// Destroys a runtime axis root without touching scene-authored objects.
        /// </summary>
        private static void DestroyAxisRoot(Transform axisRoot)
        {
            if (axisRoot != null)
            {
                Destroy(axisRoot.gameObject);
            }
        }
    }
}
