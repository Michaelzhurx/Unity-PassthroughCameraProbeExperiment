using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using Meta.XR;
using Meta.XR.Samples;
using Unity.Collections;
using UnityEngine;
using UnityEngine.UI;

namespace PassthroughCameraSamples.StereoCapture
{
    [MetaCodeSample("PassthroughCameraApiSamples-StereoCapture")]
    public class StereoCaptureManager : MonoBehaviour
    {
        [SerializeField] private PassthroughCameraAccess m_leftCameraAccess;
        [SerializeField] private PassthroughCameraAccess m_rightCameraAccess;
        [SerializeField] private Text m_debugText;
        [SerializeField] private string m_outputFolderName = "StereoCapturesRaw";
        [SerializeField] private float m_captureIntervalSeconds = 1f / 30f;

        private string m_outputRoot;
        private string m_leftImageFolder;
        private string m_rightImageFolder;
        private string m_leftSceneCameraJsonPath;
        private string m_rightSceneCameraJsonPath;

        private bool m_captureContinuously = false;
        private int m_captureIndex;
        private float m_nextCaptureTime;
        private bool m_hasUnflushedMetadata = false;

        private readonly Dictionary<string, SceneCameraEntry> m_leftSceneCameraDict = new();
        private readonly Dictionary<string, SceneCameraEntry> m_rightSceneCameraDict = new();

        private IEnumerator Start()
        {
            if (m_leftCameraAccess == null || m_rightCameraAccess == null)
            {
                Debug.LogError("PCA: Left and Right PassthroughCameraAccess references are required.");
                if (m_debugText != null)
                {
                    m_debugText.text = "PCA: Left and Right PassthroughCameraAccess references are required.";
                }
                enabled = false;
                yield break;
            }

            while (!m_leftCameraAccess.IsPlaying || !m_rightCameraAccess.IsPlaying)
            {
                yield return null;
            }

            m_outputRoot = Path.Combine(Application.persistentDataPath, m_outputFolderName);
            m_leftImageFolder = Path.Combine(m_outputRoot, "left");
            m_rightImageFolder = Path.Combine(m_outputRoot, "right");
            m_leftSceneCameraJsonPath = Path.Combine(m_outputRoot, "left_scene_camera.json");
            m_rightSceneCameraJsonPath = Path.Combine(m_outputRoot, "right_scene_camera.json");

            Directory.CreateDirectory(m_outputRoot);
            Directory.CreateDirectory(m_leftImageFolder);
            Directory.CreateDirectory(m_rightImageFolder);

            Debug.Log($"PCA: Output root = {m_outputRoot}");
            Debug.Log($"PCA: StereoCaptureManager is ready. Interval = {m_captureIntervalSeconds:F4}s");

            if (m_debugText != null)
            {
                m_debugText.text = $"Ready | dt={m_captureIntervalSeconds:F4}s";
            }
        }

        private void Update()
        {
            if (!m_leftCameraAccess.IsPlaying || !m_rightCameraAccess.IsPlaying)
            {
                return;
            }

            if (InputManager.IsButtonADownOrPinchStarted())
            {
                CaptureStereoFrame();
            }

            if (InputManager.IsButtonBDownOrMiddleFingerPinchStarted())
            {
                m_captureContinuously = !m_captureContinuously;

                if (m_captureContinuously)
                {
                    m_nextCaptureTime = Time.time;
                }
                else
                {
                    FlushMetadataToDisk();
                }

                Debug.Log(
                    $"PCA: Continuous capture is now {(m_captureContinuously ? "ON" : "OFF")}, " +
                    $"interval = {m_captureIntervalSeconds:F4}s");

                if (m_debugText != null)
                {
                    m_debugText.text =
                        $"Continuous {(m_captureContinuously ? "ON" : "OFF")} | dt={m_captureIntervalSeconds:F4}s";
                }
            }

            if (m_captureContinuously && Time.time >= m_nextCaptureTime)
            {
                CaptureStereoFrame();
                m_nextCaptureTime += m_captureIntervalSeconds;

                // 防止主线程卡顿后时间严重漂移，直接追到当前时间附近
                if (Time.time > m_nextCaptureTime + m_captureIntervalSeconds)
                {
                    m_nextCaptureTime = Time.time + m_captureIntervalSeconds;
                }
            }
        }

        private void OnApplicationQuit()
        {
            FlushMetadataToDisk();
        }

        private void OnDisable()
        {
            FlushMetadataToDisk();
        }

        private void CaptureStereoFrame()
        {
            if (!m_leftCameraAccess.IsUpdatedThisFrame || !m_rightCameraAccess.IsUpdatedThisFrame)
            {
                Debug.LogWarning("PCA: Left or right camera was not updated this frame. Skip.");
                if (m_debugText != null)
                {
                    m_debugText.text = "Skip: camera not updated";
                }
                return;
            }

            string frameId = m_captureIndex.ToString();

            try
            {
                var leftData = SaveSingleCameraFrameRaw(
                    cameraAccess: m_leftCameraAccess,
                    cameraName: "left",
                    imageFolder: m_leftImageFolder,
                    imageFileName: $"{frameId}.raw"
                );

                var rightData = SaveSingleCameraFrameRaw(
                    cameraAccess: m_rightCameraAccess,
                    cameraName: "right",
                    imageFolder: m_rightImageFolder,
                    imageFileName: $"{frameId}.raw"
                );

                m_leftSceneCameraDict[frameId] = new SceneCameraEntry
                {
                    cam_K = new[]
                    {
                        leftData.fx, 0.0f, leftData.cx,
                        0.0f, leftData.fy, leftData.cy,
                        0.0f, 0.0f, 1.0f
                    },
                    depth_scale = 1.0f,
                    timestep = leftData.timestep
                };

                m_rightSceneCameraDict[frameId] = new SceneCameraEntry
                {
                    cam_K = new[]
                    {
                        rightData.fx, 0.0f, rightData.cx,
                        0.0f, rightData.fy, rightData.cy,
                        0.0f, 0.0f, 1.0f
                    },
                    depth_scale = 1.0f,
                    timestep = rightData.timestep
                };

                m_hasUnflushedMetadata = true;

                if (m_debugText != null)
                {
                    m_debugText.text = $"Saved raw frame {frameId}";
                }

                m_captureIndex++;
            }
            catch (Exception e)
            {
                Debug.LogError($"PCA: Failed to capture stereo frame {frameId}. {e}");
                if (m_debugText != null)
                {
                    m_debugText.text = $"Capture failed: {frameId}";
                }
            }
        }

        // 下面是两种不同的保存方式，分别保存为灰度图和RGB图。根据需要选择其中一种，并注释掉另一种。
        // 保存灰度
        //private CameraFrameData SaveSingleCameraFrameRaw(
        //    PassthroughCameraAccess cameraAccess,
        //    string cameraName,
        //    string imageFolder,
        //    string imageFileName)
        //{
        //    var resolution = cameraAccess.CurrentResolution;
        //    var pixels = cameraAccess.GetColors();

        //    if (!pixels.IsCreated || pixels.Length == 0)
        //    {
        //        throw new InvalidOperationException($"PCA: {cameraName} GetColors() returned empty data.");
        //    }

        //    int width = resolution.x;
        //    int height = resolution.y;
        //    int expectedPixels = width * height;

        //    if (pixels.Length != expectedPixels)
        //    {
        //        Debug.LogWarning(
        //            $"PCA: {cameraName} pixel count mismatch. " +
        //            $"Expected {expectedPixels}, got {pixels.Length}.");
        //    }

        ////    byte[] grayBytes = ConvertColor32ToGray(pixels, width, height);
        //    byte[] grayBytes = ConvertColor32ToGrayFlipBoth(pixels, width, height);

        //    string imagePath = Path.Combine(imageFolder, imageFileName);
        //    File.WriteAllBytes(imagePath, grayBytes);

        //    var intrinsics = cameraAccess.Intrinsics;
        //    var timestamp = cameraAccess.Timestamp;

        //    return new CameraFrameData
        //    {
        //        width = width,
        //        height = height,
        //        fx = intrinsics.FocalLength.x,
        //        fy = intrinsics.FocalLength.y,
        //        cx = intrinsics.PrincipalPoint.x,
        //        cy = intrinsics.PrincipalPoint.y,
        //        timestep = ToUnixSeconds(timestamp)
        //    };
        //}

        //保存RGB
        private CameraFrameData SaveSingleCameraFrameRaw(
            PassthroughCameraAccess cameraAccess,
            string cameraName,
            string imageFolder,
            string imageFileName)
        {
            var resolution = cameraAccess.CurrentResolution;
            var pixels = cameraAccess.GetColors();

            if (!pixels.IsCreated || pixels.Length == 0)
            {
                throw new InvalidOperationException($"PCA: {cameraName} GetColors() returned empty data.");
            }

            int width = resolution.x;
            int height = resolution.y;
            int expectedPixels = width * height;

            if (pixels.Length != expectedPixels)
            {
                Debug.LogWarning(
                    $"PCA: {cameraName} pixel count mismatch. " +
                    $"Expected {expectedPixels}, got {pixels.Length}.");
            }

            //byte[] rgbBytes = ConvertColor32ToRgb(pixels, width, height);
            //byte[] rgbBytes = ConvertColor32ToRgbFlipBoth(pixels, width, height);
            byte[] rgbBytes = ConvertColor32ToRgbFlipVertical(pixels, width, height);

            string imagePath = Path.Combine(imageFolder, imageFileName);
            File.WriteAllBytes(imagePath, rgbBytes);

            var intrinsics = cameraAccess.Intrinsics;
            var timestamp = cameraAccess.Timestamp;

            return new CameraFrameData
            {
                width = width,
                height = height,
                fx = intrinsics.FocalLength.x,
                fy = intrinsics.FocalLength.y,
                cx = intrinsics.PrincipalPoint.x,
                cy = intrinsics.PrincipalPoint.y,
                timestep = ToUnixSeconds(timestamp)
            };
        }

        private byte[] ConvertColor32ToGray(NativeArray<Color32> pixels, int width, int height)
        {
            int pixelCount = width * height;
            byte[] gray = new byte[pixelCount];

            for (int i = 0; i < pixelCount; i++)
            {
                var c = pixels[i];
                gray[i] = (byte)((77 * c.r + 150 * c.g + 29 * c.b) >> 8);
            }

            return gray;
        }

        private byte[] ConvertColor32ToGrayFlipBoth(NativeArray<Color32> pixels, int width, int height)
        {
            int pixelCount = width * height;
            byte[] gray = new byte[pixelCount];

            for (int srcIdx = 0; srcIdx < pixelCount; srcIdx++)
            {
                int dstIdx = pixelCount - 1 - srcIdx;

                var c = pixels[srcIdx];
                gray[dstIdx] = (byte)((77 * c.r + 150 * c.g + 29 * c.b) >> 8);
            }

            return gray;
        }

        private byte[] ConvertColor32ToRgb(NativeArray<Color32> pixels, int width, int height)
        {
            int pixelCount = width * height;
            byte[] rgb = new byte[pixelCount * 3];

            for (int i = 0; i < pixelCount; i++)
            {
                var c = pixels[i];

                int baseIdx = i * 3;
                rgb[baseIdx] = c.r;
                rgb[baseIdx + 1] = c.g;
                rgb[baseIdx + 2] = c.b;
            }

            return rgb;
        }

        private byte[] ConvertColor32ToRgbFlipBoth(NativeArray<Color32> pixels, int width, int height)
        {
            int pixelCount = width * height;
            byte[] rgb = new byte[pixelCount * 3];

            for (int srcIdx = 0; srcIdx < pixelCount; srcIdx++)
            {
                // 上下翻转 + 左右镜像 等价于把一维像素索引反过来
                int dstIdx = pixelCount - 1 - srcIdx;

                var c = pixels[srcIdx];

                int dstBase = dstIdx * 3;
                rgb[dstBase] = c.r;
                rgb[dstBase + 1] = c.g;
                rgb[dstBase + 2] = c.b;
            }

            return rgb;
        }

        private byte[] ConvertColor32ToRgbFlipVertical(NativeArray<Color32> pixels, int width, int height)
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

                    var c = pixels[srcIdx];

                    int dstBase = dstIdx * 3;
                    rgb[dstBase] = c.r;
                    rgb[dstBase + 1] = c.g;
                    rgb[dstBase + 2] = c.b;
                }
            }

            return rgb;
        }

        private double ToUnixSeconds(DateTime timestamp)
        {
            DateTime utcTime = timestamp.Kind == DateTimeKind.Utc
                ? timestamp
                : timestamp.ToUniversalTime();

            DateTime epoch = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
            return (utcTime - epoch).TotalSeconds;
        }

        private void FlushMetadataToDisk()
        {
            if (!m_hasUnflushedMetadata)
            {
                return;
            }

            try
            {
                WriteSceneCameraJson(m_leftSceneCameraJsonPath, m_leftSceneCameraDict);
                WriteSceneCameraJson(m_rightSceneCameraJsonPath, m_rightSceneCameraDict);
                m_hasUnflushedMetadata = false;

                Debug.Log("PCA: Metadata flushed to disk.");
            }
            catch (Exception e)
            {
                Debug.LogError($"PCA: Failed to flush metadata. {e}");
            }
        }

        private void WriteSceneCameraJson(string jsonPath, Dictionary<string, SceneCameraEntry> sceneCameraDict)
        {
            var sb = new StringBuilder();
            sb.AppendLine("{");

            int index = 0;
            int count = sceneCameraDict.Count;

            foreach (var kv in sceneCameraDict)
            {
                string frameId = kv.Key;
                SceneCameraEntry entry = kv.Value;

                sb.Append($"  \"{frameId}\": ");
                sb.Append("{");
                sb.Append("\"cam_K\": [");
                sb.AppendFormat(
                    CultureInfo.InvariantCulture,
                    "{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}",
                    entry.cam_K[0], entry.cam_K[1], entry.cam_K[2],
                    entry.cam_K[3], entry.cam_K[4], entry.cam_K[5],
                    entry.cam_K[6], entry.cam_K[7], entry.cam_K[8]
                );
                sb.Append("], ");
                sb.AppendFormat(
                    CultureInfo.InvariantCulture,
                    "\"depth_scale\": {0}, ",
                    entry.depth_scale
                );
                sb.AppendFormat(
                    CultureInfo.InvariantCulture,
                    "\"timestep\": {0}",
                    entry.timestep
                );
                sb.Append("}");

                if (index < count - 1)
                {
                    sb.Append(",");
                }

                sb.AppendLine();
                index++;
            }

            sb.AppendLine("}");
            File.WriteAllText(jsonPath, sb.ToString());
        }

        [Serializable]
        private class SceneCameraEntry
        {
            public float[] cam_K;
            public float depth_scale;
            public double timestep;
        }

        private struct CameraFrameData
        {
            public int width;
            public int height;
            public float fx;
            public float fy;
            public float cx;
            public float cy;
            public double timestep;
        }
    }
}