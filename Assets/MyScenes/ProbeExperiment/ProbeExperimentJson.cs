using System;
using System.Globalization;
using System.Text;
using Meta.XR;
using UnityEngine;

namespace PassthroughCameraSamples.ProbeExperiment
{
    /// <summary>
    /// Writes compact JSON fragments for experiment frame logs without allocating full DTO objects.
    /// </summary>
    public static class ProbeExperimentJson
    {
        /// <summary>
        /// Serializes a pose into a JSON object containing position, quaternion, and matrix forms.
        /// </summary>
        public static string WritePoseMatrix(Pose pose)
        {
            var sb = new StringBuilder(256);
            WritePoseMatrix(sb, pose);
            return sb.ToString();
        }

        /// <summary>
        /// Appends a pose JSON object to an existing StringBuilder.
        /// </summary>
        public static void WritePoseMatrix(StringBuilder sb, Pose pose)
        {
            Matrix4x4 matrix = ProbeExperimentPoseMath.ToMatrix(pose);
            sb.Append("{\"position\":");
            WriteVector3(sb, pose.position);
            sb.Append(",\"rotation_xyzw\":");
            WriteQuaternion(sb, pose.rotation);
            sb.Append(",\"matrix4x4\":[");

            for (int row = 0; row < 4; row++)
            {
                for (int col = 0; col < 4; col++)
                {
                    if (row != 0 || col != 0)
                    {
                        sb.Append(',');
                    }

                    AppendFloat(sb, matrix[row, col]);
                }
            }

            sb.Append("]}");
        }

        /// <summary>
        /// Appends a Vector3 as a JSON array [x,y,z].
        /// </summary>
        public static void WriteVector3(StringBuilder sb, Vector3 value)
        {
            sb.Append('[');
            AppendFloat(sb, value.x);
            sb.Append(',');
            AppendFloat(sb, value.y);
            sb.Append(',');
            AppendFloat(sb, value.z);
            sb.Append(']');
        }

        /// <summary>
        /// Appends a Quaternion as a JSON array [x,y,z,w].
        /// </summary>
        public static void WriteQuaternion(StringBuilder sb, Quaternion value)
        {
            sb.Append('[');
            AppendFloat(sb, value.x);
            sb.Append(',');
            AppendFloat(sb, value.y);
            sb.Append(',');
            AppendFloat(sb, value.z);
            sb.Append(',');
            AppendFloat(sb, value.w);
            sb.Append(']');
        }

        /// <summary>
        /// Appends original camera intrinsics and image size.
        /// </summary>
        public static void WriteIntrinsics(StringBuilder sb, PassthroughCameraAccess.CameraIntrinsics intrinsics, Vector2Int resolution)
        {
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

            WriteIntrinsicsValues(
                sb,
                resolution,
                imageFx,
                imageFy,
                imageCx,
                imageCy,
                intrinsics.SensorResolution);
        }

        /// <summary>
        /// Appends intrinsics for a vertically flipped image while keeping poses in the original camera frame.
        /// </summary>
        public static void WriteVerticalFlipIntrinsics(StringBuilder sb, PassthroughCameraAccess.CameraIntrinsics intrinsics, Vector2Int resolution)
        {
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

            ProbeExperimentImageGeometry.ComputeVerticalFlipIntrinsics(
                imageFx,
                imageFy,
                imageCx,
                imageCy,
                resolution.y,
                out float flippedFx,
                out float flippedFy,
                out float flippedCx,
                out float flippedCy);

            WriteIntrinsicsValues(
                sb,
                resolution,
                flippedFx,
                flippedFy,
                flippedCx,
                flippedCy,
                intrinsics.SensorResolution);
        }

        /// <summary>
        /// Appends camera intrinsics from scalar values.
        /// </summary>
        private static void WriteIntrinsicsValues(
            StringBuilder sb,
            Vector2Int resolution,
            float fx,
            float fy,
            float cx,
            float cy,
            Vector2Int sensorResolution)
        {
            sb.Append("{\"width\":");
            sb.Append(resolution.x.ToString(CultureInfo.InvariantCulture));
            sb.Append(",\"height\":");
            sb.Append(resolution.y.ToString(CultureInfo.InvariantCulture));
            sb.Append(",\"fx\":");
            AppendFloat(sb, fx);
            sb.Append(",\"fy\":");
            AppendFloat(sb, fy);
            sb.Append(",\"cx\":");
            AppendFloat(sb, cx);
            sb.Append(",\"cy\":");
            AppendFloat(sb, cy);
            sb.Append(",\"sensor_width\":");
            sb.Append(sensorResolution.x.ToString(CultureInfo.InvariantCulture));
            sb.Append(",\"sensor_height\":");
            sb.Append(sensorResolution.y.ToString(CultureInfo.InvariantCulture));
            sb.Append('}');
        }

        /// <summary>
        /// Appends a JSON string with the small set of escape sequences needed by experiment logs.
        /// </summary>
        public static void WriteString(StringBuilder sb, string value)
        {
            sb.Append('"');
            foreach (char c in value ?? string.Empty)
            {
                switch (c)
                {
                    case '\\':
                        sb.Append("\\\\");
                        break;
                    case '"':
                        sb.Append("\\\"");
                        break;
                    case '\n':
                        sb.Append("\\n");
                        break;
                    case '\r':
                        sb.Append("\\r");
                        break;
                    case '\t':
                        sb.Append("\\t");
                        break;
                    default:
                        sb.Append(c);
                        break;
                }
            }
            sb.Append('"');
        }

        /// <summary>
        /// Converts a DateTime timestamp to Unix seconds in UTC.
        /// </summary>
        public static double ToUnixSeconds(DateTime timestamp)
        {
            DateTime utcTime = timestamp.Kind == DateTimeKind.Utc
                ? timestamp
                : timestamp.ToUniversalTime();

            return (utcTime - DateTime.UnixEpoch).TotalSeconds;
        }

        /// <summary>
        /// Appends a float using invariant culture and round-trip formatting.
        /// </summary>
        public static void AppendFloat(StringBuilder sb, float value)
        {
            sb.Append(value.ToString("R", CultureInfo.InvariantCulture));
        }

        /// <summary>
        /// Appends a double using invariant culture and round-trip formatting.
        /// </summary>
        public static void AppendDouble(StringBuilder sb, double value)
        {
            sb.Append(value.ToString("R", CultureInfo.InvariantCulture));
        }
    }
}
