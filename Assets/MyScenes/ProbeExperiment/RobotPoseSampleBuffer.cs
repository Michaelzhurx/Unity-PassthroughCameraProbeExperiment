using System.Collections.Generic;
using UnityEngine;

namespace PassthroughCameraSamples.ProbeExperiment
{
    /// <summary>
    /// Timestamped robot pose sample normalized into the experiment robot base frame.
    /// </summary>
    public struct RobotPoseSample
    {
        public int sequence;
        public bool poseValid;
        public Pose baseFromProbe;
        public double robotTimestampSeconds;
        public double samplePcTimestampUnixSeconds;
        public double pcReceiveUnixSeconds;
        public double pcSendUnixSeconds;
    }

    /// <summary>
    /// Maintains an ordered robot pose ring buffer and samples it at camera timestamps.
    /// </summary>
    public sealed class RobotPoseSampleBuffer
    {
        private readonly List<RobotPoseSample> m_samples = new();
        private readonly int m_capacity;
        private readonly double m_bufferDurationSeconds;

        public RobotPoseSampleBuffer(int capacity, double bufferDurationSeconds)
        {
            m_capacity = capacity < 2 ? 2 : capacity;
            m_bufferDurationSeconds = bufferDurationSeconds <= 0.0 ? 10.0 : bufferDurationSeconds;
        }

        public int Count => m_samples.Count;

        /// <summary>
        /// Removes all cached robot pose samples.
        /// </summary>
        public void Clear()
        {
            m_samples.Clear();
        }

        /// <summary>
        /// Adds a sample while keeping the buffer ordered by PC-domain sample time.
        /// </summary>
        public void Add(RobotPoseSample sample)
        {
            if (m_samples.Count > 0)
            {
                RobotPoseSample last = m_samples[m_samples.Count - 1];
                if (sample.sequence <= last.sequence && sample.samplePcTimestampUnixSeconds <= last.samplePcTimestampUnixSeconds)
                {
                    return;
                }
            }

            int insertIndex = m_samples.Count;
            while (insertIndex > 0 && m_samples[insertIndex - 1].samplePcTimestampUnixSeconds > sample.samplePcTimestampUnixSeconds)
            {
                insertIndex--;
            }

            if (insertIndex > 0 && m_samples[insertIndex - 1].sequence == sample.sequence)
            {
                return;
            }

            if (insertIndex < m_samples.Count && m_samples[insertIndex].sequence == sample.sequence)
            {
                return;
            }

            m_samples.Insert(insertIndex, sample);
            Trim(sample.samplePcTimestampUnixSeconds);
        }

        /// <summary>
        /// Samples T_B_P at a PC-domain timestamp using interpolation only.
        /// </summary>
        public bool TrySampleAt(
            double targetPcTimestampUnixSeconds,
            double requestedQuestTimestampUnixSeconds,
            double maxInterpolationSpanSeconds,
            double pcQuestOffsetSeconds,
            double pcQuestRttSeconds,
            out Pose baseFromProbe,
            out RobotPoseTimingInfo timingInfo)
        {
            baseFromProbe = default;
            timingInfo = CreateInvalidTiming(
                "buffer_empty",
                requestedQuestTimestampUnixSeconds,
                targetPcTimestampUnixSeconds,
                pcQuestOffsetSeconds,
                pcQuestRttSeconds);

            if (m_samples.Count == 0)
            {
                return false;
            }

            RobotPoseSample first = m_samples[0];
            RobotPoseSample last = m_samples[m_samples.Count - 1];

            if (m_samples.Count == 1 && System.Math.Abs(targetPcTimestampUnixSeconds - first.samplePcTimestampUnixSeconds) <= double.Epsilon)
            {
                if (!first.poseValid)
                {
                    timingInfo = CreateInvalidTiming(
                        "sample_invalid",
                        requestedQuestTimestampUnixSeconds,
                        targetPcTimestampUnixSeconds,
                        pcQuestOffsetSeconds,
                        pcQuestRttSeconds);
                    FillBoundaryTiming(ref timingInfo, first, first);
                    return false;
                }

                baseFromProbe = first.baseFromProbe;
                timingInfo = new RobotPoseTimingInfo
                {
                    valid = true,
                    mode = "exact",
                    invalidReason = string.Empty,
                    requestedTimestampUnixSeconds = requestedQuestTimestampUnixSeconds,
                    robotTimestampSeconds = first.robotTimestampSeconds,
                    pcTimestampUnixSeconds = targetPcTimestampUnixSeconds,
                    beforeTimestampUnixSeconds = first.samplePcTimestampUnixSeconds,
                    afterTimestampUnixSeconds = first.samplePcTimestampUnixSeconds,
                    timeDeltaSeconds = 0.0,
                    interpolationSpanSeconds = 0.0,
                    pcQuestOffsetSeconds = pcQuestOffsetSeconds,
                    pcQuestRttSeconds = pcQuestRttSeconds,
                    sequence = first.sequence
                };
                return true;
            }

            if (targetPcTimestampUnixSeconds < first.samplePcTimestampUnixSeconds)
            {
                timingInfo = CreateInvalidTiming(
                    "too_old",
                    requestedQuestTimestampUnixSeconds,
                    targetPcTimestampUnixSeconds,
                    pcQuestOffsetSeconds,
                    pcQuestRttSeconds);
                FillBoundaryTiming(ref timingInfo, first, first);
                return false;
            }

            if (targetPcTimestampUnixSeconds > last.samplePcTimestampUnixSeconds)
            {
                timingInfo = CreateInvalidTiming(
                    "too_new",
                    requestedQuestTimestampUnixSeconds,
                    targetPcTimestampUnixSeconds,
                    pcQuestOffsetSeconds,
                    pcQuestRttSeconds);
                FillBoundaryTiming(ref timingInfo, last, last);
                return false;
            }

            for (int i = 0; i < m_samples.Count - 1; i++)
            {
                RobotPoseSample before = m_samples[i];
                RobotPoseSample after = m_samples[i + 1];
                if (targetPcTimestampUnixSeconds < before.samplePcTimestampUnixSeconds
                    || targetPcTimestampUnixSeconds > after.samplePcTimestampUnixSeconds)
                {
                    continue;
                }

                double span = after.samplePcTimestampUnixSeconds - before.samplePcTimestampUnixSeconds;
                if (span < 0.0)
                {
                    continue;
                }

                if (!before.poseValid || !after.poseValid)
                {
                    timingInfo = CreateInvalidTiming(
                        "sample_invalid",
                        requestedQuestTimestampUnixSeconds,
                        targetPcTimestampUnixSeconds,
                        pcQuestOffsetSeconds,
                        pcQuestRttSeconds);
                    FillBoundaryTiming(ref timingInfo, before, after);
                    return false;
                }

                if (span > maxInterpolationSpanSeconds)
                {
                    timingInfo = CreateInvalidTiming(
                        "gap_too_large",
                        requestedQuestTimestampUnixSeconds,
                        targetPcTimestampUnixSeconds,
                        pcQuestOffsetSeconds,
                        pcQuestRttSeconds);
                    FillBoundaryTiming(ref timingInfo, before, after);
                    return false;
                }

                float t = span <= double.Epsilon
                    ? 0f
                    : Mathf.Clamp01((float)((targetPcTimestampUnixSeconds - before.samplePcTimestampUnixSeconds) / span));
                baseFromProbe = new Pose(
                    Vector3.Lerp(before.baseFromProbe.position, after.baseFromProbe.position, t),
                    Quaternion.Slerp(before.baseFromProbe.rotation, after.baseFromProbe.rotation, t));

                timingInfo = new RobotPoseTimingInfo
                {
                    valid = true,
                    mode = span <= double.Epsilon ? "exact" : "interpolated",
                    invalidReason = string.Empty,
                    requestedTimestampUnixSeconds = requestedQuestTimestampUnixSeconds,
                    robotTimestampSeconds = LerpDouble(before.robotTimestampSeconds, after.robotTimestampSeconds, t),
                    pcTimestampUnixSeconds = targetPcTimestampUnixSeconds,
                    beforeTimestampUnixSeconds = before.samplePcTimestampUnixSeconds,
                    afterTimestampUnixSeconds = after.samplePcTimestampUnixSeconds,
                    timeDeltaSeconds = System.Math.Min(
                        System.Math.Abs(targetPcTimestampUnixSeconds - before.samplePcTimestampUnixSeconds),
                        System.Math.Abs(after.samplePcTimestampUnixSeconds - targetPcTimestampUnixSeconds)),
                    interpolationSpanSeconds = span,
                    pcQuestOffsetSeconds = pcQuestOffsetSeconds,
                    pcQuestRttSeconds = pcQuestRttSeconds,
                    sequence = t < 0.5f ? before.sequence : after.sequence
                };
                return true;
            }

            timingInfo = CreateInvalidTiming(
                "no_bracket",
                requestedQuestTimestampUnixSeconds,
                targetPcTimestampUnixSeconds,
                pcQuestOffsetSeconds,
                pcQuestRttSeconds);
            FillBoundaryTiming(ref timingInfo, first, last);
            return false;
        }

        private void Trim(double newestTimestamp)
        {
            while (m_samples.Count > m_capacity)
            {
                m_samples.RemoveAt(0);
            }

            double oldestAllowed = newestTimestamp - m_bufferDurationSeconds;
            while (m_samples.Count > 2 && m_samples[0].samplePcTimestampUnixSeconds < oldestAllowed)
            {
                m_samples.RemoveAt(0);
            }
        }

        private static RobotPoseTimingInfo CreateInvalidTiming(
            string reason,
            double requestedQuestTimestampUnixSeconds,
            double targetPcTimestampUnixSeconds,
            double pcQuestOffsetSeconds,
            double pcQuestRttSeconds)
        {
            return new RobotPoseTimingInfo
            {
                valid = false,
                mode = "invalid",
                invalidReason = reason,
                requestedTimestampUnixSeconds = requestedQuestTimestampUnixSeconds,
                robotTimestampSeconds = 0.0,
                pcTimestampUnixSeconds = targetPcTimestampUnixSeconds,
                beforeTimestampUnixSeconds = 0.0,
                afterTimestampUnixSeconds = 0.0,
                timeDeltaSeconds = 0.0,
                interpolationSpanSeconds = 0.0,
                pcQuestOffsetSeconds = pcQuestOffsetSeconds,
                pcQuestRttSeconds = pcQuestRttSeconds,
                sequence = 0
            };
        }

        private static void FillBoundaryTiming(ref RobotPoseTimingInfo timingInfo, RobotPoseSample before, RobotPoseSample after)
        {
            timingInfo.robotTimestampSeconds = before.robotTimestampSeconds;
            timingInfo.beforeTimestampUnixSeconds = before.samplePcTimestampUnixSeconds;
            timingInfo.afterTimestampUnixSeconds = after.samplePcTimestampUnixSeconds;
            timingInfo.interpolationSpanSeconds = after.samplePcTimestampUnixSeconds - before.samplePcTimestampUnixSeconds;
            timingInfo.sequence = before.sequence;
        }

        private static double LerpDouble(double a, double b, float t)
        {
            return a + (b - a) * t;
        }
    }
}
