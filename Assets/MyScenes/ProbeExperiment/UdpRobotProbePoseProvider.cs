using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

namespace PassthroughCameraSamples.ProbeExperiment
{
    /// <summary>
    /// Receives timestamped robot probe poses from a PC relay over UDP and samples them at camera timestamps.
    /// </summary>
    public class UdpRobotProbePoseProvider : RobotProbePoseProvider
    {
        [SerializeField] private int m_listenPort = 15000;
        [SerializeField] private string m_pcHost = "127.0.0.1";
        [SerializeField] private int m_pcSyncPort = 15001;
        [SerializeField] private float m_syncIntervalSeconds = 1.0f;
        [SerializeField] private float m_maxAcceptedSyncRttSeconds = 0.02f;
        [SerializeField] private int m_syncSampleWindow = 8;
        [SerializeField] private int m_poseBufferCapacity = 512;
        [SerializeField] private float m_poseBufferDurationSeconds = 10.0f;
        [SerializeField] private float m_maxInterpolationSpanSeconds = 0.1f;
        [SerializeField] private bool m_startOnEnable = true;
        [SerializeField] private bool m_logPacketErrors;

        private readonly object m_lock = new();
        private readonly List<TimeSyncObservation> m_syncObservations = new();
        private RobotPoseSampleBuffer m_poseBuffer;
        private UdpClient m_client;
        private IPEndPoint m_anyEndpoint;
        private int m_syncSequence;
        private double m_nextSyncTime;
        private double m_pcQuestOffsetSeconds;
        private double m_pcQuestRttSeconds;
        private bool m_hasTimeSync;
        private bool m_isListening;
        private string m_status = "UDP robot pose provider is not listening.";

        public string Status => m_status;
        public bool HasTimeSync => m_hasTimeSync;
        public double PcQuestOffsetSeconds => m_pcQuestOffsetSeconds;
        public double PcQuestRttSeconds => m_pcQuestRttSeconds;

        private void OnEnable()
        {
            EnsureBuffer();
            if (m_startOnEnable)
            {
                StartListening();
            }
        }

        private void Update()
        {
            if (!m_isListening || string.IsNullOrWhiteSpace(m_pcHost) || m_pcSyncPort <= 0)
            {
                return;
            }

            double now = Time.realtimeSinceStartupAsDouble;
            if (now < m_nextSyncTime)
            {
                return;
            }

            m_nextSyncTime = now + m_syncIntervalSeconds;
            SendTimeSyncRequest();
        }

        private void OnDisable()
        {
            StopListening();
        }

        /// <summary>
        /// Tries to sample the robot probe pose at the supplied camera Unix timestamp.
        /// </summary>
        public override bool TryGetBaseFromProbeAt(
            double timestampUnixSeconds,
            out Pose baseFromProbe,
            out RobotPoseTimingInfo timingInfo)
        {
            baseFromProbe = default;
            EnsureBuffer();

            lock (m_lock)
            {
                if (!m_hasTimeSync)
                {
                    timingInfo = CreateSyncInvalidTiming(timestampUnixSeconds);
                    return false;
                }

                double targetPcTimestamp = timestampUnixSeconds + m_pcQuestOffsetSeconds;
                return m_poseBuffer.TrySampleAt(
                    targetPcTimestamp,
                    timestampUnixSeconds,
                    m_maxInterpolationSpanSeconds,
                    m_pcQuestOffsetSeconds,
                    m_pcQuestRttSeconds,
                    out baseFromProbe,
                    out timingInfo);
            }
        }

        /// <summary>
        /// Opens the UDP socket and begins receiving robot pose packets.
        /// </summary>
        public void StartListening()
        {
            if (m_isListening)
            {
                return;
            }

            try
            {
                m_anyEndpoint = new IPEndPoint(IPAddress.Any, 0);
                m_client = new UdpClient(m_listenPort);
                m_isListening = true;
                m_status = $"Listening for robot poses on UDP {m_listenPort}.";
                BeginReceive();
            }
            catch (Exception e)
            {
                m_isListening = false;
                m_status = $"Failed to open UDP {m_listenPort}: {e.Message}";
                Debug.LogError($"UdpRobotProbePoseProvider: {m_status}", this);
            }
        }

        /// <summary>
        /// Closes the UDP socket and clears transient sync state.
        /// </summary>
        public void StopListening()
        {
            m_isListening = false;
            if (m_client != null)
            {
                m_client.Close();
                m_client = null;
            }

            lock (m_lock)
            {
                m_hasTimeSync = false;
                m_syncObservations.Clear();
            }

            m_status = "UDP robot pose provider is not listening.";
        }

        private void EnsureBuffer()
        {
            if (m_poseBuffer == null)
            {
                m_poseBuffer = new RobotPoseSampleBuffer(m_poseBufferCapacity, m_poseBufferDurationSeconds);
            }
        }

        private void BeginReceive()
        {
            if (!m_isListening || m_client == null)
            {
                return;
            }

            try
            {
                m_client.BeginReceive(OnUdpReceived, null);
            }
            catch (ObjectDisposedException)
            {
            }
            catch (Exception e)
            {
                m_status = $"UDP receive failed: {e.Message}";
                if (m_logPacketErrors)
                {
                    Debug.LogWarning($"UdpRobotProbePoseProvider: {m_status}", this);
                }
            }
        }

        private void OnUdpReceived(IAsyncResult result)
        {
            IPEndPoint remote = m_anyEndpoint;
            byte[] data = null;

            try
            {
                if (m_client == null)
                {
                    return;
                }

                data = m_client.EndReceive(result, ref remote);
            }
            catch (ObjectDisposedException)
            {
                return;
            }
            catch (Exception e)
            {
                if (m_logPacketErrors)
                {
                    Debug.LogWarning($"UdpRobotProbePoseProvider: UDP receive callback failed: {e.Message}", this);
                }
            }
            finally
            {
                BeginReceive();
            }

            if (data == null || data.Length == 0)
            {
                return;
            }

            string json = Encoding.UTF8.GetString(data);
            ProcessPacket(json);
        }

        private void ProcessPacket(string json)
        {
            try
            {
                PacketHeader header = JsonUtility.FromJson<PacketHeader>(json);
                if (header == null || string.IsNullOrEmpty(header.type))
                {
                    return;
                }

                if (header.type == "robot_pose")
                {
                    ProcessRobotPose(JsonUtility.FromJson<RobotPosePacket>(json));
                }
                else if (header.type == "time_sync_resp")
                {
                    ProcessTimeSyncResponse(JsonUtility.FromJson<TimeSyncResponsePacket>(json));
                }
            }
            catch (Exception e)
            {
                if (m_logPacketErrors)
                {
                    Debug.LogWarning($"UdpRobotProbePoseProvider: Failed to parse UDP packet: {e.Message}", this);
                }
            }
        }

        private void ProcessRobotPose(RobotPosePacket packet)
        {
            if (packet == null || packet.position == null || packet.rotation_xyzw == null)
            {
                return;
            }

            if (packet.position.Length < 3 || packet.rotation_xyzw.Length < 4)
            {
                return;
            }

            double samplePcTimestamp = packet.sample_pc_unix_s > 0.0
                ? packet.sample_pc_unix_s
                : packet.pc_receive_unix_s > 0.0
                    ? packet.pc_receive_unix_s
                    : packet.pc_send_unix_s;

            if (samplePcTimestamp <= 0.0)
            {
                return;
            }

            Quaternion rotation = new Quaternion(
                packet.rotation_xyzw[0],
                packet.rotation_xyzw[1],
                packet.rotation_xyzw[2],
                packet.rotation_xyzw[3]);
            if (rotation.x * rotation.x + rotation.y * rotation.y + rotation.z * rotation.z + rotation.w * rotation.w < 0.000001f)
            {
                rotation = Quaternion.identity;
            }
            else
            {
                rotation.Normalize();
            }

            var sample = new RobotPoseSample
            {
                sequence = packet.seq,
                poseValid = packet.pose_valid,
                baseFromProbe = new Pose(
                    new Vector3(packet.position[0], packet.position[1], packet.position[2]),
                    rotation),
                robotTimestampSeconds = packet.robot_time_s,
                samplePcTimestampUnixSeconds = samplePcTimestamp,
                pcReceiveUnixSeconds = packet.pc_receive_unix_s,
                pcSendUnixSeconds = packet.pc_send_unix_s
            };

            lock (m_lock)
            {
                m_poseBuffer.Add(sample);
            }

            m_status = $"Received robot pose seq {packet.seq}.";
        }

        private void SendTimeSyncRequest()
        {
            if (m_client == null)
            {
                return;
            }

            var packet = new TimeSyncRequestPacket
            {
                type = "time_sync_req",
                seq = ++m_syncSequence,
                quest_send_unix_s = UnixNowSeconds()
            };

            string json = JsonUtility.ToJson(packet);
            byte[] bytes = Encoding.UTF8.GetBytes(json);

            try
            {
                m_client.Send(bytes, bytes.Length, m_pcHost, m_pcSyncPort);
            }
            catch (Exception e)
            {
                m_status = $"Failed to send time sync request: {e.Message}";
                if (m_logPacketErrors)
                {
                    Debug.LogWarning($"UdpRobotProbePoseProvider: {m_status}", this);
                }
            }
        }

        private void ProcessTimeSyncResponse(TimeSyncResponsePacket packet)
        {
            if (packet == null || packet.quest_send_unix_s <= 0.0 || packet.pc_receive_unix_s <= 0.0 || packet.pc_send_unix_s <= 0.0)
            {
                return;
            }

            double questReceiveUnixSeconds = UnixNowSeconds();
            double rtt = questReceiveUnixSeconds - packet.quest_send_unix_s
                - (packet.pc_send_unix_s - packet.pc_receive_unix_s);
            if (rtt < 0.0)
            {
                rtt = 0.0;
            }

            double offset = ((packet.pc_receive_unix_s - packet.quest_send_unix_s)
                + (packet.pc_send_unix_s - questReceiveUnixSeconds)) * 0.5;

            if (rtt > m_maxAcceptedSyncRttSeconds)
            {
                m_status = $"Ignored high RTT time sync sample {rtt:F4}s.";
                return;
            }

            lock (m_lock)
            {
                m_syncObservations.Add(new TimeSyncObservation { offsetSeconds = offset, rttSeconds = rtt });
                m_syncObservations.Sort((a, b) => a.rttSeconds.CompareTo(b.rttSeconds));

                while (m_syncObservations.Count > m_syncSampleWindow)
                {
                    m_syncObservations.RemoveAt(m_syncObservations.Count - 1);
                }

                double offsetSum = 0.0;
                double rttSum = 0.0;
                foreach (TimeSyncObservation observation in m_syncObservations)
                {
                    offsetSum += observation.offsetSeconds;
                    rttSum += observation.rttSeconds;
                }

                m_pcQuestOffsetSeconds = offsetSum / m_syncObservations.Count;
                m_pcQuestRttSeconds = rttSum / m_syncObservations.Count;
                m_hasTimeSync = true;
            }

            m_status = $"Time sync offset {m_pcQuestOffsetSeconds:F6}s, RTT {m_pcQuestRttSeconds:F4}s.";
        }

        private RobotPoseTimingInfo CreateSyncInvalidTiming(double requestedTimestampUnixSeconds)
        {
            return new RobotPoseTimingInfo
            {
                valid = false,
                mode = "invalid",
                invalidReason = "sync_unready",
                requestedTimestampUnixSeconds = requestedTimestampUnixSeconds,
                robotTimestampSeconds = 0.0,
                pcTimestampUnixSeconds = requestedTimestampUnixSeconds,
                beforeTimestampUnixSeconds = 0.0,
                afterTimestampUnixSeconds = 0.0,
                timeDeltaSeconds = 0.0,
                interpolationSpanSeconds = 0.0,
                pcQuestOffsetSeconds = m_pcQuestOffsetSeconds,
                pcQuestRttSeconds = m_pcQuestRttSeconds,
                sequence = 0
            };
        }

        private static double UnixNowSeconds()
        {
            return DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;
        }

#pragma warning disable 0649
        [Serializable]
        private class PacketHeader
        {
            public string type;
        }

        [Serializable]
        private class RobotPosePacket
        {
            public string type;
            public int seq;
            public double robot_time_s;
            public double sample_pc_unix_s;
            public double pc_receive_unix_s;
            public double pc_send_unix_s;
            public float[] position;
            public float[] rotation_xyzw;
            public bool pose_valid;
        }

        [Serializable]
        private class TimeSyncRequestPacket
        {
            public string type;
            public int seq;
            public double quest_send_unix_s;
        }

        [Serializable]
        private class TimeSyncResponsePacket
        {
            public string type;
            public int seq;
            public double quest_send_unix_s;
            public double pc_receive_unix_s;
            public double pc_send_unix_s;
        }
#pragma warning restore 0649

        private struct TimeSyncObservation
        {
            public double offsetSeconds;
            public double rttSeconds;
        }
    }
}
