using NUnit.Framework;
using UnityEngine;

namespace PassthroughCameraSamples.ProbeExperiment.Tests
{
    public class RobotPoseSampleBufferTests
    {
        [Test]
        public void TrySampleAtCarriesNormalizedDataFromNearestSample()
        {
            var buffer = new RobotPoseSampleBuffer(8, 5.0);
            buffer.Add(new RobotPoseSample
            {
                sequence = 1,
                poseValid = true,
                baseFromProbe = new Pose(Vector3.zero, Quaternion.identity),
                robotTimestampSeconds = 10.0,
                samplePcTimestampUnixSeconds = 100.0,
                pcReceiveUnixSeconds = 100.0,
                pcSendUnixSeconds = 100.0,
                poseRaw = new[] { 1f, 2f, 3f, 0.1f, 0.2f, 0.3f },
                normalizedData = new[] { 0.1f, 0.2f, 0.3f }
            });
            buffer.Add(new RobotPoseSample
            {
                sequence = 2,
                poseValid = true,
                baseFromProbe = new Pose(new Vector3(1f, 0f, 0f), Quaternion.identity),
                robotTimestampSeconds = 10.1,
                samplePcTimestampUnixSeconds = 100.1,
                pcReceiveUnixSeconds = 100.1,
                pcSendUnixSeconds = 100.1,
                poseRaw = new[] { 4f, 5f, 6f, 0.4f, 0.5f, 0.6f },
                normalizedData = new[] { 0.4f, 0.5f, 0.6f }
            });

            bool sampled = buffer.TrySampleAt(
                100.06,
                99.96,
                0.1,
                0.1,
                0.002,
                out _,
                out RobotPoseTimingInfo timingInfo);

            Assert.IsTrue(sampled);
            CollectionAssert.AreEqual(new[] { 4f, 5f, 6f, 0.4f, 0.5f, 0.6f }, timingInfo.poseRaw);
            CollectionAssert.AreEqual(new[] { 0.4f, 0.5f, 0.6f }, timingInfo.normalizedData);
        }
    }
}
