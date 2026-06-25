using UnityEngine;

namespace PassthroughCameraSamples.ProbeExperiment
{
    /// <summary>
    /// Provides small rigid-transform helpers used by the probe experiment.
    /// Pose names follow the convention targetFromSource, for example worldFromCamera means T_W_C.
    /// </summary>
    public static class ProbeExperimentPoseMath
    {
        /// <summary>
        /// Composes two poses: parentFromChild * childFromGrandchild = parentFromGrandchild.
        /// </summary>
        public static Pose Compose(Pose parentFromChild, Pose childFromGrandchild)
        {
            return new Pose(
                parentFromChild.position + parentFromChild.rotation * childFromGrandchild.position,
                parentFromChild.rotation * childFromGrandchild.rotation);
        }

        /// <summary>
        /// Returns the inverse rigid transform for the supplied pose.
        /// </summary>
        public static Pose Inverse(Pose pose)
        {
            Quaternion inverseRotation = Quaternion.Inverse(pose.rotation);
            return new Pose(inverseRotation * -pose.position, inverseRotation);
        }

        /// <summary>
        /// Computes T_W_B from T_W_M and the measured marker-to-robot-base transform T_M_B.
        /// </summary>
        public static Pose ComputeWorldFromBase(Pose worldFromMarker, Pose markerFromBase)
        {
            return Compose(worldFromMarker, markerFromBase);
        }

        /// <summary>
        /// Computes probe pose in the passthrough camera frame: T_C_P = inverse(T_W_C) * T_W_B * T_B_P.
        /// </summary>
        public static Pose ComputeCameraFromProbe(Pose worldFromCamera, Pose worldFromBase, Pose baseFromProbe)
        {
            Pose worldFromProbe = Compose(worldFromBase, baseFromProbe);
            return Compose(Inverse(worldFromCamera), worldFromProbe);
        }

        /// <summary>
        /// Converts a Unity Pose into a homogeneous 4x4 matrix with unit scale.
        /// </summary>
        public static Matrix4x4 ToMatrix(Pose pose)
        {
            return Matrix4x4.TRS(pose.position, pose.rotation, Vector3.one);
        }
    }
}
