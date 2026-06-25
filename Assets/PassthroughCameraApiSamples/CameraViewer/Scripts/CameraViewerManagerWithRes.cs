using System.Collections;
using Meta.XR;
using Meta.XR.Samples;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.UI;

namespace PassthroughCameraSamples.CameraViewer
{
    [MetaCodeSample("PassthroughCameraApiSamples-CameraViewer")]
    public class CameraViewerManagerWithRes : MonoBehaviour
    {
        [SerializeField] private PassthroughCameraAccess m_cameraAccess;
        [SerializeField] private Text m_debugText;
        [SerializeField] private RawImage m_image;

        private string m_supportedResString;

        private IEnumerator Start()
        {
            var supportedResolutions = PassthroughCameraAccess.GetSupportedResolutions(
                PassthroughCameraAccess.CameraPositionType.Left);

            Assert.IsNotNull(supportedResolutions, nameof(supportedResolutions));

            // ¸ñÊ½»¯×Ö·û´®
            m_supportedResString = "Supported:";
            foreach (var res in supportedResolutions)
            {
                m_supportedResString += $"{res.x} x {res.y}\n";
            }

            while (!m_cameraAccess.IsPlaying)
            {
                yield return null;
            }

            m_image.texture = m_cameraAccess.GetTexture();
        }

        private void Update()
        {
            bool granted = OVRPermissionsRequester.IsPermissionGranted(
                OVRPermissionsRequester.Permission.PassthroughCameraAccess);

            string permissionStr = granted ? "Permission granted" : "No permission";

            if (m_cameraAccess.IsPlaying && m_cameraAccess.GetTexture() != null)
            {
                var tex = m_cameraAccess.GetTexture();
                string currentRes = $"Current: {tex.width} x {tex.height}";

                m_debugText.text = $"{permissionStr}\n{m_supportedResString}\n{currentRes}";
            }
            else
            {
                m_debugText.text = $"{permissionStr}\n\nWaiting for camera...";
            }
        }
    }
}
