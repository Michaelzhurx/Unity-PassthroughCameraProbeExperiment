namespace PassthroughCameraSamples.ProbeExperiment
{
    /// <summary>
    /// Contains image-coordinate helpers shared by capture logging and fiducial detection.
    /// </summary>
    public static class ProbeExperimentImageGeometry
    {
        /// <summary>
        /// Computes intrinsics for the current GetColors image resolution from Meta sensor intrinsics.
        /// </summary>
        public static void ComputeCurrentResolutionIntrinsics(
            float sensorFx,
            float sensorFy,
            float sensorCx,
            float sensorCy,
            int sensorWidth,
            int sensorHeight,
            int imageWidth,
            int imageHeight,
            out float imageFx,
            out float imageFy,
            out float imageCx,
            out float imageCy)
        {
            if (sensorWidth <= 0 || sensorHeight <= 0 || imageWidth <= 0 || imageHeight <= 0)
            {
                imageFx = sensorFx;
                imageFy = sensorFy;
                imageCx = sensorCx;
                imageCy = sensorCy;
                return;
            }

            float scaleX = (float)imageWidth / sensorWidth;
            float scaleY = (float)imageHeight / sensorHeight;
            float cropScale = scaleX > scaleY ? scaleX : scaleY;
            float cropWidth = imageWidth / cropScale;
            float cropHeight = imageHeight / cropScale;
            float cropX = sensorWidth * 0.5f - cropWidth * 0.5f;
            float cropY = sensorHeight * 0.5f - cropHeight * 0.5f;

            imageFx = sensorFx * imageWidth / cropWidth;
            imageFy = sensorFy * imageHeight / cropHeight;
            imageCx = (sensorCx - cropX) * imageWidth / cropWidth;
            imageCy = (sensorCy - cropY) * imageHeight / cropHeight;
        }

        /// <summary>
        /// Computes intrinsics for a vertically flipped image while keeping poses in the original camera frame.
        /// </summary>
        public static void ComputeVerticalFlipIntrinsics(
            float fx,
            float fy,
            float cx,
            float cy,
            int height,
            out float flippedFx,
            out float flippedFy,
            out float flippedCx,
            out float flippedCy)
        {
            flippedFx = fx;
            flippedFy = -fy;
            flippedCx = cx;
            flippedCy = height - 1 - cy;
        }
    }
}
