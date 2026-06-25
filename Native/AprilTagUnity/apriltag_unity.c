#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "apriltag.h"
#include "apriltag_pose.h"
#include "common/image_u8.h"
#include "common/matd.h"
#include "common/zarray.h"
#include "tag36h11.h"

#if defined(_WIN32)
#define APRILTAG_UNITY_EXPORT __declspec(dllexport)
#else
#define APRILTAG_UNITY_EXPORT __attribute__((visibility("default")))
#endif

typedef struct AprilTagUnityDetection
{
    int id;
    float px;
    float py;
    float pz;
    float qx;
    float qy;
    float qz;
    float qw;
    float reprojectionErrorPixels;
    float corner0x;
    float corner0y;
    float corner1x;
    float corner1y;
    float corner2x;
    float corner2y;
    float corner3x;
    float corner3y;
} AprilTagUnityDetection;

static void rotation_matrix_to_quaternion(const matd_t *r, float *qx, float *qy, float *qz, float *qw)
{
    const double m00 = MATD_EL(r, 0, 0);
    const double m01 = MATD_EL(r, 0, 1);
    const double m02 = MATD_EL(r, 0, 2);
    const double m10 = MATD_EL(r, 1, 0);
    const double m11 = MATD_EL(r, 1, 1);
    const double m12 = MATD_EL(r, 1, 2);
    const double m20 = MATD_EL(r, 2, 0);
    const double m21 = MATD_EL(r, 2, 1);
    const double m22 = MATD_EL(r, 2, 2);
    const double trace = m00 + m11 + m22;

    double x;
    double y;
    double z;
    double w;

    if (trace > 0.0)
    {
        const double s = sqrt(trace + 1.0) * 2.0;
        w = 0.25 * s;
        x = (m21 - m12) / s;
        y = (m02 - m20) / s;
        z = (m10 - m01) / s;
    }
    else if (m00 > m11 && m00 > m22)
    {
        const double s = sqrt(1.0 + m00 - m11 - m22) * 2.0;
        w = (m21 - m12) / s;
        x = 0.25 * s;
        y = (m01 + m10) / s;
        z = (m02 + m20) / s;
    }
    else if (m11 > m22)
    {
        const double s = sqrt(1.0 + m11 - m00 - m22) * 2.0;
        w = (m02 - m20) / s;
        x = (m01 + m10) / s;
        y = 0.25 * s;
        z = (m12 + m21) / s;
    }
    else
    {
        const double s = sqrt(1.0 + m22 - m00 - m11) * 2.0;
        w = (m10 - m01) / s;
        x = (m02 + m20) / s;
        y = (m12 + m21) / s;
        z = 0.25 * s;
    }

    const double norm = sqrt(x * x + y * y + z * z + w * w);
    if (norm > 0.0)
    {
        x /= norm;
        y /= norm;
        z /= norm;
        w /= norm;
    }

    *qx = (float)x;
    *qy = (float)y;
    *qz = (float)z;
    *qw = (float)w;
}

static void fill_detection_result(
    const apriltag_detection_t *det,
    const apriltag_pose_t *pose,
    float reprojection_error,
    AprilTagUnityDetection *out_detection)
{
    out_detection->id = det->id;
    out_detection->px = (float)MATD_EL(pose->t, 0, 0);
    out_detection->py = (float)MATD_EL(pose->t, 1, 0);
    out_detection->pz = (float)MATD_EL(pose->t, 2, 0);
    rotation_matrix_to_quaternion(
        pose->R,
        &out_detection->qx,
        &out_detection->qy,
        &out_detection->qz,
        &out_detection->qw);

    out_detection->reprojectionErrorPixels = reprojection_error;
    out_detection->corner0x = (float)det->p[0][0];
    out_detection->corner0y = (float)det->p[0][1];
    out_detection->corner1x = (float)det->p[1][0];
    out_detection->corner1y = (float)det->p[1][1];
    out_detection->corner2x = (float)det->p[2][0];
    out_detection->corner2y = (float)det->p[2][1];
    out_detection->corner3x = (float)det->p[3][0];
    out_detection->corner3y = (float)det->p[3][1];
}

static void rgba_to_gray_image(const uint8_t *rgba, int width, int height, int flip_vertically, image_u8_t *image)
{
    for (int y = 0; y < height; y++)
    {
        uint8_t *row = &image->buf[y * image->stride];
        const int source_y = flip_vertically ? (height - 1 - y) : y;
        for (int x = 0; x < width; x++)
        {
            const int rgba_index = 4 * (source_y * width + x);
            const uint8_t r = rgba[rgba_index + 0];
            const uint8_t g = rgba[rgba_index + 1];
            const uint8_t b = rgba[rgba_index + 2];
            row[x] = (uint8_t)((77 * r + 150 * g + 29 * b) >> 8);
        }
    }
}

APRILTAG_UNITY_EXPORT int apriltag_unity_smoke_test(void)
{
    apriltag_family_t *family = tag36h11_create();
    if (family == NULL)
    {
        return -1;
    }

    apriltag_detector_t *detector = apriltag_detector_create();
    if (detector == NULL)
    {
        tag36h11_destroy(family);
        return -2;
    }

    apriltag_detector_add_family_bits(detector, family, 1);
    apriltag_detector_destroy(detector);
    tag36h11_destroy(family);
    return 1;
}

APRILTAG_UNITY_EXPORT int apriltag_unity_detect_tag36h11(
    const uint8_t *rgba,
    int width,
    int height,
    float fx,
    float fy,
    float cx,
    float cy,
    float tag_size_meters,
    int target_tag_id,
    int flip_vertically,
    AprilTagUnityDetection *out_detection)
{
    if (rgba == NULL || out_detection == NULL)
    {
        return -1;
    }

    if (width <= 0 || height <= 0 || fx <= 0.0f || fy <= 0.0f || tag_size_meters <= 0.0f)
    {
        return -2;
    }

    image_u8_t *image = image_u8_create((unsigned int)width, (unsigned int)height);
    if (image == NULL)
    {
        return -3;
    }

    rgba_to_gray_image(rgba, width, height, flip_vertically, image);

    apriltag_family_t *family = tag36h11_create();
    if (family == NULL)
    {
        image_u8_destroy(image);
        return -4;
    }

    apriltag_detector_t *detector = apriltag_detector_create();
    if (detector == NULL)
    {
        tag36h11_destroy(family);
        image_u8_destroy(image);
        return -5;
    }

    detector->quad_decimate = 1.0f;
    detector->quad_sigma = 0.0f;
    detector->refine_edges = 1;
    detector->nthreads = 2;
    apriltag_detector_add_family_bits(detector, family, 1);

    zarray_t *detections = apriltag_detector_detect(detector, image);
    if (detections == NULL)
    {
        apriltag_detector_destroy(detector);
        tag36h11_destroy(family);
        image_u8_destroy(image);
        return -6;
    }

    int found = 0;
    int error_code = 0;
    AprilTagUnityDetection best_detection = {0};

    for (int i = 0; i < zarray_size(detections); i++)
    {
        apriltag_detection_t *det = NULL;
        zarray_get(detections, i, &det);
        if (det == NULL || det->id != target_tag_id)
        {
            continue;
        }

        apriltag_detection_info_t info;
        info.det = det;
        info.tagsize = tag_size_meters;
        info.fx = fx;
        info.fy = fy;
        info.cx = cx;
        info.cy = flip_vertically ? (double)(height - 1) - cy : cy;

        apriltag_pose_t pose;
        const double pose_error = estimate_tag_pose(&info, &pose);
        fill_detection_result(det, &pose, (float)pose_error, &best_detection);
        matd_destroy(pose.R);
        matd_destroy(pose.t);

        found = 1;
        break;
    }

    apriltag_detections_destroy(detections);
    apriltag_detector_destroy(detector);
    tag36h11_destroy(family);
    image_u8_destroy(image);

    if (error_code != 0)
    {
        return error_code;
    }

    if (!found)
    {
        return 0;
    }

    *out_detection = best_detection;
    return 1;
}
