#include "omni_kinematics.h"
#include "chassis_config.h"
#include <math.h>

#define SQRT3 1.7320508075688772f
#define MAX_WHEEL_RPM 240.0f

void Omni_ReferenceToBody(float vx, float vy, float yaw, float *body_x, float *body_y)
{
    float cosine = cosf(yaw);
    float sine = sinf(yaw);
    float rotated_x = cosine * vx + sine * vy;
    float rotated_y = -sine * vx + cosine * vy;
    *body_x = rotated_x;
    *body_y = rotated_y;
}

bool Omni_ToWheelRpm(float vx, float vy, float wz, float rpm[3])
{
    if (!isfinite(vx) || !isfinite(vy) || !isfinite(wz))
    {
        rpm[0] = rpm[1] = rpm[2] = 0.0f;
        return false;
    }
    /* Tangential wheel mounting assumed. Measured top tread directions:
       M1 backward-left, M2 backward-right, M3 left.
       Positive yaw requires electrical wheel signs +,-,-. */
    float rotation = CHASSIS_CENTER_RADIUS_M * wz;
    float factor = 60.0f / CHASSIS_WHEEL_CIRCUMFERENCE_M;
    rpm[0] = (-0.5f * SQRT3 * vx + 0.5f * vy + rotation) * factor;
    rpm[1] = (-0.5f * SQRT3 * vx - 0.5f * vy - rotation) * factor;
    rpm[2] = (vy - rotation) * factor;

    float peak = fmaxf(fabsf(rpm[0]), fmaxf(fabsf(rpm[1]), fabsf(rpm[2])));
    if (!isfinite(peak))
    {
        rpm[0] = rpm[1] = rpm[2] = 0.0f;
        return false;
    }
    if (peak > MAX_WHEEL_RPM)
    {
        for (unsigned int i = 0U; i < 3U; ++i)
        {
            rpm[i] *= MAX_WHEEL_RPM / peak;
        }
    }
    return true;
}

void Omni_FromWheelRpm(const float rpm[3], float *vx, float *vy, float *wz)
{
    float factor = CHASSIS_WHEEL_CIRCUMFERENCE_M / 60.0f;
    float a = rpm[0] * factor;
    float b = rpm[1] * factor;
    float c = rpm[2] * factor;
    *vx = -(a + b) / SQRT3;
    *vy = (a - b + 2.0f * c) / 3.0f;
    *wz = (a - b - c) / (3.0f * CHASSIS_CENTER_RADIUS_M);
}
