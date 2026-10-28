/** Cascaded heading P -> angular-rate PI with feedforward, ahead of wheel PI. */
#include "chassis_yaw.h"
#include <math.h>

#define PI 3.14159265358979323846f
#define HEADING_KP 1.5f
#define HEADING_RATE_LIMIT 0.3f
#define HEADING_DEADBAND 0.005f
#define RATE_KP 0.35f
#define RATE_KI 0.6f
#define RATE_CORRECTION_LIMIT 0.25f
#define RATE_INTEGRAL_LIMIT 0.20f
#define RATE_OUTPUT_LIMIT 10.5f
#define RATE_SLEW_PER_SECOND 2.0f

static float Limit(float value, float magnitude)
{
    return value > magnitude ? magnitude : value < -magnitude ? -magnitude : value;
}

static float Wrap(float angle)
{
    /* Inputs are wrapped IMU yaw and bounded heading commands. */
    while (angle > PI)
    {
        angle -= 2.0f * PI;
    }
    while (angle < -PI)
    {
        angle += 2.0f * PI;
    }
    return angle;
}

void ChassisYaw_Reset(ChassisYaw *controller, float measured_yaw)
{
    *controller = (ChassisYaw){.heading_reference = measured_yaw};
}

void ChassisYaw_SetHeading(ChassisYaw *controller, float target_yaw)
{
    ChassisYaw_Reset(controller, target_yaw);
    controller->holding = true;
}

bool ChassisYaw_Update(ChassisYaw *controller, float command, float yaw,
                       float rate, float dt, float *output)
{
    *output = 0.0f;
    if (!isfinite(command) || !isfinite(yaw) || !isfinite(rate) || !isfinite(dt) ||
        (dt <= 0.0f) || (dt > 0.05f))
    {
        return false;
    }

    float target;
    if (fabsf(command) > 0.01f)
    {
        /* While turning, track the current heading for a subsequent hold transition. */
        controller->holding = false;
        controller->heading_reference = yaw;
        controller->heading_error = 0.0f;
        target = Limit(command, 10.0f);
    }
    else
    {
        if (!controller->holding)
        {
            controller->heading_reference = yaw;
            controller->rate_integral = 0.0f;
            controller->holding = true;
        }
        controller->heading_error = Wrap(controller->heading_reference - yaw);
        target = fabsf(controller->heading_error) < HEADING_DEADBAND ? 0.0f :
                 Limit(HEADING_KP * controller->heading_error, HEADING_RATE_LIMIT);
    }

    controller->rate_target += Limit(target - controller->rate_target, RATE_SLEW_PER_SECOND * dt);
    float error = controller->rate_target - rate;
    float integral = Limit(controller->rate_integral + RATE_KI * error * dt, RATE_INTEGRAL_LIMIT);
    float correction = RATE_KP * error + integral;
    float candidate = controller->rate_target + correction;
    bool high = ((correction > RATE_CORRECTION_LIMIT) || (candidate > RATE_OUTPUT_LIMIT)) && error > 0.0f;
    bool low = ((correction < -RATE_CORRECTION_LIMIT) || (candidate < -RATE_OUTPUT_LIMIT)) && error < 0.0f;
    if (!high && !low)
    {
        controller->rate_integral = integral;
    }
    correction = Limit(RATE_KP * error + controller->rate_integral, RATE_CORRECTION_LIMIT);
    *output = Limit(controller->rate_target + correction, RATE_OUTPUT_LIMIT);
    return true;
}
