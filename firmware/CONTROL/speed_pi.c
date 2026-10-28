#include "speed_pi.h"
#include <math.h>

#define KP 2.5f
#define KI 5.0f
#define FEEDFORWARD 2.73f
#define OUTPUT_LIMIT 600.0f

void SpeedPi_Reset(SpeedPi *controller)
{
    controller->integral = 0.0f;
}

float SpeedPi_Update(SpeedPi *controller, float target_rpm, float measured_rpm, float dt_s)
{
    if (!isfinite(target_rpm) || !isfinite(measured_rpm) || !isfinite(dt_s) ||
        (target_rpm <= 0.0f) || (dt_s <= 0.0f) || (dt_s > 0.05f))
    {
        SpeedPi_Reset(controller);
        return 0.0f;
    }

    float error = target_rpm - measured_rpm;
    float integral = controller->integral + KI * error * dt_s;
    float output = FEEDFORWARD * target_rpm + KP * error + integral;

    /* Conditional integration prevents windup while allowing recovery from limits. */
    if (!((output > OUTPUT_LIMIT && error > 0.0f) || (output < 0.0f && error < 0.0f)))
    {
        controller->integral = integral;
    }
    output = FEEDFORWARD * target_rpm + KP * error + controller->integral;
    return output > OUTPUT_LIMIT ? OUTPUT_LIMIT : output < 0.0f ? 0.0f : output;
}
