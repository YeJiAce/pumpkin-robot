#ifndef PUMPKIN_CHASSIS_YAW_H
#define PUMPKIN_CHASSIS_YAW_H

#include <stdbool.h>

typedef struct
{
    float heading_reference;
    float heading_error;
    float rate_target;
    float rate_integral;
    bool holding;
} ChassisYaw;

/* Angles radians, rates rad/s; all signs use body +Z counterclockwise. */
void ChassisYaw_Reset(ChassisYaw *controller, float measured_yaw);
void ChassisYaw_SetHeading(ChassisYaw *controller, float target_yaw);
bool ChassisYaw_Update(ChassisYaw *controller, float commanded_rate, float measured_yaw,
                       float measured_rate, float dt_s, float *output_rate);

#endif
