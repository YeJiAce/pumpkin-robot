#include "display_app.h"
#include "cpu_load.h"
#include "motor_app.h"
#include "imu_app.h"
#include "main.h"
#include "oled.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <math.h>

static void FormatTenths(char text[7], int16_t value_x10)
{
    unsigned int magnitude = value_x10 < 0 ? (unsigned int)(-(int)value_x10) : (unsigned int)value_x10;
    if (magnitude > 9999U)
    {
        magnitude = 9999U;
    }
    (void)snprintf(text, 7U, "%c%03u.%u", value_x10 < 0 ? '-' : '+', magnitude / 10U, magnitude % 10U);
}

static void FormatImu(char text[22], const ImuAppStatus *imu)
{
    if (imu->state == IMU_OFFLINE)
    {
        (void)snprintf(text, 22U, "IMU: OFFLINE");
    }
    else if (imu->state == IMU_WARMUP)
    {
        (void)snprintf(text, 22U, "IMU: WARMUP");
    }
    else if (imu->state == IMU_CALIBRATING)
    {
        (void)snprintf(text, 22U, "IMU: CAL %u/200", (unsigned int)imu->calibration_samples);
    }
    else if (!imu->valid || ((HAL_GetTick() - imu->timestamp_ms) > 100U))
    {
        (void)snprintf(text, 22U, "IMU: NO DATA");
    }
    else if (!isfinite(imu->yaw_rad) || !isfinite(imu->gyro[2]))
    {
        (void)snprintf(text, 22U, "IMU: DATA ERROR");
    }
    else
    {
        char yaw[7];
        char rate[7];
        /* Y is relative yaw in degrees; W is body Z angular rate in rad/s. */
        FormatTenths(yaw, (int16_t)lroundf(imu->yaw_rad * 572.95779513f));
        long milli_rate = lroundf(imu->gyro[2] * 1000.0f);
        unsigned long magnitude = (unsigned long)(milli_rate < 0L ? -milli_rate : milli_rate);
        if (magnitude > 99999UL)
        {
            magnitude = 99999UL;
        }
        (void)snprintf(rate, sizeof(rate), "%c%02lu.%02lu", milli_rate < 0L ? '-' : '+',
                       magnitude / 1000UL, (magnitude % 1000UL) / 10UL);
        (void)snprintf(text, 22U, "IMU Y%s W%s", yaw, rate);
    }
}

void DisplayApp_Run(void)
{
    Oled_Init();
    for (;;)
    {
        MotorAppStatus motor;
        ImuAppStatus imu;
        uint16_t load;
        char first[22];
        char second[22];
        char third[22];
        char rpm[3][7];
        MotorApp_GetStatus(&motor);
        ImuApp_GetStatus(&imu);
        const char *mode = !motor.remote_mode ? "WAIT" : motor.fault != 0U ? "ROS FAULT" :
                           motor.running ? "ROS RUN" : motor.remote_armed ? "ROS ARMED" : "ROS SAFE";
        if (CpuLoad_Read(&load))
        {
            (void)snprintf(first, sizeof(first), "CPU:%u.%u%% %s", load / 10U, load % 10U, mode);
        }
        else
        {
            (void)snprintf(first, sizeof(first), "CPU:--.-%% %s", mode);
        }
        for (unsigned int i = 0U; i < MOTOR_COUNT; ++i)
        {
            FormatTenths(rpm[i], motor.rpm_x10[i]);
        }
        /* Three fixed-width signed RPM values, left to right M1/M2/M3. */
        (void)snprintf(second, sizeof(second), "RPM%s%s%s", rpm[0], rpm[1], rpm[2]);
        FormatImu(third, &imu);
        Oled_ShowLines(first, second, third);
        osDelay(200U);
    }
}
