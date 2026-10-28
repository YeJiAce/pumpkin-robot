/** Periodic IMU acquisition, static gyro calibration and body-frame conversion. */
#include "imu_app.h"
#include "icm20948.h"
#include "motor_app.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

#define IMU_PERIOD_MS 10U
#define WARMUP_MS 300U
#define CALIBRATION_COUNT 200U
#define GRAVITY_MPS2 9.80665f
#define PI 3.14159265358979323846f

typedef struct
{
    uint16_t count;
    float sum[3];
    float square_sum[3];
} GyroCalibration;

static ImuAppStatus published;
static bool recalibrate;

void ImuApp_GetStatus(ImuAppStatus *status)
{
    taskENTER_CRITICAL();
    *status = published;
    taskEXIT_CRITICAL();
}

static void Publish(ImuAppStatus *status)
{
    Icm20948Info info;
    Icm20948_GetInfo(&info);
    status->address = info.address;
    status->who_am_i = info.who_am_i;
    status->stage = info.stage;
    status->i2c_error = info.i2c_error;
    taskENTER_CRITICAL();
    published = *status;
    taskEXIT_CRITICAL();
}

static bool MotorsStill(void)
{
    MotorAppStatus motor;
    MotorApp_GetStatus(&motor);
    if (motor.running)
    {
        return false;
    }
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        if ((motor.rpm_x10[i] < -10) || (motor.rpm_x10[i] > 10))
        {
            return false;
        }
    }
    return true;
}

bool ImuApp_RequestCalibration(void)
{
    if (!MotorsStill())
    {
        return false;
    }
    taskENTER_CRITICAL();
    bool accepted = published.state != IMU_OFFLINE && !recalibrate;
    if (accepted)
    {
        recalibrate = true;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

static bool TakeCalibrationRequest(void)
{
    taskENTER_CRITICAL();
    bool requested = recalibrate;
    recalibrate = false;
    taskEXIT_CRITICAL();
    return requested;
}

static void ToBody(const float sensor[3], float body[3])
{
    /* Board +Y points forward; assumes board +Z up: proper rotation, det=+1. */
    body[0] = sensor[1];
    body[1] = -sensor[0];
    body[2] = sensor[2];
}

static bool Calibrate(GyroCalibration *cal, const Icm20948Sample *sample, float bias[3])
{
    float accel_norm = 0.0f;
    float gyro_norm = 0.0f;
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        accel_norm += sample->accel[i] * sample->accel[i];
        gyro_norm += sample->gyro[i] * sample->gyro[i];
    }
    if (!MotorsStill() || (fabsf(sqrtf(accel_norm) - GRAVITY_MPS2) > 0.8f) ||
        (gyro_norm > 0.01f))
    {
        memset(cal, 0, sizeof(*cal));
        return false;
    }
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        cal->sum[i] += sample->gyro[i];
        cal->square_sum[i] += sample->gyro[i] * sample->gyro[i];
    }
    ++cal->count;
    if (cal->count < CALIBRATION_COUNT)
    {
        return false;
    }
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        float mean = cal->sum[i] / CALIBRATION_COUNT;
        float variance = cal->square_sum[i] / CALIBRATION_COUNT - mean * mean;
        if (variance > 0.000025f) /* Gyro standard deviation above 0.005 rad/s. */
        {
            memset(cal, 0, sizeof(*cal));
            return false;
        }
    }
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        bias[i] = cal->sum[i] / CALIBRATION_COUNT;
    }
    return true;
}

void ImuApp_Run(void)
{
    ImuAppStatus status = {0};
    GyroCalibration calibration = {0};
    float bias[3] = {0};
    uint8_t failures = 0U;
    uint32_t initialized_at = 0U;
    uint32_t previous_sample = 0U;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        if (status.state == IMU_OFFLINE)
        {
            status.valid = false;
            if (!Icm20948_Init())
            {
                ++status.errors;
                Publish(&status);
                vTaskDelay(pdMS_TO_TICKS(1000U));
                continue;
            }
            status.state = IMU_WARMUP;
            memset(&calibration, 0, sizeof(calibration));
            memset(bias, 0, sizeof(bias));
            memset(status.gyro_bias, 0, sizeof(status.gyro_bias));
            status.calibration_samples = 0U;
            status.yaw_rad = 0.0f;
            initialized_at = HAL_GetTick();
            previous_sample = initialized_at;
            last_wake = xTaskGetTickCount();
            failures = 0U;
            Publish(&status);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_PERIOD_MS));
        Icm20948Sample sample;
        if (!Icm20948_Read(&sample))
        {
            ++status.errors;
            status.valid = false;
            if (status.state == IMU_CALIBRATING)
            {
                memset(&calibration, 0, sizeof(calibration));
                status.calibration_samples = 0U;
            }
            if (++failures >= 3U)
            {
                status.state = IMU_OFFLINE;
                /* Avoid immediate reconnect loops on a persistent hardware fault. */
                Publish(&status);
                vTaskDelay(pdMS_TO_TICKS(1000U));
                continue;
            }
            Publish(&status);
            continue;
        }
        failures = 0U;
        uint32_t now = HAL_GetTick();
        status.sample_ms = now - previous_sample;
        previous_sample = now;
        status.timestamp_ms = now;
        status.valid = true;
        ++status.sequence;

        if (TakeCalibrationRequest())
        {
            status.state = IMU_CALIBRATING;
            status.yaw_rad = 0.0f;
            memset(&calibration, 0, sizeof(calibration));
            memset(bias, 0, sizeof(bias));
            memset(status.gyro_bias, 0, sizeof(status.gyro_bias));
        }
        if (status.state == IMU_WARMUP && ((now - initialized_at) >= WARMUP_MS))
        {
            status.state = IMU_CALIBRATING;
        }
        if (status.state == IMU_CALIBRATING)
        {
            if (status.sample_ms > 30U)
            {
                memset(&calibration, 0, sizeof(calibration));
            }
            if (Calibrate(&calibration, &sample, bias))
            {
                status.state = IMU_READY;
                status.yaw_rad = 0.0f;
            }
            status.calibration_samples = calibration.count;
        }

        ToBody(sample.accel, status.accel);
        float corrected[3];
        for (unsigned int i = 0U; i < 3U; ++i)
        {
            corrected[i] = sample.gyro[i] - (status.state == IMU_READY ? bias[i] : 0.0f);
        }
        ToBody(corrected, status.gyro);
        ToBody(bias, status.gyro_bias);
        if (status.state == IMU_READY && (status.sample_ms <= 30U))
        {
            status.yaw_rad += status.gyro[2] * status.sample_ms * 0.001f;
            if (status.yaw_rad > PI)
            {
                status.yaw_rad -= 2.0f * PI;
            }
            else if (status.yaw_rad < -PI)
            {
                status.yaw_rad += 2.0f * PI;
            }
        }
        Publish(&status);
    }
}
