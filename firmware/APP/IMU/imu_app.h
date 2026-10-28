#ifndef PUMPKIN_IMU_APP_H
#define PUMPKIN_IMU_APP_H

#include <stdbool.h>
#include <stdint.h>

typedef enum { IMU_OFFLINE, IMU_WARMUP, IMU_CALIBRATING, IMU_READY } ImuState;

typedef struct
{
    ImuState state;
    bool valid;
    uint8_t address;
    uint8_t who_am_i;
    uint8_t stage;
    uint16_t calibration_samples;
    uint32_t errors;
    uint32_t i2c_error;
    uint32_t timestamp_ms;
    uint32_t sample_ms;
    uint32_t sequence;
    float accel[3]; /* Body frame m/s^2; gravity retained. */
    float gyro[3];  /* Body frame rad/s; bias corrected only in READY state. */
    float gyro_bias[3]; /* Body frame rad/s. */
    float yaw_rad; /* Relative gyro-Z integral; not an absolute compass heading. */
} ImuAppStatus;

void ImuApp_Run(void);
void ImuApp_GetStatus(ImuAppStatus *status);
bool ImuApp_RequestCalibration(void);

#endif
