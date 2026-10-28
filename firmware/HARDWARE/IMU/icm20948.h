#ifndef PUMPKIN_ICM20948_H
#define PUMPKIN_ICM20948_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float accel[3]; /* Sensor frame, m/s^2, including gravity. */
    float gyro[3];  /* Sensor frame, rad/s, uncorrected bias. */
} Icm20948Sample;

typedef struct
{
    uint8_t address; /* Seven-bit I2C address, normally 0x68. */
    uint8_t who_am_i;
    uint8_t stage; /* 1=detect, 2=reset, 3=power, 4=range/filter, 5=readback, 6=sample. */
    uint32_t i2c_error;
} Icm20948Info;

/* Only imuTask may access I2C2. Bus/pin setup remains CubeMX-owned. */
bool Icm20948_Init(void);
bool Icm20948_Read(Icm20948Sample *sample);
void Icm20948_GetInfo(Icm20948Info *info);

#endif
