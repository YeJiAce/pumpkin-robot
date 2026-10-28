/** Six-axis register driver based on the supplied WHEELTEC ICM20948 example. */
#include "icm20948.h"
#include "i2c.h"
#include "cmsis_os.h"
#include <stddef.h>

#define REG_BANK_SEL       0x7FU
#define WHO_AM_I           0x00U
#define USER_CTRL          0x03U
#define LP_CONFIG          0x05U
#define PWR_MGMT_1         0x06U
#define PWR_MGMT_2         0x07U
#define ACCEL_XOUT_H       0x2DU
#define GYRO_SMPLRT_DIV    0x00U
#define GYRO_CONFIG_1      0x01U
#define ACCEL_SMPLRT_DIV_1 0x10U
#define ACCEL_SMPLRT_DIV_2 0x11U
#define ACCEL_CONFIG       0x14U
#define I2C_TIMEOUT_MS     5U
#define GRAVITY_MPS2       9.80665f
#define DEG_TO_RAD         0.017453292519943295f

/* Gyro FS_SEL=2: +/-1000 degrees/s; accel FS_SEL=1: +/-4g. */
#define RANGE_FILTER       0x1BU
#define GYRO_RANGE_FILTER  0x1DU

static Icm20948Info device;

static bool Write(uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef result = HAL_I2C_Mem_Write(&hi2c2, (uint16_t)(device.address << 1),
                                                reg, I2C_MEMADD_SIZE_8BIT, &value, 1U,
                                                I2C_TIMEOUT_MS);
    device.i2c_error = HAL_I2C_GetError(&hi2c2);
    return result == HAL_OK;
}

static bool Read(uint8_t reg, uint8_t *bytes, uint16_t count)
{
    HAL_StatusTypeDef result = HAL_I2C_Mem_Read(&hi2c2, (uint16_t)(device.address << 1),
                                               reg, I2C_MEMADD_SIZE_8BIT, bytes, count,
                                               I2C_TIMEOUT_MS);
    device.i2c_error = HAL_I2C_GetError(&hi2c2);
    return result == HAL_OK;
}

bool Icm20948_Init(void)
{
    device = (Icm20948Info){.stage = 1U};
    static const uint8_t addresses[] = {0x68U, 0x69U};
    bool detected = false;
    for (size_t i = 0U; i < sizeof(addresses); ++i)
    {
        device.address = addresses[i];
        if (Write(REG_BANK_SEL, 0x00U) && Read(WHO_AM_I, &device.who_am_i, 1U) &&
            (device.who_am_i == 0xEAU))
        {
            detected = true;
            break;
        }
    }
    if (!detected)
    {
        return false;
    }

    device.stage = 2U;
    if (!Write(PWR_MGMT_1, 0x80U))
    {
        return false;
    }
    osDelay(100U);

    device.stage = 3U;
    if (!Write(REG_BANK_SEL, 0x00U) || !Write(USER_CTRL, 0x00U) ||
        !Write(LP_CONFIG, 0x00U) || !Write(PWR_MGMT_1, 0x01U) || !Write(PWR_MGMT_2, 0x00U))
    {
        return false;
    }
    osDelay(30U);

    device.stage = 4U;
    /* Native ODR is above 100 Hz so each 10-ms task read gets fresh six-axis data. */
    if (!Write(REG_BANK_SEL, 0x20U) || !Write(GYRO_SMPLRT_DIV, 4U) ||
        !Write(GYRO_CONFIG_1, GYRO_RANGE_FILTER) || !Write(ACCEL_SMPLRT_DIV_1, 0U) ||
        !Write(ACCEL_SMPLRT_DIV_2, 4U) || !Write(ACCEL_CONFIG, RANGE_FILTER))
    {
        return false;
    }

    device.stage = 5U;
    uint8_t gyro_config, accel_config;
    if (!Read(GYRO_CONFIG_1, &gyro_config, 1U) || !Read(ACCEL_CONFIG, &accel_config, 1U) ||
        ((gyro_config & 0x3FU) != GYRO_RANGE_FILTER) || ((accel_config & 0x3FU) != RANGE_FILTER) ||
        !Write(REG_BANK_SEL, 0x00U) || !Read(WHO_AM_I, &device.who_am_i, 1U) ||
        (device.who_am_i != 0xEAU))
    {
        return false;
    }
    device.stage = 0U;
    return true;
}

static int32_t SignedBigEndian(const uint8_t *bytes)
{
    uint32_t value = ((uint32_t)bytes[0] << 8) | bytes[1];
    return value < 32768U ? (int32_t)value : (int32_t)value - 65536;
}

bool Icm20948_Read(Icm20948Sample *sample)
{
    uint8_t bytes[12];
    device.stage = 6U;
    if (!Read(ACCEL_XOUT_H, bytes, sizeof(bytes)))
    {
        return false;
    }
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        sample->accel[i] = SignedBigEndian(&bytes[2U * i]) * (4.0f * GRAVITY_MPS2 / 32768.0f);
        sample->gyro[i] = SignedBigEndian(&bytes[6U + 2U * i]) * (1000.0f * DEG_TO_RAD / 32768.0f);
    }
    device.stage = 0U;
    return true;
}

void Icm20948_GetInfo(Icm20948Info *info)
{
    *info = device;
}
