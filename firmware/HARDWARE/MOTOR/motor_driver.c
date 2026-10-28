/** D24A outputs. Peripheral configuration belongs exclusively to CubeMX. */
#include "motor_driver.h"

#include "main.h"
#include "tim.h"
#include <math.h>

typedef struct
{
    uint32_t channel;
    GPIO_TypeDef *in1_port;
    uint16_t in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t in2_pin;
} MotorOutput;

#define MOTOR_PORT_COUNT 4U
#define REVERSAL_COAST_MS 100U

static int8_t drive_sign[MOTOR_COUNT];
static bool reversing[MOTOR_COUNT];
static uint32_t reversed_at[MOTOR_COUNT];

/* C,B,D chassis order. A/D wiring is swapped relative to the supplied example
   for this diagnostic build; confirm against physical port labels on hardware.
   CubeMX MOTOR_A/MOTOR_D labels below still name the original example signals. */
static const MotorOutput outputs[MOTOR_PORT_COUNT] =
{
    {TIM_CHANNEL_1, MOTOR_C_IN1_GPIO_Port, MOTOR_C_IN1_Pin,
                    MOTOR_C_IN2_GPIO_Port, MOTOR_C_IN2_Pin},
    {TIM_CHANNEL_4, MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin,
                    MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin},
    {TIM_CHANNEL_3, MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin,
                    MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin},
    {TIM_CHANNEL_2, MOTOR_D_IN1_GPIO_Port, MOTOR_D_IN1_Pin,
                    MOTOR_D_IN2_GPIO_Port, MOTOR_D_IN2_Pin}
};

void MotorDriver_StopAll(void)
{
    /* Disable the bridge before modifying direction or buffered PWM registers. */
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_RESET);

    for (uint8_t i = 0U; i < MOTOR_PORT_COUNT; ++i)
    {
        __HAL_TIM_SET_COMPARE(&htim8, outputs[i].channel, 0U);
        HAL_GPIO_WritePin(outputs[i].in1_port, outputs[i].in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(outputs[i].in2_port, outputs[i].in2_pin, GPIO_PIN_RESET);
    }
    for (uint8_t i = 0U; i < MOTOR_COUNT; ++i)
    {
        drive_sign[i] = 0;
        reversing[i] = false;
    }
}

bool MotorDriver_Init(void)
{
    MotorDriver_StopAll();

    if (HAL_TIM_PWM_Stop(&htim8, outputs[MOTOR_COUNT].channel) != HAL_OK)
    {
        return false;
    }

    for (uint8_t i = 0U; i < MOTOR_COUNT; ++i)
    {
        if (HAL_TIM_PWM_Start(&htim8, outputs[i].channel) != HAL_OK)
        {
            MotorDriver_StopAll();
            return false;
        }
    }
    return true;
}

bool MotorDriver_TestOne(uint8_t index, uint8_t duty_percent)
{
    return MotorDriver_TestSigned(index, duty_percent);
}

bool MotorDriver_TestSigned(uint8_t index, int16_t duty_percent)
{
    MotorDriver_StopAll();
    if ((index >= MOTOR_COUNT) || (duty_percent < -100) || (duty_percent > 100))
    {
        return false;
    }
    if (duty_percent == 0)
    {
        return true;
    }

    const MotorOutput *motor = &outputs[index];
    drive_sign[index] = duty_percent < 0 ? -1 : 1;
    TIM_CCxChannelCmd(htim8.Instance, motor->channel, TIM_CCx_ENABLE);
    /* Match the manufacturer's positive electrical direction: IN1=0, IN2=1.
       Physical forward direction and encoder polarity remain uncalibrated. */
    HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin,
                      duty_percent < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin,
                      duty_percent < 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    uint32_t magnitude = (uint32_t)(duty_percent < 0 ? -duty_percent : duty_percent);
    __HAL_TIM_SET_COMPARE(&htim8, motor->channel,
                         ((__HAL_TIM_GET_AUTORELOAD(&htim8) + 1U) * magnitude) / 100U);

    /* Latch all zeroed channels before enabling the common bridge. */
    if (HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_UPDATE) != HAL_OK)
    {
        MotorDriver_StopAll();
        return false;
    }
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET);
    return true;
}

bool MotorDriver_TestAll(uint8_t duty_percent)
{
    MotorDriver_StopAll();
    if ((duty_percent == 0U) || (duty_percent > 100U))
    {
        return false;
    }

    uint32_t compare = ((__HAL_TIM_GET_AUTORELOAD(&htim8) + 1U) * duty_percent) / 100U;
    for (uint8_t i = 0U; i < MOTOR_COUNT; ++i)
    {
        drive_sign[i] = 1;
        TIM_CCxChannelCmd(htim8.Instance, outputs[i].channel, TIM_CCx_ENABLE);
        HAL_GPIO_WritePin(outputs[i].in1_port, outputs[i].in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(outputs[i].in2_port, outputs[i].in2_pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim8, outputs[i].channel, compare);
    }

    if (HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_UPDATE) != HAL_OK)
    {
        MotorDriver_StopAll();
        return false;
    }
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET);
    return true;
}

bool MotorDriver_UpdateDuty(uint8_t index, uint16_t permille)
{
    if ((index >= MOTOR_COUNT) || (permille > 1000U))
    {
        return false;
    }
    __HAL_TIM_SET_COMPARE(&htim8, outputs[index].channel,
                         ((__HAL_TIM_GET_AUTORELOAD(&htim8) + 1U) * permille) / 1000U);
    return true;
}

bool MotorDriver_BeginWheels(const float signed_rpm[MOTOR_COUNT])
{
    MotorDriver_StopAll();
    for (uint8_t i = 0U; i < MOTOR_COUNT; ++i)
    {
        if (!isfinite(signed_rpm[i]))
        {
            MotorDriver_StopAll();
            return false;
        }
        if (signed_rpm[i] != 0.0f)
        {
            drive_sign[i] = signed_rpm[i] < 0.0f ? -1 : 1;
            HAL_GPIO_WritePin(outputs[i].in1_port, outputs[i].in1_pin,
                              signed_rpm[i] < 0.0f ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(outputs[i].in2_port, outputs[i].in2_pin,
                              signed_rpm[i] < 0.0f ? GPIO_PIN_RESET : GPIO_PIN_SET);
        }
        TIM_CCxChannelCmd(htim8.Instance, outputs[i].channel, TIM_CCx_ENABLE);
    }
    if (HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_UPDATE) != HAL_OK)
    {
        MotorDriver_StopAll();
        return false;
    }
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET);
    return true;
}

bool MotorDriver_ApplySignedDuty(const int16_t duty[MOTOR_COUNT], int16_t effective[MOTOR_COUNT])
{
    for (uint8_t i = 0U; i < MOTOR_COUNT; ++i)
    {
        if ((duty[i] < -1000) || (duty[i] > 1000))
        {
            MotorDriver_StopAll();
            return false;
        }
    }
    uint32_t now = HAL_GetTick();
    for (uint8_t i = 0U; i < MOTOR_COUNT; ++i)
    {
        effective[i] = 0;
        int8_t requested = duty[i] < 0 ? -1 : duty[i] > 0 ? 1 : 0;
        if ((requested != 0) && (drive_sign[i] != 0) && (requested != drive_sign[i]))
        {
            /* Disable output immediately: CCR is preloaded and alone is not sufficient. */
            TIM_CCxChannelCmd(htim8.Instance, outputs[i].channel, TIM_CCx_DISABLE);
            __HAL_TIM_SET_COMPARE(&htim8, outputs[i].channel, 0U);
            HAL_GPIO_WritePin(outputs[i].in1_port, outputs[i].in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(outputs[i].in2_port, outputs[i].in2_pin, GPIO_PIN_RESET);
            drive_sign[i] = 0;
            reversing[i] = true;
            reversed_at[i] = now;
        }
        if (reversing[i] && ((now - reversed_at[i]) < REVERSAL_COAST_MS))
        {
            continue;
        }
        reversing[i] = false;
        if (requested == 0)
        {
            __HAL_TIM_SET_COMPARE(&htim8, outputs[i].channel, 0U);
            continue;
        }
        if (drive_sign[i] == 0)
        {
            HAL_GPIO_WritePin(outputs[i].in1_port, outputs[i].in1_pin,
                              requested < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(outputs[i].in2_port, outputs[i].in2_pin,
                              requested < 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
            drive_sign[i] = requested;
        }
        uint32_t magnitude = (uint32_t)(duty[i] < 0 ? -duty[i] : duty[i]);
        __HAL_TIM_SET_COMPARE(&htim8, outputs[i].channel,
                             ((__HAL_TIM_GET_AUTORELOAD(&htim8) + 1U) * magnitude) / 1000U);
        TIM_CCxChannelCmd(htim8.Instance, outputs[i].channel, TIM_CCx_ENABLE);
        effective[i] = duty[i];
    }
    return HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_UPDATE) == HAL_OK;
}
