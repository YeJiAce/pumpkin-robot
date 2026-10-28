#include "encoder.h"

#include "tim.h"

/* Diagnostic A/D swap: motor3 uses the example's A encoder (TIM5). */
static TIM_HandleTypeDef *const timers[MOTOR_COUNT] = {&htim4, &htim3, &htim5};
static uint16_t previous[MOTOR_COUNT];

bool Encoder_Init(void)
{
    for (uint8_t i = 0U; i < MOTOR_COUNT; ++i)
    {
        if (HAL_TIM_Encoder_Start(timers[i], TIM_CHANNEL_ALL) != HAL_OK)
        {
            return false;
        }
        previous[i] = (uint16_t)__HAL_TIM_GET_COUNTER(timers[i]);
    }
    return true;
}

void Encoder_Sample(int32_t delta[MOTOR_COUNT])
{
    for (uint8_t i = 0U; i < MOTOR_COUNT; ++i)
    {
        uint16_t current = (uint16_t)__HAL_TIM_GET_COUNTER(timers[i]);
        uint16_t difference = (uint16_t)(current - previous[i]);
        delta[i] = (difference < 32768U) ? (int32_t)difference
                                        : (int32_t)difference - 65536;
        previous[i] = current;
    }
}
