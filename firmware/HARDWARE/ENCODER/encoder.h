#ifndef PUMPKIN_ENCODER_H
#define PUMPKIN_ENCODER_H

#include "motor_driver.h"

bool Encoder_Init(void);
/* Read modulo-65536 differences without clearing a running hardware counter.
   Each sampling interval must contain fewer than 32768 counts per motor. */
void Encoder_Sample(int32_t delta[MOTOR_COUNT]);

#endif
