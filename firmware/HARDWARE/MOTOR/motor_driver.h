#ifndef PUMPKIN_MOTOR_DRIVER_H
#define PUMPKIN_MOTOR_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_COUNT 3U

/* Logical indices 0/1/2 correspond to MotorC/MotorB/MotorD. */
bool MotorDriver_Init(void);
void MotorDriver_StopAll(void);
bool MotorDriver_TestOne(uint8_t index, uint8_t duty_percent);
/* Signed electrical duty, -100..100. Only motorTask may call this API. */
bool MotorDriver_TestSigned(uint8_t index, int16_t duty_percent);
/* Update the active motor PWM without disabling the bridge or changing direction. */
bool MotorDriver_UpdateDuty(uint8_t index, uint16_t permille);
/* Configure all three directions once with zero PWM; signed values select direction.
   Caller must stop/coast before starting or reversing. Unused A remains disabled. */
bool MotorDriver_BeginWheels(const float signed_rpm[MOTOR_COUNT]);
/* Atomic control-cycle update. Per-wheel reversal blanks that channel for 100 ms.
   effective receives actual signed permille after reversal interlocks. */
bool MotorDriver_ApplySignedDuty(const int16_t duty[MOTOR_COUNT], int16_t effective[MOTOR_COUNT]);
/* Drive chassis ports C/B/D simultaneously; A stays disabled. */
bool MotorDriver_TestAll(uint8_t duty_percent);

#endif
