#ifndef PUMPKIN_SPEED_PI_H
#define PUMPKIN_SPEED_PI_H

typedef struct
{
    float integral;
} SpeedPi;

void SpeedPi_Reset(SpeedPi *controller);
/* Magnitudes only: RPM in, PWM permille out (0..600). Direction handled by driver.
   Initial tuning: Kp=2.5 permille/RPM, Ki=5 permille/(RPM*s), FF=2.73/RPM. */
float SpeedPi_Update(SpeedPi *controller, float target_rpm, float measured_rpm, float dt_s);

#endif
