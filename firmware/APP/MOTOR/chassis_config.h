#ifndef PUMPKIN_CHASSIS_CONFIG_H
#define PUMPKIN_CHASSIS_CONFIG_H

/* Five-turn manual measurements: 7810, 7811, 7819 counts.
   Common initial calibration; refine later with longer repeat measurements. */
#define CHASSIS_ENCODER_COUNTS_PER_REV 1563.0f
#define CHASSIS_WHEEL_DIAMETER_M 0.085f
#define CHASSIS_CENTER_RADIUS_M  0.100f
#define CHASSIS_WHEEL_CIRCUMFERENCE_M (CHASSIS_WHEEL_DIAMETER_M * 3.14159265358979323846f)

#endif
