#ifndef PUMPKIN_OMNI_KINEMATICS_H
#define PUMPKIN_OMNI_KINEMATICS_H

#include <stdbool.h>

/* Body frame: +X forward (IMU +Y), +Y left, +yaw counterclockwise from above.
   Wheel order C/B/D = front-left/front-right/rear.
   Wheel signs follow measured positive electrical output, not chassis forward. */
bool Omni_ToWheelRpm(float vx_mps, float vy_mps, float wz_radps, float rpm[3]);
void Omni_FromWheelRpm(const float rpm[3], float *vx_mps, float *vy_mps, float *wz_radps);
/* Reference axes are fixed at spin-mode entry; delta_yaw = current - entry yaw. */
void Omni_ReferenceToBody(float vx_ref, float vy_ref, float delta_yaw, float *vx_body, float *vy_body);

#endif
