#ifndef PUMPKIN_MOTOR_APP_H
#define PUMPKIN_MOTOR_APP_H

#include "motor_driver.h"

typedef struct
{
    bool ready;
    bool running;
    bool speed_mode;
    bool chassis_mode;
    bool velocity_stream;
    bool heading_hold;
    bool heading_move;
    bool spin_mode;
    float spin_reference_rad;
    uint8_t fault; /* 0=none, 1=IMU unavailable/stale, 2=control timing, 3=command timeout. */
    bool remote_mode;
    uint32_t mode_generation; /* Changes on every physical USER mode switch. */
    bool remote_armed;
    uint8_t remote_result;
    uint8_t stop_reason;
    uint16_t last_applied_cmd_seq;
    uint16_t command_age_ms;
    uint32_t remote_ticket;
    uint32_t timestamp_ms;
    uint32_t encoder_count[MOTOR_COUNT]; /* Modulo 2^32, never cleared by handshake. */
    int16_t wheel_pwm_permille[MOTOR_COUNT];
    int16_t yaw_reference_mrad;
    int16_t yaw_error_mrad;
    int16_t yaw_rate_target_mradps;
    int16_t yaw_rate_measured_mradps;
    int16_t yaw_rate_output_mradps;
    int16_t wheel_target_x10[MOTOR_COUNT];
    int16_t wheel_pwm[MOTOR_COUNT];
    int16_t body_command[3]; /* vx mm/s, vy mm/s, wz mrad/s. */
    int16_t body_measured[3]; /* Wheel-only estimate, same units; not fused odometry. */
    int16_t target_rpm;
    int16_t rpm_x10[MOTOR_COUNT];
    int16_t duty_percent;
    uint8_t motor_number; /* 0=all motors; 1=C, 2=B, 3=D for individual tests. */
    uint32_t transitions;
    uint32_t sample_ms;
    uint32_t idle_ms;
    int32_t delta[MOTOR_COUNT];
    int32_t test_counts[MOTOR_COUNT]; /* Counts since boot or explicit zero command. */
} MotorAppStatus;

typedef enum { MOTOR_REQUEST_TEST, MOTOR_REQUEST_SPEED, MOTOR_REQUEST_STOP, MOTOR_REQUEST_ZERO } MotorRequestType;
/* Legacy local test APIs are disabled (return false) in the ROS handoff firmware.
   Use the session protocol; USER only switches ROS permission and stops motors. */
/* value = signed PWM percent for TEST, signed RPM for SPEED. */
bool MotorApp_Request(MotorRequestType type, uint8_t motor_number, int16_t value);
/* Bounded bench command: +-800 mm/s X/Y, +-10000 mrad/s yaw, 5s duration. */
bool MotorApp_RequestMove(int16_t vx_mmps, int16_t vy_mmps, int16_t wz_mradps);
/* Velocity stream requires refresh within 500 ms; all-zero command stops. */
bool MotorApp_RequestVelocity(int16_t vx_mmps, int16_t vy_mmps, int16_t wz_mradps);
/* Absolute target in the IMU's relative-yaw frame, -180..180 degrees. */
bool MotorApp_RequestHeading(int16_t heading_degrees);
typedef enum
{
    MOTOR_REMOTE_RESET, MOTOR_REMOTE_ARM, MOTOR_REMOTE_DISARM, MOTOR_REMOTE_STOP,
    MOTOR_REMOTE_VELOCITY, MOTOR_REMOTE_HEADING, MOTOR_REMOTE_CALIBRATE, MOTOR_REMOTE_SPIN
} MotorRemoteAction;
typedef struct
{
    MotorRemoteAction action;
    uint32_t ticket;
    uint32_t mode_generation;
    uint16_t sequence;
    int16_t values[3];
} MotorRemoteRequest;
/* Applies in motorTask; result/ticket published only after application. */
bool MotorApp_PostRemote(const MotorRemoteRequest *request);
void MotorApp_Run(void);
void MotorApp_GetStatus(MotorAppStatus *status);

#endif
