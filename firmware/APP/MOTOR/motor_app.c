/** Motor task owns wheel PI, yaw loops, command watchdogs and feedback snapshots. */
#include "motor_app.h"
#include "chassis_config.h"
#include "encoder.h"
#include "user_key.h"

#include "imu_app.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "speed_pi.h"
#include "omni_kinematics.h"
#include "chassis_yaw.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define MOTOR_PERIOD_MS 10U
#define PWM_DURATION_MS 2000U
#define SPEED_DURATION_MS 5000U
#define MOVE_DURATION_MS 5000U


#define HEADING_DURATION_MS 8000U
#define VELOCITY_TIMEOUT_MS 500U
#define RESTART_GAP_MS 300U
#define IMU_MAX_AGE_MS 50U
#define PI 3.14159265358979323846f

typedef struct
{
    MotorRequestType type;
    uint8_t motor_number;
    int16_t value;
    bool chassis;
    bool streaming;
    bool heading_move;
    bool spin;
    float heading;
    float wheel_rpm[MOTOR_COUNT];
    int16_t body[3];
    uint32_t queued_at_ms;
} MotorRequest;

static MotorAppStatus published;


static MotorRemoteRequest remote_mailbox;
static bool remote_pending;

bool MotorApp_PostRemote(const MotorRemoteRequest *request)
{
    bool priority = request->action == MOTOR_REMOTE_RESET || request->action == MOTOR_REMOTE_STOP ||
                    request->action == MOTOR_REMOTE_DISARM;
    taskENTER_CRITICAL();
    bool accepted = published.ready && (!remote_pending || priority);
    if (accepted)
    {
        remote_mailbox = *request;
        remote_pending = true;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

static bool TakeRemote(MotorRemoteRequest *request)
{
    taskENTER_CRITICAL();
    bool pending = remote_pending;
    if (pending)
    {
        *request = remote_mailbox;
        remote_pending = false;
    }
    taskEXIT_CRITICAL();
    return pending;
}

static int16_t ToInt16(float value)
{
    return value > INT16_MAX ? INT16_MAX : value < INT16_MIN ? INT16_MIN : (int16_t)lroundf(value);
}

static bool ImuReady(const ImuAppStatus *imu)
{
    return imu->state == IMU_READY && imu->valid &&
           (HAL_GetTick() - imu->timestamp_ms <= IMU_MAX_AGE_MS) &&
           isfinite(imu->yaw_rad) && isfinite(imu->gyro[2]);
}

void MotorApp_GetStatus(MotorAppStatus *status)
{
    taskENTER_CRITICAL();
    *status = published;
    taskEXIT_CRITICAL();
}

static void Publish(const MotorAppStatus *status)
{
    taskENTER_CRITICAL();
    published = *status;
    taskEXIT_CRITICAL();
}

/* Legacy local command entry points are disabled in the ROS handoff firmware. */
static bool PostRequest(const MotorRequest *request)
{
    (void)request;
    return false;
}
bool MotorApp_Request(MotorRequestType type, uint8_t number, int16_t value)
{
    if ((unsigned int)type > (unsigned int)MOTOR_REQUEST_ZERO)
    {
        return false;
    }
    bool drive = (type == MOTOR_REQUEST_TEST) || (type == MOTOR_REQUEST_SPEED);
    int limit = type == MOTOR_REQUEST_SPEED ? 120 : 100;
    if (drive && ((number < 1U) || (number > MOTOR_COUNT) ||
                  (value == 0) || (value < -limit) || (value > limit)))
    {
        return false;
    }
    MotorRequest request = {.type = type, .motor_number = number, .value = value};
    if (type == MOTOR_REQUEST_SPEED)
    {
        request.wheel_rpm[number - 1U] = (float)value;
    }
    return PostRequest(&request);
}

static bool RequestBody(int16_t vx, int16_t vy, int16_t wz, bool streaming)
{
    if ((vx < -800) || (vx > 800) || (vy < -800) || (vy > 800) ||
        (wz < -10000) || (wz > 10000))
    {
        return false;
    }
    if ((vx == 0) && (vy == 0) && (wz == 0))
    {
        return MotorApp_Request(MOTOR_REQUEST_STOP, 0U, 0);
    }
    MotorRequest request = {.type = MOTOR_REQUEST_SPEED, .chassis = true,
                            .streaming = streaming, .body = {vx, vy, wz}};
    return PostRequest(&request);
}

bool MotorApp_RequestMove(int16_t vx, int16_t vy, int16_t wz)
{
    return RequestBody(vx, vy, wz, false);
}

bool MotorApp_RequestVelocity(int16_t vx, int16_t vy, int16_t wz)
{
    return RequestBody(vx, vy, wz, true);
}

bool MotorApp_RequestHeading(int16_t degrees)
{
    if ((degrees < -180) || (degrees > 180))
    {
        return false;
    }
    MotorRequest request = {.type = MOTOR_REQUEST_SPEED, .chassis = true,
                            .heading_move = true, .heading = degrees * PI / 180.0f};
    return PostRequest(&request);
}

static void Stop(MotorAppStatus *status, SpeedPi controllers[MOTOR_COUNT],
                  float targets[MOTOR_COUNT], ChassisYaw *yaw, uint8_t fault)
{
    MotorDriver_StopAll();
    status->running = false;
    status->heading_hold = false;
    status->spin_mode = false;
    status->duty_percent = 0;
    status->target_rpm = 0;
    status->yaw_rate_output_mradps = 0;
    status->fault = fault;
    memset(status->body_command, 0, sizeof(status->body_command));
    for (unsigned int i = 0U; i < MOTOR_COUNT; ++i)
    {
        SpeedPi_Reset(&controllers[i]);
        targets[i] = 0.0f;
        status->wheel_target_x10[i] = 0;
        status->wheel_pwm[i] = 0;
        status->wheel_pwm_permille[i] = 0;
    }
    ChassisYaw_Reset(yaw, 0.0f);
    ++status->transitions;
}

static void SampleFeedback(MotorAppStatus *status, float rpm[MOTOR_COUNT])
{
    Encoder_Sample(status->delta);
    status->timestamp_ms = HAL_GetTick();
    for (unsigned int i = 0U; i < MOTOR_COUNT; ++i)
    {
        if (status->sample_ms != 0U)
        {
            float raw = status->delta[i] * 60000.0f /
                        (CHASSIS_ENCODER_COUNTS_PER_REV * status->sample_ms);
            float alpha = status->sample_ms / (50.0f + status->sample_ms);
            rpm[i] += alpha * (raw - rpm[i]);
            status->rpm_x10[i] = ToInt16(rpm[i] * 10.0f);
        }
        int64_t total = (int64_t)status->test_counts[i] + status->delta[i];
        status->encoder_count[i] += (uint32_t)status->delta[i];
        status->test_counts[i] = total > INT32_MAX ? INT32_MAX :
                               total < INT32_MIN ? INT32_MIN : (int32_t)total;
    }
    float vx, vy, wz;
    Omni_FromWheelRpm(rpm, &vx, &vy, &wz);
    status->body_measured[0] = ToInt16(vx * 1000.0f);
    status->body_measured[1] = ToInt16(vy * 1000.0f);
    status->body_measured[2] = ToInt16(wz * 1000.0f);
}

static void ConfigureBodyRequest(const MotorRequest *request, MotorAppStatus *status,
                                 ChassisYaw *yaw, const ImuAppStatus *imu, bool starting)
{
    if (request->spin && (starting || !status->spin_mode))
    {
        status->spin_reference_rad = imu->yaw_rad;
    }
    status->spin_mode = request->spin;
    if (starting || (status->heading_move && !request->heading_move))
    {
        ChassisYaw_Reset(yaw, imu->yaw_rad);
    }
    if (request->heading_move)
    {
        ChassisYaw_SetHeading(yaw, request->heading);
    }
    status->heading_move = request->heading_move;
    status->velocity_stream = request->streaming;
    memcpy(status->body_command, request->body, sizeof(status->body_command));
}

static bool BeginRequest(const MotorRequest *request, MotorAppStatus *status,
                         SpeedPi controllers[MOTOR_COUNT], float targets[MOTOR_COUNT],
                         ChassisYaw *yaw, const ImuAppStatus *imu)
{
    status->motor_number = request->motor_number;
    status->speed_mode = request->type == MOTOR_REQUEST_SPEED;
    status->chassis_mode = request->chassis;
    status->target_rpm = status->speed_mode && !request->chassis ? request->value : 0;
    status->duty_percent = status->speed_mode ? 0 : request->value;
    status->fault = 0U;
    status->heading_hold = false;
    status->heading_move = false;
    status->velocity_stream = false;
    memset(status->body_command, 0, sizeof(status->body_command));
    ChassisYaw_Reset(yaw, imu->yaw_rad);
    if (request->chassis)
    {
        ConfigureBodyRequest(request, status, yaw, imu, true);
    }
    for (unsigned int i = 0U; i < MOTOR_COUNT; ++i)
    {
        SpeedPi_Reset(&controllers[i]);
        targets[i] = request->wheel_rpm[i];
        status->wheel_target_x10[i] = ToInt16(targets[i] * 10.0f);
        status->wheel_pwm[i] = 0;
    }
    if (status->speed_mode)
    {
        return MotorDriver_BeginWheels(targets);
    }
    status->wheel_pwm[request->motor_number - 1U] = request->value;
    status->wheel_pwm_permille[request->motor_number - 1U] = request->value * 10;
    return MotorDriver_TestSigned(request->motor_number - 1U, request->value);
}

static bool UpdateChassis(MotorAppStatus *status, float targets[MOTOR_COUNT],
                           SpeedPi controllers[MOTOR_COUNT], ChassisYaw *yaw,
                           const ImuAppStatus *imu)
{
    float output;
    if (!ImuReady(imu) || !ChassisYaw_Update(yaw, status->body_command[2] * 0.001f,
        imu->yaw_rad, imu->gyro[2], status->sample_ms * 0.001f, &output))
    {
        return false;
    }
    float next[MOTOR_COUNT];
    float vx = status->body_command[0] * 0.001f;
    float vy = status->body_command[1] * 0.001f;
    if (status->spin_mode)
    {
        Omni_ReferenceToBody(vx, vy, imu->yaw_rad - status->spin_reference_rad, &vx, &vy);
    }
    if (!Omni_ToWheelRpm(vx, vy, output, next))
    {
        return false;
    }
    for (unsigned int i = 0U; i < MOTOR_COUNT; ++i)
    {
        /* Avoid turning gyro noise into repeated low-speed motor reversals. */
        if (fabsf(next[i]) < 0.5f)
        {
            next[i] = 0.0f;
        }
        if ((next[i] * targets[i] <= 0.0f) && (next[i] != targets[i]))
        {
            SpeedPi_Reset(&controllers[i]);
        }
        targets[i] = next[i];
        status->wheel_target_x10[i] = ToInt16(targets[i] * 10.0f);
    }
    status->heading_hold = yaw->holding;
    status->yaw_reference_mrad = ToInt16(yaw->heading_reference * 1000.0f);
    status->yaw_error_mrad = ToInt16(yaw->heading_error * 1000.0f);
    status->yaw_rate_target_mradps = ToInt16(yaw->rate_target * 1000.0f);
    status->yaw_rate_measured_mradps = ToInt16(imu->gyro[2] * 1000.0f);
    status->yaw_rate_output_mradps = ToInt16(output * 1000.0f);
    return true;
}

static void UpdateSpeed(MotorAppStatus *status, SpeedPi controllers[MOTOR_COUNT],
                        const float targets[MOTOR_COUNT], const float rpm[MOTOR_COUNT])
{
    int16_t duty[MOTOR_COUNT];
    int16_t effective[MOTOR_COUNT];
    float previous_integral[MOTOR_COUNT];
    for (unsigned int i = 0U; i < MOTOR_COUNT; ++i)
    {
        previous_integral[i] = controllers[i].integral;
        float sign = targets[i] < 0.0f ? -1.0f : 1.0f;
        /* Coast before reversing a spinning wheel; do not integrate while waiting. */
        if ((targets[i] * rpm[i] < 0.0f) && (fabsf(rpm[i]) > 5.0f))
        {
            SpeedPi_Reset(&controllers[i]);
            duty[i] = 0;
            continue;
        }
        float output = SpeedPi_Update(&controllers[i], fabsf(targets[i]), sign * rpm[i],
                                      status->sample_ms * 0.001f);
        duty[i] = (int16_t)(sign * lroundf(output));
    }
    if (!MotorDriver_ApplySignedDuty(duty, effective))
    {
        Error_Handler();
    }
    for (unsigned int i = 0U; i < MOTOR_COUNT; ++i)
    {
        if ((effective[i] == 0) && (duty[i] != 0))
        {
            controllers[i].integral = previous_integral[i];
        }
        status->wheel_pwm[i] = effective[i] / 10;
        status->wheel_pwm_permille[i] = effective[i];
    }
    if (status->motor_number != 0U)
    {
        status->duty_percent = status->wheel_pwm[status->motor_number - 1U];
    }
}

/* Remote requests are executed only here, in the same task that owns PWM. */
static void ApplyRemote(const MotorRemoteRequest *request, MotorAppStatus *status,
                         const ImuAppStatus *imu, SpeedPi controllers[MOTOR_COUNT],
                         float targets[MOTOR_COUNT], ChassisYaw *yaw,
                         uint32_t *command_at, uint32_t *velocity_at, uint32_t *stopped_at)
{
    uint32_t now = HAL_GetTick();
    uint8_t result = 0U;
    if (!status->remote_mode || request->mode_generation != status->mode_generation)
    {
        result = 8U; /* Physical ROS mode permission is required. */
    }
    else if (request->action <= MOTOR_REMOTE_STOP)
    {
        if (request->action == MOTOR_REMOTE_ARM)
        {
            bool still = !status->running;
            for (unsigned int i = 0U; i < MOTOR_COUNT; ++i)
            {
                still = still && abs(status->rpm_x10[i]) < 50;
            }
            if (!ImuReady(imu))
            {
                result = 3U;
            }
            else if (!still || status->sample_ms > 50U || !status->remote_mode)
            {
                result = 4U;
            }
            else
            {
                status->remote_armed = true;
                status->fault = 0U;
                status->stop_reason = 0U;
                *velocity_at = now;
            }
        }
        else
        {
            Stop(status, controllers, targets, yaw, 0U);
            status->remote_armed = false;
            status->stop_reason = 2U;
            *stopped_at = now;
            *velocity_at = now;
            if (request->action == MOTOR_REMOTE_RESET)
            {
                status->last_applied_cmd_seq = 0U;
            }
        }
    }
    else if (request->action == MOTOR_REMOTE_CALIBRATE)
    {
        result = !status->remote_armed && !status->running && ImuApp_RequestCalibration() ? 0U : 4U;
    }
    else if (!status->remote_armed)
    {
        result = 2U;
    }
    else if (!ImuReady(imu))
    {
        result = 3U;
    }
    else if (request->action == MOTOR_REMOTE_VELOCITY || request->action == MOTOR_REMOTE_SPIN)
    {
        int16_t vx = request->values[0], vy = request->values[1], wz = request->values[2];
        if (abs(vx) > 800 || abs(vy) > 800 || abs(wz) > 10000)
        {
            result = 1U;
        }
        else
        {
            if (vx == 0 && vy == 0 && wz == 0)
            {
                Stop(status, controllers, targets, yaw, 0U);
                status->stop_reason = 1U;
                status->heading_move = false;
                status->velocity_stream = true;
                *stopped_at = now;
            }
            else
            {
                MotorRequest motion = {.type = MOTOR_REQUEST_SPEED, .chassis = true,
                                       .streaming = true, .body = {vx, vy, wz},
                                       .spin = request->action == MOTOR_REMOTE_SPIN};
                if (status->running)
                {
                    ConfigureBodyRequest(&motion, status, yaw, imu, false);
                }
                else if (!BeginRequest(&motion, status, controllers, targets, yaw, imu))
                {
                    Error_Handler();
                }
                status->running = true;
                status->stop_reason = 0U;
            }
            *command_at = now;
            *velocity_at = now;
            status->last_applied_cmd_seq = request->sequence;
        }
    }
    else if (request->action == MOTOR_REMOTE_HEADING)
    {
        if (abs(request->values[0]) > 3142)
        {
            result = 1U;
        }
        else
        {
            MotorRequest motion = {.type = MOTOR_REQUEST_SPEED, .chassis = true,
                                   .heading_move = true, .heading = request->values[0] * 0.001f};
            if (status->running)
            {
                ConfigureBodyRequest(&motion, status, yaw, imu, false);
            }
            else if (!BeginRequest(&motion, status, controllers, targets, yaw, imu))
            {
                Error_Handler();
            }
            status->running = true;
            status->last_applied_cmd_seq = request->sequence;
            status->stop_reason = 0U;
            *command_at = now;
        }
    }
    else
    {
        result = 7U;
    }
    status->remote_ticket = request->ticket;
    status->remote_result = result;
}

static void RemoteTick(MotorAppStatus *status, const ImuAppStatus *imu, bool pressed,
                        SpeedPi controllers[MOTOR_COUNT], float targets[MOTOR_COUNT],
                        const float rpm[MOTOR_COUNT], ChassisYaw *yaw, uint32_t command_at,
                        uint32_t velocity_at, uint32_t *stopped_at, uint32_t *aligned_ms)
{
    uint32_t now = HAL_GetTick();
    uint32_t age = now - velocity_at;
    status->command_age_ms = age > UINT16_MAX ? UINT16_MAX : (uint16_t)age;
    uint8_t reason = pressed ? 4U : status->sample_ms > 50U ? 6U : !ImuReady(imu) ? 5U : 0U;
    if (status->remote_armed && reason == 0U)
    {
        if (status->running && status->heading_move)
        {
            reason = (now - command_at) >= HEADING_DURATION_MS ? 3U : 0U;
        }
        else if (age >= VELOCITY_TIMEOUT_MS)
        {
            reason = 3U;
        }
    }
    if (pressed || (status->remote_armed && reason != 0U))
    {
        uint8_t fault = reason == 5U ? 1U : reason == 6U ? 2U : reason == 3U ? 3U : 0U;
        Stop(status, controllers, targets, yaw, fault);
        status->remote_armed = false;
        status->stop_reason = reason;
        *stopped_at = now;
    }
    if (status->running && status->remote_armed)
    {
        if (!UpdateChassis(status, targets, controllers, yaw, imu))
        {
            Stop(status, controllers, targets, yaw, 1U);
            status->remote_armed = false;
            status->stop_reason = 5U;
            *stopped_at = now;
        }
        else
        {
            bool aligned = status->heading_move && fabsf(yaw->heading_error) < 0.02f &&
                           fabsf(imu->gyro[2]) < 0.03f;
            *aligned_ms = aligned ? *aligned_ms + status->sample_ms : 0U;
            if (*aligned_ms >= 300U)
            {
                Stop(status, controllers, targets, yaw, 0U);
                status->remote_armed = false;
                status->stop_reason = 0U;
                *stopped_at = now;
            }
            else
            {
                UpdateSpeed(status, controllers, targets, rpm);
            }
        }
    }
}

void MotorApp_Run(void)
{
    MotorAppStatus status = {.remote_mode = true};
    SpeedPi controllers[MOTOR_COUNT] = {0};
    ChassisYaw yaw = {0};
    float rpm[MOTOR_COUNT] = {0};
    float targets[MOTOR_COUNT] = {0};
    uint32_t command_at = 0U;
    uint32_t heading_settled_ms = 0U;
    uint32_t remote_velocity_at = 0U;
    uint32_t stopped_at = HAL_GetTick();
    uint32_t previous_sample = stopped_at;
    TickType_t last_wake = xTaskGetTickCount();

    if (!MotorDriver_Init() || !Encoder_Init())
    {
        MotorDriver_StopAll();
        Error_Handler();
    }
    UserKey_Init(HAL_GetTick());
    status.ready = true;
    Publish(&status);

    for (;;)
    {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MOTOR_PERIOD_MS));
        uint32_t now = HAL_GetTick();
        status.sample_ms = now - previous_sample;
        previous_sample = now;
        SampleFeedback(&status, rpm);
        ImuAppStatus imu;
        ImuApp_GetStatus(&imu);
        bool pressed = UserKey_Pressed(now);
        MotorRemoteRequest remote;
        bool has_remote = TakeRemote(&remote);

        /* Physical mode switching takes priority over any received command. */
        if (pressed)
        {
            Stop(&status, controllers, targets, &yaw, 0U);
            status.remote_armed = false;
            status.remote_mode = !status.remote_mode;
            ++status.mode_generation;
            status.stop_reason = 4U;
            status.last_applied_cmd_seq = 0U;
            status.command_age_ms = 0U;
            heading_settled_ms = 0U;
            stopped_at = now;
            remote_velocity_at = now;
            if (has_remote)
            {
                status.remote_ticket = remote.ticket;
                status.remote_result = 8U;
            }
            Publish(&status);
            continue;
        }

        if (has_remote)
        {
            ApplyRemote(&remote, &status, &imu, controllers, targets, &yaw,
                        &command_at, &remote_velocity_at, &stopped_at);
            heading_settled_ms = 0U;
        }
        if (status.remote_mode)
        {
            RemoteTick(&status, &imu, false, controllers, targets, rpm, &yaw,
                       command_at, remote_velocity_at, &stopped_at, &heading_settled_ms);
        }
        status.idle_ms = status.running ? 0U : now - stopped_at;
        Publish(&status);
    }
}
