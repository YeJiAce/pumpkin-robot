/** Line commands run only in commTask; actuator requests go to motorTask. */
#include "command_dispatch.h"
#include "motor_app.h"
#include "serial_dma.h"
#include "imu_app.h"
#include "main.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static bool imu_stream;
static uint32_t imu_reported_at;

static const char *ImuStateName(ImuState state)
{
    switch (state)
    {
        case IMU_WARMUP: return "WARMUP";
        case IMU_CALIBRATING: return "CALIBRATING";
        case IMU_READY: return "READY";
        default: return "OFFLINE";
    }
}

static long MilliUnits(float value)
{
    return isfinite(value) ? lroundf(value * 1000.0f) : 0L;
}

static void ReportImu(bool include_bias)
{
    ImuAppStatus imu;
    char message[192];
    ImuApp_GetStatus(&imu);
    (void)snprintf(message, sizeof(message),
                   "IMU %s valid=%u addr=0x%02X id=0x%02X cal=%u/200 dt=%lu age=%lu seq=%lu errors=%lu stage=%u i2c=%08lX\r\n",
                   ImuStateName(imu.state), (unsigned int)imu.valid,
                   (unsigned int)imu.address, (unsigned int)imu.who_am_i,
                   (unsigned int)imu.calibration_samples, (unsigned long)imu.sample_ms,
                   (unsigned long)(HAL_GetTick() - imu.timestamp_ms),
                   (unsigned long)imu.sequence, (unsigned long)imu.errors,
                   (unsigned int)imu.stage, (unsigned long)imu.i2c_error);
    (void)SerialDma_Write(message);
    if (imu.valid)
    {
        (void)snprintf(message, sizeof(message),
                       "acc_mmps2=%ld,%ld,%ld gyro_mradps=%ld,%ld,%ld yaw_mrad=%ld\r\n",
                       MilliUnits(imu.accel[0]), MilliUnits(imu.accel[1]), MilliUnits(imu.accel[2]),
                       MilliUnits(imu.gyro[0]), MilliUnits(imu.gyro[1]), MilliUnits(imu.gyro[2]),
                       MilliUnits(imu.yaw_rad));
        (void)SerialDma_Write(message);
    }
    if (include_bias)
    {
        (void)snprintf(message, sizeof(message),
                       "gyro_bias_mradps=%ld,%ld,%ld; body +X=IMU+Y,+Y=IMU-X,+Z=IMU+Z\r\n",
                       MilliUnits(imu.gyro_bias[0]), MilliUnits(imu.gyro_bias[1]), MilliUnits(imu.gyro_bias[2]));
        (void)SerialDma_Write(message);
    }
}

void CommandDispatch_Poll(void)
{
    uint32_t now = HAL_GetTick();
    if (imu_stream && ((now - imu_reported_at) >= 200U))
    {
        imu_reported_at = now;
        ReportImu(false);
    }
}

static void SkipSpaces(const char **cursor)
{
    while (**cursor == ' ')
    {
        ++*cursor;
    }
}

/* Bounded integer parser: reject overflow and malformed/trailing tokens. */
static bool ReadInteger(const char **cursor, int *value)
{
    int sign = 1;
    unsigned int magnitude = 0U;
    SkipSpaces(cursor);
    if ((**cursor == '-') || (**cursor == '+'))
    {
        sign = **cursor == '-' ? -1 : 1;
        ++*cursor;
    }
    if ((**cursor < '0') || (**cursor > '9'))
    {
        return false;
    }
    while ((**cursor >= '0') && (**cursor <= '9'))
    {
        magnitude = magnitude * 10U + (unsigned int)(**cursor - '0');
        if (magnitude > 4000U)
        {
            return false;
        }
        ++*cursor;
    }
    *value = sign * (int)magnitude;
    return true;
}

void CommandDispatch_ReportStatus(void)
{
    MotorAppStatus status;
    char message[192];
    MotorApp_GetStatus(&status);
    if (status.chassis_mode)
    {
        (void)snprintf(message, sizeof(message),
                       "BODY %s cmd=%d,%d,%d measured=%d,%d,%d dt=%lu\r\n",
                       status.running ? "RUN" : "STOP",
                       status.body_command[0], status.body_command[1], status.body_command[2],
                       status.body_measured[0], status.body_measured[1], status.body_measured[2],
                       (unsigned long)status.sample_ms);
        (void)SerialDma_Write(message);
        (void)snprintf(message, sizeof(message),
                       "WHEELS target_x10=%d,%d,%d rpm_x10=%d,%d,%d pwm=%d,%d,%d\r\n",
                       status.wheel_target_x10[0], status.wheel_target_x10[1], status.wheel_target_x10[2],
                       status.rpm_x10[0], status.rpm_x10[1], status.rpm_x10[2],
                       status.wheel_pwm[0], status.wheel_pwm[1], status.wheel_pwm[2]);
        (void)SerialDma_Write(message);
        (void)snprintf(message, sizeof(message),
                       "YAW %s ref_mrad=%d error_mrad=%d target_mradps=%d gyro_mradps=%d output_mradps=%d stream=%u fault=%u\r\n",
                       !status.running ? "OFF" : status.heading_hold ? "HOLD" : "RATE",
                       status.yaw_reference_mrad, status.yaw_error_mrad,
                       status.yaw_rate_target_mradps, status.yaw_rate_measured_mradps,
                       status.yaw_rate_output_mradps, (unsigned int)status.velocity_stream,
                       (unsigned int)status.fault);
        (void)SerialDma_Write(message);
        return;
    }
    (void)snprintf(message, sizeof(message),
                   "M%u %s mode=%s target=%d pwm=%d rpm_x10=%d,%d,%d dt=%lu count=%ld,%ld,%ld\r\n",
                   (unsigned int)status.motor_number, status.running ? "RUN" : "STOP",
                   status.speed_mode ? "SPEED" : "PWM", (int)status.target_rpm,
                   (int)status.duty_percent,
                   (int)status.rpm_x10[0], (int)status.rpm_x10[1], (int)status.rpm_x10[2],
                   (unsigned long)status.sample_ms,
                   (long)status.test_counts[0], (long)status.test_counts[1],
                   (long)status.test_counts[2]);
    (void)SerialDma_Write(message);
}

void CommandDispatch_Execute(const char *line)
{
    MotorRequestType type;
    int number = 0;
    int duty = 0;

    if (strcmp(line, "imu") == 0)
    {
        ReportImu(true);
        return;
    }
    if (strcmp(line, "imu on") == 0 || strcmp(line, "imu off") == 0)
    {
        imu_stream = strcmp(line, "imu on") == 0;
        imu_reported_at = HAL_GetTick();
        (void)SerialDma_Write(imu_stream ? "OK IMU stream on (5Hz)\r\n" : "OK IMU stream off\r\n");
        if (imu_stream)
        {
            ReportImu(true);
        }
        return;
    }
    if (strcmp(line, "imu cal") == 0)
    {
        (void)SerialDma_Write(ImuApp_RequestCalibration()
                              ? "OK IMU calibration queued; keep stationary\r\n"
                              : "ERR IMU offline/busy or motors moving\r\n");
        return;
    }

    if (strcmp(line, "ping") == 0)
    {
        (void)SerialDma_Write("pong\r\n");
        return;
    }
    if (strcmp(line, "status") == 0)
    {
        CommandDispatch_ReportStatus();
        return;
    }
    if (strcmp(line, "help") == 0)
    {
        (void)SerialDma_Write("ping | status | zero | stop | motor <1..3> <-100..100,nonzero>\r\n");
        (void)SerialDma_Write("motor: 2s test; wait for STOP then 300ms before next test\r\n");
        (void)SerialDma_Write("speed <1..3> <-120..120,nonzero> RPM: PI test 5s, PWM limited to 60%\r\n");
        (void)SerialDma_Write("move <vx> <vy> <wz>: mm/s,mm/s,mrad/s; limits 800,800,2000; 5s test\r\n");
        (void)SerialDma_Write("cmd <vx> <vy> <wz>: same units, refresh >=10Hz, 500ms watchdog; zero stops\r\n");
        (void)SerialDma_Write("heading <-180..180>: degrees in relative IMU frame, 8s max, auto-stop on arrival\r\n");
        (void)SerialDma_Write("move/cmd require IMU READY; Wz=0 while translating holds heading\r\n");
        (void)SerialDma_Write("body axes: +X forward, +Y left, +yaw CCW; move 100 0 0 = forward\r\n");
        (void)SerialDma_Write("imu | imu on | imu off | imu cal; accel mm/s^2, gyro mrad/s, relative yaw mrad\r\n");
        return;
    }
    if (strncmp(line, "heading ", 8U) == 0)
    {
        const char *cursor = line + 8;
        int heading;
        if (!ReadInteger(&cursor, &heading))
        {
            (void)SerialDma_Write("ERR heading syntax\r\n");
            return;
        }
        SkipSpaces(&cursor);
        if ((*cursor != '\0') || (heading < -180) || (heading > 180))
        {
            (void)SerialDma_Write("ERR heading range/syntax\r\n");
            return;
        }
        (void)SerialDma_Write(MotorApp_RequestHeading((int16_t)heading)
                              ? "OK heading queued\r\n" : "ERR busy/IMU not ready; send imu\r\n");
        return;
    }
    if (strncmp(line, "move ", 5U) == 0 || strncmp(line, "cmd ", 4U) == 0)
    {
        bool streaming = line[0] == 'c';
        const char *cursor = line + (streaming ? 4 : 5);
        int vx, vy, wz;
        if (!ReadInteger(&cursor, &vx) || (*cursor != ' ') ||
            !ReadInteger(&cursor, &vy) || (*cursor != ' ') || !ReadInteger(&cursor, &wz))
        {
            (void)SerialDma_Write("ERR move syntax\r\n");
            return;
        }
        SkipSpaces(&cursor);
        if ((*cursor != '\0') || (vx < -800) || (vx > 800) ||
            (vy < -800) || (vy > 800) || (wz < -2000) || (wz > 2000))
        {
            (void)SerialDma_Write("ERR move range/syntax\r\n");
            return;
        }
        bool accepted = streaming ? MotorApp_RequestVelocity((int16_t)vx, (int16_t)vy, (int16_t)wz) :
                                    MotorApp_RequestMove((int16_t)vx, (int16_t)vy, (int16_t)wz);
        (void)SerialDma_Write(accepted ? "OK queued\r\n" : "ERR busy/IMU not ready; send imu\r\n");
        return;
    }
    if (strcmp(line, "stop") == 0)
    {
        type = MOTOR_REQUEST_STOP;
    }
    else if (strcmp(line, "zero") == 0)
    {
        type = MOTOR_REQUEST_ZERO;
    }
    else if ((strncmp(line, "motor ", 6U) == 0) || (strncmp(line, "speed ", 6U) == 0))
    {
        bool speed_mode = line[0] == 's';
        int limit = speed_mode ? 120 : 100;
        const char *cursor = line + 6;
        if (!ReadInteger(&cursor, &number) || (*cursor != ' ') ||
            !ReadInteger(&cursor, &duty))
        {
            (void)SerialDma_Write("ERR motor syntax\r\n");
            return;
        }
        SkipSpaces(&cursor);
        if ((*cursor != '\0') || (number < 1) || (number > 3) ||
            (duty == 0) || (duty < -limit) || (duty > limit))
        {
            (void)SerialDma_Write("ERR motor range/syntax\r\n");
            return;
        }
        type = speed_mode ? MOTOR_REQUEST_SPEED : MOTOR_REQUEST_TEST;
    }
    else
    {
        (void)SerialDma_Write("ERR unknown command; send help\r\n");
        return;
    }

    (void)SerialDma_Write(MotorApp_Request(type, (uint8_t)number, (int16_t)duty)
                          ? "OK queued\r\n" : "ERR busy/not ready; stop and retry\r\n");
}
